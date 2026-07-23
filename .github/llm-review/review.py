#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
# SPDX-License-Identifier: MPL-2.0
"""
Multi-stage LLM PR reviewer for DPsim, driven by an OpenAI-compatible endpoint
(KI:connect / RWTHgpt). Pure stdlib, no pip installs needed on the runner.

Flow: read the base..head diff, run each stage in prompts.STAGES against the
model, run a synthesis pass, then post one non-blocking COMMENT PR review with
inline comments anchored only to lines present in the diff.

Env:
  LLM_BASE_URL   e.g. https://chat.kiconnect.nrw/api/v1   (trailing / ok)
  LLM_CHAT_PATH  chat path appended to the base, default /chat/completions
  LLM_API_KEY    Bearer API key
  LLM_MODEL      e.g. mistral-small-4-119b-2603
  GITHUB_TOKEN   provided by Actions
  GITHUB_REPOSITORY  provided by Actions (owner/repo)
  PR_NUMBER, BASE_SHA, HEAD_SHA  passed by the workflow; if absent they are read
                 from GITHUB_EVENT_PATH (pull_request event payload)
  MAX_DIFF_CHARS optional cap on diff sent to the model (default 60000)
"""
import json
import os
import re
import subprocess
import sys
import urllib.request
import urllib.error

# Make `import prompts` work no matter the current working directory.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import prompts  # noqa: E402

SEV_RANK = {"critical": 0, "high": 1, "medium": 2, "low": 3}
CONF_RANK = {"high": 0, "medium": 1, "low": 2}


def env(name, default=None, required=False):
    v = os.environ.get(name, default)
    if required and not v:
        sys.exit(f"missing required env var: {name}")
    return v


def get_pr():
    """Return (number, base_sha, head_sha).

    Prefer the explicit env vars the workflow passes; fall back to the Actions
    pull_request event payload so the script also works if run bare.
    """
    number = os.environ.get("PR_NUMBER")
    base = os.environ.get("BASE_SHA")
    head = os.environ.get("HEAD_SHA")
    if number and base and head:
        return int(number), base, head

    path = os.environ.get("GITHUB_EVENT_PATH")
    if path and os.path.exists(path):
        with open(path) as f:
            event = json.load(f)
        pr = event.get("pull_request")
        if pr:
            return pr["number"], pr["base"]["sha"], pr["head"]["sha"]
    sys.exit(
        "could not determine PR (need PR_NUMBER/BASE_SHA/HEAD_SHA or a "
        "pull_request event); nothing to review"
    )


def api_pr_diff(pr_number):
    """Fetch the PR unified diff via the GitHub API.

    Used in the workflow_run (fork-PR) context, where the PR head is not checked
    out. Reads the diff as data only; it never executes PR code.
    """
    repo = env("GITHUB_REPOSITORY", required=True)
    token = env("GITHUB_TOKEN", required=True)
    url = f"https://api.github.com/repos/{repo}/pulls/{pr_number}"
    req = urllib.request.Request(
        url,
        headers={
            "Authorization": "Bearer " + token,
            "Accept": "application/vnd.github.v3.diff",
        },
    )
    with urllib.request.urlopen(req, timeout=60) as resp:
        return resp.read().decode(errors="replace")


def get_diff(pr_number, base_sha, head_sha):
    # In a workflow_run run the PR branch is not present locally, so fetch the
    # diff via the API. Otherwise (same-repo PR, local dry-run) use git.
    if os.environ.get("USE_API_DIFF"):
        out = api_pr_diff(pr_number)
    else:
        out = subprocess.run(
            ["git", "diff", "--unified=3", f"{base_sha}...{head_sha}"],
            capture_output=True,
            text=True,
            check=True,
        ).stdout
    cap = int(env("MAX_DIFF_CHARS", "60000"))
    if len(out) > cap:
        out = out[:cap] + "\n\n[diff truncated for length]\n"
    return out


def changed_files(diff):
    # Derive the changed-file set from the diff text itself, so it works in both
    # the git and the API path. Only files with a "+++ b/<path>" header (i.e.
    # not pure deletions) can carry an inline comment on the RIGHT side.
    files = set()
    for line in diff.splitlines():
        if line.startswith("+++ b/"):
            path = line[len("+++ b/") :].strip()
            if path and path != "/dev/null":
                files.add(path)
    return files


def llm(system, user):
    """One OpenAI-compatible chat completion. Returns the assistant text."""
    base = env("LLM_BASE_URL", required=True).rstrip("/")
    # <base> already ends in /api/v1 for KI:connect, so the chat path is just
    # /chat/completions. Override LLM_CHAT_PATH if the gateway differs.
    path = env("LLM_CHAT_PATH", "/chat/completions")
    url = base + path
    body = json.dumps(
        {
            "model": env("LLM_MODEL", required=True),
            "temperature": 0.1,
            "messages": [
                {"role": "system", "content": system},
                {"role": "user", "content": user},
            ],
        }
    ).encode()
    req = urllib.request.Request(
        url,
        data=body,
        method="POST",
        headers={
            "Content-Type": "application/json",
            "Authorization": "Bearer " + env("LLM_API_KEY", required=True),
        },
    )
    with urllib.request.urlopen(req, timeout=300) as resp:
        data = json.load(resp)
    return data["choices"][0]["message"]["content"]


def strip_reasoning(text):
    """Remove <think>...</think> reasoning blocks (this is a reasoning model).

    A trace can contain braces that would confuse the outermost-object grab, so
    drop the reasoning before extracting JSON. Also handle an unclosed <think>
    where only the opening tag is present.
    """
    t = re.sub(r"(?is)<think>.*?</think>", "", text)
    if "<think>" in t and "</think>" not in t:
        t = t.split("<think>", 1)[0]
    return t


def parse_findings(text):
    """Tolerant JSON extraction: strip reasoning + fences, grab outermost object."""
    t = strip_reasoning(text).strip()
    if t.startswith("```"):
        t = t.split("```", 2)[1]
        if t.lstrip().lower().startswith("json"):
            t = t.lstrip()[4:]
    start, end = t.find("{"), t.rfind("}")
    if start == -1 or end == -1:
        return []
    try:
        obj = json.loads(t[start : end + 1])
    except json.JSONDecodeError:
        return []
    findings = obj.get("findings", [])
    return findings if isinstance(findings, list) else []


def run_stages(diff):
    all_findings = []
    for stage in prompts.STAGES:
        user = f"{stage['prompt']}\n\n=== PR DIFF (base..head) ===\n{diff}"
        try:
            raw = llm(prompts.SYSTEM, user)
        except (urllib.error.URLError, KeyError, ValueError) as e:
            # Non-blocking: one failed stage must not sink the whole review.
            print(f"[{stage['id']}] stage failed: {e}", file=sys.stderr)
            continue
        found = parse_findings(raw)
        for f in found:
            f["stage"] = stage["id"]
        print(f"[{stage['id']}] {len(found)} finding(s)", file=sys.stderr)
        all_findings.extend(found)
    return all_findings


def synthesize(findings):
    if not findings:
        return []
    try:
        raw = llm(prompts.SYNTHESIS, json.dumps({"findings": findings}, indent=2))
    except (urllib.error.URLError, KeyError, ValueError) as e:
        print(f"synthesis failed, using raw union: {e}", file=sys.stderr)
        return findings
    merged = parse_findings(raw)
    return merged or findings  # fall back to raw union if synthesis misbehaves


def render_body(f):
    sev = f.get("severity", "low").upper()
    body = f"**[{sev}] {f.get('title', '')}**\n\n{f.get('detail', '')}"
    if f.get("suggestion"):
        body += f"\n\n_Suggested fix:_ {f['suggestion']}"
    body += (
        f"\n\n<sub>stage: {f.get('stage', '?')} · "
        f"confidence: {f.get('confidence', '?')}</sub>"
    )
    return body


def build_review(findings, head_sha, valid_files):
    """Assemble the GitHub review payload (summary body + inline comments)."""
    findings.sort(
        key=lambda f: (
            SEV_RANK.get(f.get("severity"), 9),
            CONF_RANK.get(f.get("confidence"), 9),
        )
    )

    inline, general = [], []
    for f in findings:
        body = render_body(f)
        # Only attach inline comments to files+lines actually in the diff,
        # otherwise the GitHub review API rejects the whole submission.
        if f.get("file") in valid_files and isinstance(f.get("line"), int):
            inline.append(
                {"path": f["file"], "line": f["line"], "side": "RIGHT", "body": body}
            )
        else:
            loc = f.get("file", "general")
            line0 = body.splitlines()[0]
            general.append(f"- {line0}  ({loc})")

    counts = {}
    for f in findings:
        s = f.get("severity", "low")
        counts[s] = counts.get(s, 0) + 1
    summary = "### DPsim LLM review\n"
    if findings:
        summary += (
            "Found: "
            + ", ".join(
                f"{n} {s}"
                for s, n in sorted(counts.items(), key=lambda x: SEV_RANK.get(x[0], 9))
            )
            + ".\n"
        )
    else:
        summary += "No issues surfaced by the automated passes.\n"
    if general:
        summary += "\n**Findings without a diff line:**\n" + "\n".join(general)
    model = os.environ.get("LLM_MODEL", "LLM")
    summary += (
        "\n\n<sub>Automated, non-blocking review. May be wrong. "
        f"Model: {model}.</sub>"
    )

    payload = {
        "commit_id": head_sha,
        "event": "COMMENT",
        "body": summary,
        "comments": inline,
    }
    return payload


def gh_post_review(pr_number, payload):
    repo = env("GITHUB_REPOSITORY", required=True)
    token = env("GITHUB_TOKEN", required=True)
    summary = payload["body"]
    url = f"https://api.github.com/repos/{repo}/pulls/{pr_number}/reviews"
    req = urllib.request.Request(
        url,
        data=json.dumps(payload).encode(),
        method="POST",
        headers={
            "Authorization": "Bearer " + token,
            "Accept": "application/vnd.github+json",
            "Content-Type": "application/json",
        },
    )
    try:
        with urllib.request.urlopen(req, timeout=60) as resp:
            print(f"posted review, HTTP {resp.status}", file=sys.stderr)
    except urllib.error.HTTPError as e:
        # Fall back to a plain issue comment if inline anchoring is rejected.
        print(
            f"review POST failed HTTP {e.code}: "
            f"{e.read().decode(errors='replace')[:300]}",
            file=sys.stderr,
        )
        icu = f"https://api.github.com/repos/{repo}/issues/{pr_number}/comments"
        ireq = urllib.request.Request(
            icu,
            data=json.dumps({"body": summary}).encode(),
            method="POST",
            headers={
                "Authorization": "Bearer " + token,
                "Accept": "application/vnd.github+json",
                "Content-Type": "application/json",
            },
        )
        with urllib.request.urlopen(ireq, timeout=60) as resp:
            print(f"posted fallback issue comment, HTTP {resp.status}", file=sys.stderr)


def main():
    # DRY_RUN (or --dry-run) runs the full pipeline against the endpoint but
    # prints the assembled review instead of posting it, and needs no
    # GITHUB_TOKEN. Handy for local testing without any Actions runner.
    dry_run = os.environ.get("DRY_RUN") or "--dry-run" in sys.argv

    pr_number, base_sha, head_sha = get_pr()
    diff = get_diff(pr_number, base_sha, head_sha)
    if not diff.strip():
        print("empty diff, nothing to review", file=sys.stderr)
        return
    valid_files = changed_files(diff)
    findings = synthesize(run_stages(diff))
    payload = build_review(findings, head_sha, valid_files)

    if dry_run:
        print("=== DRY RUN: review body ===\n")
        print(payload["body"])
        print(f"\n=== {len(payload['comments'])} inline comment(s) ===")
        for c in payload["comments"]:
            print(f"\n--- {c['path']}:{c['line']} ---")
            print(c["body"])
        return

    gh_post_review(pr_number, payload)


if __name__ == "__main__":
    main()
