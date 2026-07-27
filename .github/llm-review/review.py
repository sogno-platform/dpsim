#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
# SPDX-License-Identifier: MPL-2.0
"""
Multi-stage LLM PR reviewer for DPsim, driven by an OpenAI-compatible endpoint
(KI:connect / RWTHgpt). Pure stdlib, no pip installs needed on the runner.

Flow: diff + full source of changed files -> STAGES (finders, on one or two
cheap/unlimited models) -> dedup -> per-file verification against the real code
(mid tier) -> optional final re-check of the survivors (strongest tier) ->
synthesis -> one non-blocking COMMENT review with inline suggestions.

The finder models are meant to over-generate; precision is bought by grounding
every candidate against the real source in the verify tiers, escalating only the
survivors to the strongest (and most rate-limited) model. See main().

Env:
  LLM_BASE_URL   e.g. https://chat.kiconnect.nrw/api/v1   (trailing / ok)
  LLM_CHAT_PATH  chat path appended to the base, default /chat/completions
  LLM_API_KEY    Bearer API key
  LLM_MODEL      finder model, cheap/unlimited (e.g. mistral-small-4-119b-2603)
  LLM_MODEL_2    optional second finder model, for cross-model recall
  LLM_VERIFY_MODEL  optional mid-tier verifier (falls back to LLM_MODEL)
  LLM_FINAL_MODEL   optional strongest tier; re-checks only survivors
  LLM_FINDER_CONCURRENCY  parallel finder calls, default 4 (finders are unlimited)
  GITHUB_TOKEN   provided by Actions
  GITHUB_REPOSITORY  provided by Actions (owner/repo)
  PR_NUMBER, BASE_SHA, HEAD_SHA  passed by the workflow; if absent they are read
                 from GITHUB_EVENT_PATH (pull_request event payload)
  MAX_DIFF_CHARS optional cap on diff sent to the model (default 60000)
"""
import concurrent.futures
import json
import os
import random
import re
import subprocess
import sys
import time
import urllib.request
import urllib.error
import urllib.parse

# Make `import prompts` work no matter the current working directory.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import prompts  # noqa: E402

SEV_RANK = {"critical": 0, "high": 1, "medium": 2, "low": 3}

# Applyable ```suggestion blocks are emitted only for high-confidence, confirmed
# findings, so a one-click "Commit suggestion" cannot apply a shaky guess; a
# lower-confidence finding still gets its prose fix. Inline comments are capped so
# a bad run cannot carpet a PR.
SUGGESTION_MIN_CONF = 80
MAX_INLINE = 8

# Finder calls run serially by default: the gateway caps concurrent requests per
# key (429 too_many_concurrent_requests), so parallelism isn't reliably available.
# Raise LLM_FINDER_CONCURRENCY if your gateway allows it; 429s back off and retry.
FINDER_CONCURRENCY = int(os.environ.get("LLM_FINDER_CONCURRENCY", "1"))
# Per-call retry budget for transient throttling (429) and 5xx, with backoff.
MAX_LLM_ATTEMPTS = 6

# Models that 400 on a `temperature` field (GPT-5.x lock it); filled on first
# rejection so later calls skip the param.
_NO_TEMPERATURE = set()


def confidence_display(f):
    """What to show for a finding's confidence: "85%" when the model supplied a
    number, the word when a stage returned one instead, or None. We never invent
    a number from a categorical word; if there is no real value, show nothing."""
    c = f.get("confidence")
    if isinstance(c, (int, float)):
        return f"{int(round(c))}%"
    if isinstance(c, str) and c.strip():
        return c.strip()
    return None


def confidence_sort_key(f):
    """Ordering only (never shown): higher confidence first. A word maps to a
    rough rank so mixed number/word outputs still sort sensibly."""
    c = f.get("confidence")
    if isinstance(c, (int, float)):
        return -c
    return {"high": -90, "medium": -60, "low": -30}.get(str(c).lower(), 0.0)


def is_low_confidence(f):
    """Whether a finding reads as tentative, from a number (<50) or the word."""
    c = f.get("confidence")
    if isinstance(c, (int, float)):
        return c < 50
    return isinstance(c, str) and c.strip().lower() == "low"


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

    GitHub returns HTTP 406 for the .diff media type once a PR's diff is too
    large (very many files / lines). In that case fall back to reconstructing a
    unified diff from the paginated files endpoint, which never 406s.
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
    try:
        with urllib.request.urlopen(req, timeout=60) as resp:
            return resp.read().decode(errors="replace")
    except urllib.error.HTTPError as e:
        if e.code != 406:
            raise
        print(
            "diff too large for the .diff endpoint (HTTP 406); "
            "reconstructing from the files API",
            file=sys.stderr,
        )
        return _diff_from_files(pr_number, repo, token)


def _diff_from_files(pr_number, repo, token):
    """Rebuild a unified diff from GitHub's per-file `patch` fields.

    Emits the `diff --git` / `---` / `+++` headers the rest of the pipeline keys
    off (changed_files, file_hunk, added_lines) followed by each file's patch
    hunks. Files GitHub omits a patch for (binary, or individually too large) are
    skipped and counted; they simply carry no inline comments.
    """
    parts, skipped, page = [], 0, 1
    while True:
        url = (
            f"https://api.github.com/repos/{repo}/pulls/{pr_number}"
            f"/files?per_page=100&page={page}"
        )
        files = gh_get_json(url, token)
        if not files:
            break
        for f in files:
            new = f.get("filename", "")
            status = f.get("status", "")
            old = f.get("previous_filename", new)
            patch = f.get("patch")
            if not patch:
                skipped += 1
                continue
            old_h = "/dev/null" if status == "added" else f"a/{old}"
            new_h = "/dev/null" if status == "removed" else f"b/{new}"
            parts.append(f"diff --git a/{old} b/{new}\n")
            parts.append(f"--- {old_h}\n")
            parts.append(f"+++ {new_h}\n")
            parts.append(patch if patch.endswith("\n") else patch + "\n")
        if len(files) < 100:
            break
        page += 1
    if skipped:
        print(
            f"files API: {skipped} file(s) had no patch (binary/oversized), "
            "skipped for inline review",
            file=sys.stderr,
        )
    return "".join(parts)


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


def llm(system, user, model=None):
    """One OpenAI-compatible chat completion. Returns the assistant text.

    model selects the endpoint model for this call; None falls back to LLM_MODEL.
    This is how the pipeline runs cheap unlimited models for the finder passes and
    escalates verification to a stronger, rate-limited one (see main)."""
    base = env("LLM_BASE_URL", required=True).rstrip("/")
    # <base> already ends in /api/v1 for KI:connect, so the chat path is just
    # /chat/completions. Override LLM_CHAT_PATH if the gateway differs.
    path = env("LLM_CHAT_PATH", "/chat/completions")
    url = base + path
    model = model or env("LLM_MODEL", required=True)
    payload = {
        "model": model,
        "messages": [
            {"role": "system", "content": system},
            {"role": "user", "content": user},
        ],
    }
    if model not in _NO_TEMPERATURE:
        payload["temperature"] = 0.1

    def post(p):
        req = urllib.request.Request(
            url,
            data=json.dumps(p).encode(),
            method="POST",
            headers={
                "Content-Type": "application/json",
                "Authorization": "Bearer " + env("LLM_API_KEY", required=True),
            },
        )
        with urllib.request.urlopen(req, timeout=300) as resp:
            return json.load(resp)

    last_detail = ""
    for attempt in range(MAX_LLM_ATTEMPTS):
        try:
            data = post(payload)
            return data["choices"][0]["message"]["content"]
        except urllib.error.HTTPError as e:
            detail = ""
            try:
                detail = e.read().decode(errors="replace")[:600]
            except Exception:
                pass
            last_detail = detail or last_detail
            # GPT-5.x reject a temperature field with 400; drop it and retry.
            if e.code == 400 and "temperature" in payload:
                payload.pop("temperature", None)
                _NO_TEMPERATURE.add(model)
                print(
                    f"[{model}] rejected temperature; retrying without it",
                    file=sys.stderr,
                )
                continue
            # Concurrency cap / transient upstream errors: back off and retry.
            if e.code in (429, 500, 502, 503) and attempt < MAX_LLM_ATTEMPTS - 1:
                delay = random.uniform(0.5, min(30.0, 1.5 ** (attempt + 1)))
                print(
                    f"[{model}] HTTP {e.code}; backing off {delay:.1f}s "
                    f"(attempt {attempt + 1}/{MAX_LLM_ATTEMPTS})",
                    file=sys.stderr,
                )
                time.sleep(delay)
                continue
            raise urllib.error.URLError(
                f"HTTP {e.code} from model {model}: {detail}"
            ) from e
        except urllib.error.URLError:
            # Network hiccup (not an HTTPError): a few backoff retries.
            if attempt < MAX_LLM_ATTEMPTS - 1:
                time.sleep(random.uniform(0.5, 1.5 ** (attempt + 1)))
                continue
            raise
    raise urllib.error.URLError(
        f"exhausted {MAX_LLM_ATTEMPTS} attempts for model {model}: {last_detail}"
    )


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


def extract_object(text):
    """Tolerant JSON-object extraction: strip reasoning + fences, grab the
    outermost {...}. Returns the parsed dict, or None if nothing parseable."""
    t = strip_reasoning(text).strip()
    if t.startswith("```"):
        t = t.split("```", 2)[1]
        if t.lstrip().lower().startswith("json"):
            t = t.lstrip()[4:]
    start, end = t.find("{"), t.rfind("}")
    if start == -1 or end == -1:
        return None
    try:
        obj = json.loads(t[start : end + 1])
    except json.JSONDecodeError:
        return None
    return obj if isinstance(obj, dict) else None


def extract_findings(text):
    """Findings list on a successful parse (possibly empty, i.e. the model
    deliberately returned none), or None when the output could not be parsed.

    Callers that must tell "nothing found" apart from "parse failed" (e.g. the
    verifier) rely on this distinction; parse_findings() collapses both to an
    empty list for the stage path.
    """
    obj = extract_object(text)
    if obj is None:
        return None
    findings = obj.get("findings", [])
    return findings if isinstance(findings, list) else None


def parse_findings(text):
    """Findings list, empty on either no-findings or parse failure."""
    return extract_findings(text) or []


def gh_get_json(url, token):
    """GET a GitHub API endpoint as JSON."""
    req = urllib.request.Request(
        url,
        headers={
            "Authorization": "Bearer " + token,
            "Accept": "application/vnd.github+json",
        },
    )
    with urllib.request.urlopen(req, timeout=60) as resp:
        return json.load(resp)


def pr_context(pr_number, base_sha, head_sha):
    """PR title, description and commit messages, so the review can understand
    the author's intent and check that what the PR claims matches the code.

    API in the fork path (USE_API_DIFF), git log locally. Best-effort: returns ""
    on any failure. Read-only, data only.
    """
    lines = []
    if os.environ.get("USE_API_DIFF"):
        repo = os.environ.get("GITHUB_REPOSITORY")
        token = os.environ.get("GITHUB_TOKEN")
        if repo and token:
            try:
                pr = gh_get_json(
                    f"https://api.github.com/repos/{repo}/pulls/{pr_number}", token
                )
                lines.append(f"PR #{pr_number}: {pr.get('title', '')}".strip())
                body = (pr.get("body") or "").strip()
                if body:
                    lines.append(body)
            except (urllib.error.URLError, ValueError, KeyError) as e:
                print(f"pr_context: PR fetch failed: {e}", file=sys.stderr)
            try:
                commits = gh_get_json(
                    f"https://api.github.com/repos/{repo}/pulls/{pr_number}"
                    "/commits?per_page=100",
                    token,
                )
                for c in commits or []:
                    msg = (c.get("commit", {}).get("message") or "").strip()
                    if msg:
                        lines.append("commit: " + msg)
            except (urllib.error.URLError, ValueError, KeyError) as e:
                print(f"pr_context: commits fetch failed: {e}", file=sys.stderr)
    else:
        r = subprocess.run(
            ["git", "log", f"{base_sha}..{head_sha}", "--format=%B%x00"],
            capture_output=True,
            text=True,
        )
        if r.returncode == 0:
            for msg in r.stdout.split("\x00"):
                msg = msg.strip()
                if msg:
                    lines.append("commit: " + msg)
    if not lines:
        return ""
    text = "\n\n".join(lines)
    if len(text) > MAX_PR_CONTEXT_CHARS:
        text = text[:MAX_PR_CONTEXT_CHARS] + "\n[truncated]"
    return (
        "\n\n=== PR INTENT (title, description, commit messages) ===\n"
        "Use this to understand the change and to check the code does what the PR "
        "claims; it is context, NOT a spec that excuses a real bug.\n" + text
    )


def changed_sources_block(diff, head_sha):
    """Numbered full source of the changed files, as context for every stage.

    Lets the finders see beyond the diff window (a present override, a base
    method) so they stop raising "missing X". Unfetchable/oversized are omitted.
    """
    files = sorted(changed_files(diff))
    cache, blocks, total, dropped = {}, [], 0, 0
    for path in files:
        if len(blocks) >= MAX_STAGE_SOURCE_FILES or total >= MAX_STAGE_SOURCE_CHARS:
            dropped += 1
            continue
        content = fetch_file(path, head_sha)
        if content is None:
            continue
        cache[path] = content
        blocks.append(f"\n--- {path} ---\n{number_lines(content)}\n")
        total += len(content)
    if dropped:
        print(
            f"stage context: {dropped} changed file(s) omitted past the cap",
            file=sys.stderr,
        )
    if not blocks:
        return ""
    return (
        "\n\n=== FULL SOURCE OF CHANGED FILES (context to judge the changed "
        "lines; do NOT raise findings about unchanged code) ===\n" + "".join(blocks)
    )


def run_stages(diff, head_sha, pr_ctx="", models=None):
    # Every (finder model, stage) is an independent call; the finder models are
    # unlimited, so fan them out over a small thread pool (I/O-bound HTTP).
    models = models or [None]
    context = changed_sources_block(diff, head_sha)

    def one(model, stage):
        label = model or env("LLM_MODEL", required=True)
        user = (
            f"{stage['prompt']}\n\n=== PR DIFF (base..head) ===\n"
            f"{diff}{context}{pr_ctx}"
        )
        try:
            raw = llm(prompts.SYSTEM, user, model=model)
        except (urllib.error.URLError, KeyError, ValueError) as e:
            # Non-blocking: one failed stage must not sink the whole review.
            print(f"[{label}/{stage['id']}] stage failed: {e}", file=sys.stderr)
            return []
        found = parse_findings(raw)
        for f in found:
            f["stage"] = stage["id"]
            f["finder"] = label
        print(f"[{label}/{stage['id']}] {len(found)} finding(s)", file=sys.stderr)
        return found

    tasks = [(m, stage) for m in models for stage in prompts.STAGES]
    workers = max(1, min(FINDER_CONCURRENCY, len(tasks)))
    all_findings = []
    with concurrent.futures.ThreadPoolExecutor(max_workers=workers) as ex:
        for found in ex.map(lambda t: one(*t), tasks):
            all_findings.extend(found)
    return all_findings


# Verification fetches+checks at most this many files (one LLM call each); the
# The mid tier verifies every file with findings (per-file, focused) -- it does
# the real culling and must not miss a file. The final tier is a single batched
# call over the survivors, bounded by context size instead of file count.
MAX_FINAL_SOURCE_CHARS = 200000
MAX_VERIFY_FILE_CHARS = 90000
UNCONFIRMED_CONF_CAP = 35
# Base-class/interface headers given to the verifier, followed up the #include
# chain (3 hops: .cpp -> .h -> base -> base's base). Bounded, cached per run.
MAX_CONTEXT_HEADERS = 12
MAX_CONTEXT_CHARS = 80000
CONTEXT_HOPS = 3
# Full source of the changed files given to the finder stages (context beyond
# the diff, to cut "missing X" false positives). Bounded.
MAX_STAGE_SOURCE_FILES = 20
MAX_STAGE_SOURCE_CHARS = 300000
# PR title/description/commit messages given as intent context. Bounded.
MAX_PR_CONTEXT_CHARS = 8000


def fetch_file(path, ref):
    """Full source of `path` at `ref`, or None (read-only, never runs PR code).

    Contents API in the fork path (USE_API_DIFF), git locally. Oversized -> None.
    """
    if os.environ.get("USE_API_DIFF"):
        repo = env("GITHUB_REPOSITORY", required=True)
        token = env("GITHUB_TOKEN", required=True)
        url = (
            f"https://api.github.com/repos/{repo}/contents/"
            f"{urllib.parse.quote(path)}?ref={urllib.parse.quote(ref)}"
        )
        req = urllib.request.Request(
            url,
            headers={
                "Authorization": "Bearer " + token,
                "Accept": "application/vnd.github.raw+json",
            },
        )
        try:
            with urllib.request.urlopen(req, timeout=60) as resp:
                content = resp.read().decode(errors="replace")
        except urllib.error.URLError as e:
            print(f"fetch {path} failed: {e}", file=sys.stderr)
            return None
    else:
        r = subprocess.run(
            ["git", "show", f"{ref}:{path}"], capture_output=True, text=True
        )
        if r.returncode != 0:
            return None
        content = r.stdout
    if len(content) > MAX_VERIFY_FILE_CHARS:
        return None
    return content


def number_lines(content):
    """Prefix each line with its 1-based number for the verifier to cite."""
    return "\n".join(
        f"{i:5d}| {ln}" for i, ln in enumerate(content.splitlines(), start=1)
    )


def file_hunk(diff, path):
    """The section of the unified diff that belongs to `path` (for scope)."""
    out, capture = [], False
    for ln in diff.splitlines(keepends=True):
        if ln.startswith("diff --git "):
            capture = f" a/{path} " in ln or ln.rstrip().endswith(f" b/{path}")
        if capture:
            out.append(ln)
    return "".join(out)


def added_lines(diff, path):
    """New-file line numbers the diff adds/changes for `path` (its scope)."""
    added, new_ln = set(), 0
    for ln in file_hunk(diff, path).splitlines():
        if ln.startswith("@@"):
            m = re.search(r"\+(\d+)", ln)
            new_ln = int(m.group(1)) if m else 0
        elif ln.startswith(("+++", "---")):
            continue
        elif ln.startswith("+"):
            added.add(new_ln)
            new_ln += 1
        elif ln.startswith("-"):
            continue
        else:
            new_ln += 1
    return added


def to_ranges(nums):
    """Compact "1-4, 9, 20-22" rendering of a set of line numbers."""
    nums = sorted(nums)
    if not nums:
        return "(none)"
    out, start, prev = [], nums[0], nums[0]
    for n in nums[1:]:
        if n == prev + 1:
            prev = n
            continue
        out.append(f"{start}-{prev}" if start != prev else f"{start}")
        start = prev = n
    out.append(f"{start}-{prev}" if start != prev else f"{start}")
    return ", ".join(out)


INCLUDE_RE = re.compile(r'#\s*include\s*[<"]((?:dpsim-models|dpsim)/[^">]+\.h)[>"]')


def resolve_include(inc):
    """DPsim #include target -> repo path, or None for externals.
    e.g. dpsim-models/X.h -> dpsim-models/include/dpsim-models/X.h"""
    for pkg in ("dpsim-models/", "dpsim/"):
        if inc.startswith(pkg):
            return pkg + "include/" + inc
    return None


def dpsim_includes(code):
    """Repo paths of the DPsim (non-external) headers a source #includes."""
    out = []
    for m in INCLUDE_RE.finditer(code):
        p = resolve_include(m.group(1))
        if p:
            out.append(p)
    return out


def gather_context(code, ref, cache):
    """Base-class/interface headers this file depends on, following its #include
    chain up to CONTEXT_HOPS hops. Bounded, cached across the run. These carry the
    parent hook declarations that override/signature claims must be judged against."""
    ctx, total = {}, 0
    frontier = dpsim_includes(code)
    for _hop in range(CONTEXT_HOPS):
        nxt = []
        for p in frontier:
            if len(ctx) >= MAX_CONTEXT_HEADERS or total >= MAX_CONTEXT_CHARS:
                break
            if p in ctx:
                continue
            if p not in cache:
                cache[p] = fetch_file(p, ref)
            content = cache[p]
            if content is None:
                continue
            ctx[p] = content
            total += len(content)
            nxt.extend(dpsim_includes(content))
        frontier = nxt
    return ctx


def dedup_findings(findings):
    """Collapse exact-duplicate findings (same file, line, normalized title)
    before verification. Keeps the copy with a code_suggestion, else the first.
    Semantic merging is synthesis's job."""
    seen = {}
    for f in findings:
        key = (
            f.get("file"),
            f.get("line"),
            " ".join(str(f.get("title", "")).lower().split()),
        )
        cur = seen.get(key)
        if cur is None:
            f["finders"] = sorted({f.get("finder")} - {None})
            seen[key] = f
        else:
            # Same finding from another finder model: record the agreement, and
            # prefer the copy carrying a code_suggestion.
            agreed = set(cur.get("finders", [])) | ({f.get("finder")} - {None})
            keep = (
                f
                if (not cur.get("code_suggestion") and f.get("code_suggestion"))
                else cur
            )
            keep["finders"] = sorted(agreed)
            seen[key] = keep
    return list(seen.values())


def verify(findings, diff, head_sha, model=None, max_files=None, single_call=False):
    """Adjudicate findings per file against the real source (code as truth):
    confirmed -> kept, refuted -> dropped, unconfirmed -> kept tentative with
    confidence capped. Nothing drops without a refutation; unreachable code or an
    omitted verdict defaults to unconfirmed. Non-blocking on per-file failure.
    Returns (kept, stats) where stats carries the raw/kept/refuted counts and a
    sample of refuted findings with reasons, for the methodology footer."""
    if not findings:
        return [], {}
    vlabel = model or env("LLM_MODEL", required=True)
    # Finders sometimes cite a wrong path (src/ vs include/). Remap to the real
    # changed file by basename so verification fetches the true source instead of
    # failing and defaulting everything to unconfirmed.
    real_paths = changed_files(diff)
    by_base = {}
    for p in real_paths:
        by_base.setdefault(p.rsplit("/", 1)[-1], p)
    for f in findings:
        fp = f.get("file")
        if fp and fp not in real_paths:
            real = by_base.get(fp.rsplit("/", 1)[-1])
            if real:
                f["file"] = real
    before = len(findings)
    findings = dedup_findings(findings)
    if len(findings) < before:
        print(
            f"verification: deduped {before} -> {len(findings)} findings",
            file=sys.stderr,
        )
    for i, f in enumerate(findings):
        f["_id"] = i

    by_file = {}
    for f in findings:
        by_file.setdefault(f.get("file"), []).append(f)

    # Most-findings files first; file-less (process) findings pass through
    # unchanged. max_files=None means no cap (check every file with findings) --
    # used by the mid tier, which does the real culling and must not miss a file.
    files = [p for p in by_file if p]
    files.sort(key=lambda p: len(by_file[p]), reverse=True)
    if max_files and len(files) > max_files:
        print(
            f"verification: {len(files)} files with findings, checking the top "
            f"{max_files}; the rest are kept as unconfirmed",
            file=sys.stderr,
        )
        files = files[:max_files]
    to_check = set(files)

    verdicts = {}  # id -> verdict dict from the model
    header_cache = {}  # (path -> content|None) shared across files this run

    def source_block(path, code):
        # Full file plus the base-class/interface headers it inherits, so
        # override/hook/signature claims are judged against the real parent.
        context = gather_context(code, head_sha, header_cache)
        related = "".join(f"\n--- {p} ---\n{c}\n" for p, c in context.items())
        if context:
            print(
                f"verification: {path} + {len(context)} base/interface header(s)",
                file=sys.stderr,
            )
        return (
            f"\n\n=== FULL CURRENT SOURCE OF {path} (ground truth) ===\n"
            + number_lines(code)
            + (
                f"\n\n=== RELATED SOURCES for {path} (base classes / interfaces, "
                "ground truth) ===\n" + related
                if related
                else ""
            )
            + f"\n\n=== CHANGED LINES in {path} (in-scope lines) ===\n"
            + to_ranges(added_lines(diff, path))
            + f"\n\n=== DIFF HUNK FOR {path} ===\n"
            + file_hunk(diff, path)
        )

    def collect(raw):
        obj = extract_object(raw)
        for v in (obj or {}).get("verdicts", []) if isinstance(obj, dict) else []:
            if isinstance(v, dict) and isinstance(v.get("id"), int):
                verdicts[v["id"]] = v

    def payload_for(fs):
        return [
            {
                k: f.get(k)
                for k in ("_id", "file", "severity", "title", "detail", "line")
            }
            for f in fs
        ]

    sources = {}
    for path in to_check:
        code = fetch_file(path, head_sha)
        if code is None:
            print(
                f"verification: no source for {path}, keeping unconfirmed",
                file=sys.stderr,
            )
            continue
        sources[path] = code

    if single_call:
        # One request over all files at once; the final tier's model is the most
        # rate-limited, so a single batched call is cheapest on quota. Include as
        # many survivor files as fit the context budget (most-flagged first); any
        # that don't fit stay unconfirmed.
        blocks, included, total = [], [], 0
        for path in sorted(sources, key=lambda p: len(by_file[p]), reverse=True):
            block = source_block(path, sources[path])
            if included and total + len(block) > MAX_FINAL_SOURCE_CHARS:
                continue
            blocks.append(block)
            included.append(path)
            total += len(block)
        dropped = len(sources) - len(included)
        if dropped:
            print(
                f"final tier: {dropped} file(s) beyond the context budget kept "
                "unconfirmed",
                file=sys.stderr,
            )
        fs = [f for p in included for f in by_file[p]]
        if fs:
            user = (
                "You are given SEVERAL files. Judge each finding against the file "
                'named in its "file" field.\n\n=== FINDINGS (JSON) ===\n'
                + json.dumps({"findings": payload_for(fs)}, indent=2)
                + "".join(blocks)
            )
            try:
                collect(llm(prompts.VERIFICATION, user, model=model))
            except (urllib.error.URLError, KeyError, ValueError) as e:
                print(
                    f"final-tier verification failed, keeping unconfirmed: {e}",
                    file=sys.stderr,
                )
    else:
        for path, code in sources.items():
            user = (
                f"FILE: {path}\n\n=== FINDINGS (JSON) ===\n"
                + json.dumps({"findings": payload_for(by_file[path])}, indent=2)
                + source_block(path, code)
            )
            try:
                collect(llm(prompts.VERIFICATION, user, model=model))
            except (urllib.error.URLError, KeyError, ValueError) as e:
                print(
                    f"verification of {path} failed, keeping unconfirmed: {e}",
                    file=sys.stderr,
                )

    kept, drops, unconf, refuted_list = [], 0, 0, []
    for f in findings:
        v = verdicts.get(f["_id"]) or {}
        verdict = v.get("verdict")
        note = v.get("note") if isinstance(v.get("note"), str) else None
        if verdict == "refuted":
            drops += 1
            if len(refuted_list) < 15:
                refuted_list.append(
                    {
                        "title": f.get("title", ""),
                        "file": f.get("file"),
                        "note": note or "not supported by the code",
                    }
                )
            continue
        if verdict == "confirmed":
            # A stronger tier may confirm what an earlier tier left tentative;
            # shed the unconfirmed flag/cap so it can anchor and score normally.
            f.pop("unconfirmed", None)
            if isinstance(v.get("confidence"), (int, float)):
                f["confidence"] = v["confidence"]
            if isinstance(v.get("line"), int):
                f["line"] = v["line"]
        else:
            # unconfirmed / omitted / unchecked: keep tentative, cap confidence.
            f["unconfirmed"] = True
            unconf += 1
            base = (
                v.get("confidence")
                if isinstance(v.get("confidence"), (int, float))
                else f.get("confidence")
            )
            f["confidence"] = (
                min(base, UNCONFIRMED_CONF_CAP)
                if isinstance(base, (int, float))
                else UNCONFIRMED_CONF_CAP
            )
            if isinstance(v.get("line"), int):
                f["line"] = v["line"]
        if note:
            f["verify_note"] = note
        f.pop("_id", None)
        kept.append(f)

    for f in findings:
        f.pop("_id", None)
    print(
        f"verification [{vlabel}]: {len(kept)} kept ({unconf} unconfirmed), "
        f"{drops} refuted of {len(findings)}",
        file=sys.stderr,
    )
    stats = {
        "raw": before,
        "checked": len(findings),
        "kept": len(kept),
        "unconfirmed": unconf,
        "refuted": drops,
        "refuted_list": refuted_list,
    }
    return kept, stats


def synthesize(findings, pr_ctx="", model=None):
    """Merge/prioritize findings and produce a short overview.

    Returns (overview_text, findings). On any failure fall back to the raw
    union with an empty overview so the review still posts.
    """
    if not findings:
        return "", []
    try:
        raw = llm(
            prompts.SYNTHESIS,
            json.dumps({"findings": findings}, indent=2) + pr_ctx,
            model=model,
        )
    except (urllib.error.URLError, KeyError, ValueError) as e:
        print(f"synthesis failed, using raw union: {e}", file=sys.stderr)
        return "", findings
    obj = extract_object(raw)
    if obj is None:
        return "", findings  # synthesis misbehaved; keep the raw union
    merged = obj.get("findings")
    if not isinstance(merged, list) or not merged:
        merged = findings
    elif any(not str(m.get("title", "")).strip() for m in merged):
        # Some models return the findings list with title/detail dropped; that
        # would render as empty headers. Keep the verified findings intact and
        # take only the overview from synthesis.
        print(
            "synthesis returned findings with empty titles; keeping verified set",
            file=sys.stderr,
        )
        merged = findings
    overview = obj.get("summary")
    overview = overview.strip() if isinstance(overview, str) else ""
    return overview, merged


def claim_check(diff, pr_ctx, model=None):
    """Compare what the PR claims (pr_ctx) with what the diff does. Returns
    {claimed, done, difference} or None (no intent given, or the call failed)."""
    if not pr_ctx:
        return None
    user = f"{pr_ctx}\n\n=== PR DIFF (base..head) ===\n{diff}"
    try:
        raw = llm(prompts.CLAIM_CHECK, user, model=model)
    except (urllib.error.URLError, KeyError, ValueError) as e:
        print(f"claim check failed: {e}", file=sys.stderr)
        return None
    obj = extract_object(raw)
    return obj if isinstance(obj, dict) else None


def render_body(f):
    """Inline comment body. The tag carries the severity category and the
    model's own confidence number (a percentage), shown only when it exists."""
    parts = [f"severity: {f.get('severity', 'low')}"]
    cd = confidence_display(f)
    if cd:
        parts.append(f"confidence: {cd}")
    tag = " · ".join(parts)
    body = f"**{f.get('title', '')}**  \n`{tag}`\n\n{f.get('detail', '')}"
    if f.get("suggestion"):
        body += f"\n\n_Suggested fix:_ {f['suggestion']}"
    # ```suggestion renders as a one-click change; only offer it for a confirmed,
    # high-confidence finding so an applied guess cannot be wrong code. The prose
    # fix above stays either way.
    code = f.get("code_suggestion")
    conf = f.get("confidence")
    conf_ok = (isinstance(conf, (int, float)) and conf >= SUGGESTION_MIN_CONF) or (
        isinstance(conf, str) and conf.strip().lower() == "high"
    )
    if isinstance(code, str) and code.strip() and conf_ok and not f.get("unconfirmed"):
        body += "\n\n```suggestion\n" + code.rstrip("\n") + "\n```"
    if f.get("verify_note"):
        body += f"\n\n_Checked against the source: {f['verify_note']}_"
    body += f"\n\n<sub>stage: {f.get('stage', '?')}</sub>"
    return body


# Severity buckets for the summary body, in display order. Critical/high always
# land in the top bucket regardless of confidence; among the rest, a low-
# confidence finding is a tentative "worth a glance" rather than an action item.
SECTIONS = [
    ("critical", "\U0001f534 Critical & high"),
    ("suggestion", "\U0001f7e1 Suggestions"),
    ("optional", "\U0001f535 Optional / low-confidence"),
]


def bucket(f):
    # Unconfirmed findings stay tentative, never Critical.
    if f.get("unconfirmed"):
        return "optional"
    if f.get("severity") in ("critical", "high"):
        return "critical"
    if is_low_confidence(f):
        return "optional"
    return "suggestion"


def summary_line(f, anchored, compact=False):
    """One grouped-summary line: title, severity+confidence tag, and location.

    Anchored findings point at their inline comment for the detail. Compact mode
    (used for the collapsed low-confidence bucket) shows only that one line;
    otherwise an unanchored finding carries its detail and fix here.
    """
    sev = f.get("severity", "low")
    cd = confidence_display(f)
    tag = sev if not cd else f"{sev} · {cd} confidence"
    if f.get("unconfirmed"):
        tag += " · unconfirmed"
    where = f.get("file") or "general"
    if isinstance(f.get("line"), int):
        where = f"{where}:{f['line']}"
    line = f"- **{f.get('title', '')}** `[{tag}]` in `{where}`"
    if anchored:
        return line + " _(details inline)_"
    if compact:
        return line
    detail = " ".join((f.get("detail", "") or "").split())
    if detail:
        line += f"  \n  {detail}"
    if f.get("suggestion"):
        line += f"  \n  _Fix:_ {' '.join(f['suggestion'].split())}"
    return line


def methodology_block(stats, claim=None):
    """A collapsible note on how the review was produced, with a sample of the
    findings verification refuted (and why), so a reader can trust the culling."""
    if not stats or not stats.get("raw"):
        return []
    out = []
    if claim and (claim.get("claimed") or claim.get("done")):
        out += [
            "",
            "<details><summary>Claim vs. implementation</summary>",
            "",
            f"- Claimed: {claim.get('claimed', '(not stated)')}",
            f"- Done: {claim.get('done', '(unclear)')}",
            f"- Difference: {claim.get('difference', 'none')}",
            "</details>",
        ]
    out += [
        "",
        "<details><summary>How this review was produced</summary>",
        "",
        f"{len(prompts.STAGES)} specialized finder passes raised "
        f"{stats.get('raw', 0)} findings over the diff and the full changed "
        "sources. After de-duplication, {} were re-checked against the current "
        "file and the base-class / interface headers it inherits (code as truth), "
        "escalating survivors to a stronger model: {} refuted as unsupported, {} "
        "kept ({} tentative).".format(
            stats.get("checked", 0),
            stats.get("refuted", 0),
            stats.get("kept", 0),
            stats.get("unconfirmed", 0),
        ),
    ]
    refuted = stats.get("refuted_list") or []
    if refuted:
        out += ["", "Refuted by verification:"]
        for r in refuted:
            loc = f" ({r['file']})" if r.get("file") else ""
            out.append(f"- {r.get('title', '')}{loc}: {r.get('note', '')}")
    out.append("</details>")
    return out


def build_review(
    findings, head_sha, valid_files, overview="", stats=None, claim=None, diff=""
):
    """Assemble the GitHub review payload (summary body + inline comments).

    Body: a one-line TL;DR, a claim-vs-code note, a severity tally, findings
    grouped into buckets, and a collapsible methodology note. Findings that
    anchor to a diff line are also emitted as inline comments with the detail.
    """
    findings.sort(
        key=lambda f: (SEV_RANK.get(f.get("severity"), 9), confidence_sort_key(f))
    )

    # GitHub rejects the whole review (HTTP 422 "Line could not be resolved") if
    # any inline comment anchors to a line that is not on the diff's RIGHT side,
    # so only anchor to lines the diff actually adds. Cache per file.
    _added_cache = {}

    def _is_added_line(path, line):
        if path not in _added_cache:
            _added_cache[path] = added_lines(diff, path) if diff else None
        added = _added_cache[path]
        # No diff available (e.g. legacy callers): fall back to prior behavior.
        return True if added is None else line in added

    inline = []
    for f in findings:
        # Inline only on diff lines (API rejects otherwise); unconfirmed stay
        # summary-only; and stop once the inline cap is reached so a bad run
        # cannot flood the PR. Findings past the cap keep their detail in the
        # summary (they are not marked anchored).
        anchorable = (
            not f.get("unconfirmed")
            and f.get("file") in valid_files
            and isinstance(f.get("line"), int)
            and _is_added_line(f.get("file"), f.get("line"))
        )
        f["_anchored"] = anchorable and len(inline) < MAX_INLINE
        if f["_anchored"]:
            inline.append(
                {
                    "path": f["file"],
                    "line": f["line"],
                    "side": "RIGHT",
                    "body": render_body(f),
                }
            )

    lines = ["### DPsim LLM review", ""]
    if claim:
        diff_note = (claim.get("difference") or "").strip()
        if diff_note and diff_note.lower() != "none":
            lines += [f"**Claim vs. code:** {diff_note}", ""]
        else:
            lines += ["**Claim vs. code:** matches the description.", ""]
    if not findings:
        lines.append("No issues surfaced by the automated passes.")
    else:
        if overview:
            lines += [f"**TL;DR:** {overview}", ""]
        counts = {}
        for f in findings:
            sev = f.get("severity", "low")
            counts[sev] = counts.get(sev, 0) + 1
        tally = ", ".join(
            f"{n} {s}"
            for s, n in sorted(counts.items(), key=lambda x: SEV_RANK.get(x[0], 9))
        )
        lines.append(f"Found {tally} ({len(inline)} anchored to lines below).")
        for key, header in SECTIONS:
            items = [f for f in findings if bucket(f) == key]
            if not items:
                continue
            # Collapse the tentative bucket into a compact, foldable block so it
            # does not crowd the actionable findings.
            if key == "optional":
                lines += [
                    "",
                    f"<details><summary>{header} ({len(items)})</summary>",
                    "",
                ]
                lines += [summary_line(f, f["_anchored"], compact=True) for f in items]
                lines += ["", "</details>"]
            else:
                lines += ["", f"#### {header}"]
                lines += [summary_line(f, f["_anchored"]) for f in items]

    lines += methodology_block(stats, claim)
    finders = [os.environ.get("LLM_MODEL", "LLM")]
    if os.environ.get("LLM_MODEL_2"):
        finders.append(os.environ["LLM_MODEL_2"])
    stack = [
        f"find {', '.join(finders)}",
        f"verify {os.environ.get('LLM_VERIFY_MODEL') or finders[0]}",
    ]
    if os.environ.get("LLM_FINAL_MODEL"):
        stack.append(f"final {os.environ['LLM_FINAL_MODEL']}")
    lines += [
        "",
        "<sub>Automated, non-blocking review. May be wrong. Models: "
        + " → ".join(stack)
        + ".</sub>",
    ]

    payload = {
        "commit_id": head_sha,
        "event": "COMMENT",
        "body": "\n".join(lines),
        "comments": inline,
    }
    return payload


def gh_post_review(pr_number, payload):
    repo = env("GITHUB_REPOSITORY", required=True)
    token = env("GITHUB_TOKEN", required=True)
    summary = payload["body"]
    url = f"https://api.github.com/repos/{repo}/pulls/{pr_number}/reviews"

    def _post(url, body):
        req = urllib.request.Request(
            url,
            data=json.dumps(body).encode(),
            method="POST",
            headers={
                "Authorization": "Bearer " + token,
                "Accept": "application/vnd.github+json",
                "Content-Type": "application/json",
            },
        )
        with urllib.request.urlopen(req, timeout=60) as resp:
            return resp.status

    try:
        print(f"posted review, HTTP {_post(url, payload)}", file=sys.stderr)
        return
    except urllib.error.HTTPError as e:
        detail = e.read().decode(errors="replace")[:300]
        print(f"review POST failed HTTP {e.code}: {detail}", file=sys.stderr)

    # A single unresolvable inline line 422s the whole review. Retry without the
    # inline comments so the finding still lands as a proper review (visible in
    # the PR's review thread), not a detached issue comment.
    if payload.get("comments"):
        summary_only = {**payload, "comments": []}
        try:
            print(
                f"posted review without inline comments, HTTP "
                f"{_post(url, summary_only)}",
                file=sys.stderr,
            )
            return
        except urllib.error.HTTPError as e:
            print(
                f"summary-only review POST failed HTTP {e.code}: "
                f"{e.read().decode(errors='replace')[:300]}",
                file=sys.stderr,
            )

    # Last resort: a plain issue comment.
    icu = f"https://api.github.com/repos/{repo}/issues/{pr_number}/comments"
    print(
        f"posted fallback issue comment, HTTP {_post(icu, {'body': summary})}",
        file=sys.stderr,
    )


def merge_vstats(first, second):
    """Combine verification stats across two tiers for the methodology footer:
    keep the first tier's raw/checked counts, take the survivor count from the
    second, and sum what each tier refuted (with a bounded, combined sample)."""
    if not second:
        return first
    if not first:
        return second
    refuted_list = (first.get("refuted_list") or []) + (
        second.get("refuted_list") or []
    )
    return {
        "raw": first.get("raw", 0),
        "checked": first.get("checked", 0),
        "kept": second.get("kept", 0),
        "unconfirmed": second.get("unconfirmed", 0),
        "refuted": first.get("refuted", 0) + second.get("refuted", 0),
        "refuted_list": refuted_list[:15],
    }


def main():
    # DRY_RUN (or --dry-run) runs the full pipeline against the endpoint but
    # prints the assembled review instead of posting it, and needs no
    # GITHUB_TOKEN. Handy for local testing without any Actions runner.
    dry_run = os.environ.get("DRY_RUN") or "--dry-run" in sys.argv

    # Tiered models (all optional; unset -> everything runs on LLM_MODEL, i.e.
    # the original single-model behavior):
    #   LLM_MODEL         cheap/unlimited finder pass (over-generates candidates)
    #   LLM_MODEL_2       a second cheap/unlimited finder, for cross-model recall
    #   LLM_VERIFY_MODEL  mid-tier that grounds every candidate against the code
    #   LLM_FINAL_MODEL   strongest tier; re-checks only the survivors
    # The finders are noisy on purpose; precision comes from the verify tiers.
    finder2 = env("LLM_MODEL_2")
    verify_model = env("LLM_VERIFY_MODEL")
    final_model = env("LLM_FINAL_MODEL")

    pr_number, base_sha, head_sha = get_pr()
    diff = get_diff(pr_number, base_sha, head_sha)
    if not diff.strip():
        print("empty diff, nothing to review", file=sys.stderr)
        return
    valid_files = changed_files(diff)
    pr_ctx = pr_context(pr_number, base_sha, head_sha)
    claim = claim_check(diff, pr_ctx, model=verify_model)

    finder_models = [None] + ([finder2] if finder2 else [])
    raw_findings = run_stages(diff, head_sha, pr_ctx, models=finder_models)

    verified, vstats = verify(raw_findings, diff, head_sha, model=verify_model)
    if final_model:
        print(
            f"final tier ({final_model}): re-checking {len(verified)} survivor(s)",
            file=sys.stderr,
        )
        verified, vstats2 = verify(
            verified, diff, head_sha, model=final_model, single_call=True
        )
        vstats = merge_vstats(vstats, vstats2)

    overview, findings = synthesize(verified, pr_ctx, model=verify_model)
    payload = build_review(
        findings, head_sha, valid_files, overview, vstats, claim, diff=diff
    )

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
