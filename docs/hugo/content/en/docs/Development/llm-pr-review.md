---
title: "LLM Pull Request Review"
linkTitle: "LLM PR Review"
---

# Overview

DPsim ships an optional, non-blocking pull-request reviewer that runs a series of
specialised passes over the diff of a pull request using a large language model
and posts a single review comment. It is intended as an assistive first pass: it
never requests changes and cannot block a merge, so a human review remains
authoritative.

The reviewer lives under `.github/llm-review/` (the prompts in `prompts.py` and a
pure-standard-library runner in `review.py`) and is driven by two workflows:
`llm-review-collect.yml`, which runs on the pull request itself, and
`llm-review.yml`, which performs the review. The split is what makes reviewing
pull requests from forks safe (see [Fork pull requests](#fork-pull-requests)). It
communicates with any OpenAI-compatible chat endpoint, configured through the
environment variables described below.

# How it works

For each pull request the runner reads the `base..head` diff and sends it, in
turn, to a set of focused review stages, each with its own prompt. The stages
cover model equations and their derivation, MNA stamping and domain modeling,
numerical correctness, task scheduling and attribute usage, real-time safety,
C++ class design and reuse, naming and in-code documentation, logging discipline,
the Python bindings, input parsing, the build system and dependencies, testing
and component coverage, and licensing and pull-request hygiene. Each stage returns
a strict JSON list of findings. A final synthesis pass deduplicates and
prioritises them, and the runner posts them as one review, anchoring inline
comments only to lines present in the diff.

The prompts encode DPsim's documented conventions (see
[Guidelines]({{< ref "guidelines.md" >}})) and the recurring points raised in
past reviews, so the feedback stays specific to this project rather than generic.

# Configuration

The workflow requires one repository secret:

- `RWTH_LLM_TOKEN`: the bearer API key for the chat endpoint.

The following repository Actions variables are optional and override the
defaults baked into the workflow:

- `LLM_BASE_URL`: the OpenAI-compatible base URL.
- `LLM_MODEL`: the model identifier.
- `LLM_CHAT_PATH`: the chat path appended to the base URL (default
  `/chat/completions`).
- `LLM_REVIEW_RUNNER`: the runner label (default `ubuntu-latest`; see
  [Runner selection](#runner-selection)).

The workflow's baked-in defaults target an OpenAI-compatible deployment; override
`LLM_BASE_URL` and `LLM_MODEL` to point at a different endpoint or model.

# Obtaining an API key

Obtain a bearer API key from the chosen OpenAI-compatible provider and store it as
the `RWTH_LLM_TOKEN` repository secret. The key is only exposed to workflow runs on
pull requests from the repository itself, never from forks.

Before storing the secret, a single request confirms that the key reaches the
model:

```bash
curl -sS -X POST "$LLM_BASE_URL/chat/completions" \
  -H "Authorization: Bearer $RWTH_LLM_TOKEN" \
  -H "Content-Type: application/json" \
  -d "{\"model\":\"$LLM_MODEL\",\"messages\":[{\"role\":\"user\",\"content\":\"ok\"}]}"
```

# Running locally

The runner has a dry-run mode that executes the full pipeline and prints the
assembled review instead of posting it. It needs no GitHub token and no Actions
runner, only network access to the endpoint:

```bash
cd .github/llm-review
export LLM_BASE_URL='<openai-compatible-base-url>'
export LLM_MODEL='<model-identifier>'
export LLM_CHAT_PATH='/chat/completions'
export LLM_API_KEY="$RWTH_LLM_TOKEN"
export BASE_SHA=$(git rev-parse origin/main) HEAD_SHA=$(git rev-parse HEAD) PR_NUMBER=0
python3 review.py --dry-run
```

# Runner selection

The workflow defaults to a GitHub-hosted `ubuntu-latest` runner. If the chosen
endpoint is only reachable from within a particular network, set the
`LLM_REVIEW_RUNNER` variable to a self-hosted runner label registered inside that
network; no change to the workflow is required.

# Fork pull requests

Reviewing pull requests from forks requires care, because a fork's code is
untrusted and must never gain access to the secret. The reviewer uses the
`workflow_run` pattern for this, rather than `pull_request_target`, and splits the
work into two workflows:

- `llm-review-collect.yml` runs on the `pull_request` event, including from
  forks. GitHub withholds secrets from fork `pull_request` runs, so this job has
  no key. It checks out nothing and runs no code from the pull request; it only
  records the PR number and commit SHAs, taken from trusted GitHub context, into
  an artifact.
- `llm-review.yml` runs on `workflow_run`, after the collect job completes, in
  the base repository context where the secret is available. It checks out the
  base repository's own code, never the pull request's, and reads the diff as
  data through the GitHub API. It never builds or executes anything from the pull
  request.

Two properties keep the key safe. First, the key is only ever sent as an
`Authorization` header to the configured LLM endpoint, and is never placed in the
model prompt, so a prompt-injection payload in the diff cannot reveal it. Second,
the privileged job runs only trusted base-repository code, so untrusted pull
request code never executes with the secret in scope. The PR metadata read from
the collect artifact is validated (numeric PR number, hexadecimal SHAs) before
use. For this to operate, both the workflows and the secret must reside on the
repository the pull requests target.
