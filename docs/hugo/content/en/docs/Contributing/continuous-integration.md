---
title: "Continuous Integration"
linkTitle: "Continuous Integration"
description: >
  How the GitHub Actions workflows fit together and which of them are load bearing.
weight: 3
---

## Entry point

Almost everything runs from a single workflow, `.github/workflows/ci.yaml`. It
triggers on pushes to `master`, on tags, on pull requests and on manual dispatch,
and it calls the reusable workflows that hold the actual work:

| Reusable workflow | Contents |
| --- | --- |
| `prepare-images.yaml` | builds the container images from their Dockerfiles and pushes them to the GitHub container registry |
| `build-linux.yaml` | the five Linux compilations, Fedora and Rocky, with ccache |
| `build-windows.yaml` | the two Windows compilations |
| `checks.yaml` | pre-commit, the notebook output rule, the `pyproject.toml` extras resolution, and cppcheck |
| `test-linux.yaml` | the notebook test suite with coverage, the notebook result comparison, and the compiled WSCC 9 bus examples |
| `run_villas_example.yaml` | one VILLASnode example per matrix entry |
| `build-nix.yaml` | the Nix build |
| `packaging-python.yaml` | source distribution, wheels, and the PyPI uploads |
| `documentation.yaml` | the reference documentation and both deployment targets |

`ci.yaml` itself holds no build steps, only the triggers, the `setup` job and the
ordering.

One consequence is worth knowing before renaming anything. A required status
check is identified by its name, and a job called through a reusable workflow
reports as `caller job / inner job`, so the branch protection rule has to list
`Build Linux / Rocky Linux` rather than `Rocky Linux`. Matrix jobs whose `name`
interpolates a matrix value, as the build jobs do, are reported under exactly
that rendered name; a matrix job with a static name would instead get a
`(matrix, values)` suffix appended. Renaming a job, or moving one between
`ci.yaml` and a reusable workflow, therefore invalidates its protection entry and
has to be done together with an update to the branch protection settings.

## How a run picks its container images

The `setup` job decides this per image, not for all of them at once. Each image
has its own path filter, so editing `Dockerfile.dev-rocky` rebuilds the Rocky
image and leaves the other five alone. The two images that are built `FROM` the
development image, the release image and the Binder image, track the development
image's paths as well, so a change to `Dockerfile.dev` rebuilds all three.
Everything under `packaging/Shell/` counts towards every image.

An image whose definition did not change is used as published on Docker Hub. One
whose definition did change is rebuilt by `prepare-images.yaml` and pushed to
`ghcr.io/<owner>/dpsim/<image>` under two tags, the commit SHA and the slugified
ref name, and the run then builds against the SHA tag.

Manual dispatch takes a `rebuild_images` input that forces every image to be
rebuilt.

## Pull requests from forks

Rebuilding an image needs a token that may write packages, and a fork pull
request gets a read-only one. `CI` therefore listens on two events and lets
exactly one of them through: a pull request from a branch of this repository runs
on `pull_request`, and one from a fork runs on `pull_request_target`. The `setup`
job carries the condition, every other job depends on it, so the run that does
not apply skips in its entirety.

On the `pull_request_target` path `setup` also carries the `fork-pr` environment.
That environment has required reviewers, and because the whole pipeline hangs off
`setup`, the approval is the single gate in front of a privileged run of code that
came from a fork. It should not be granted without reading the change. Approving
means the fork's code builds and runs with a token that may write packages, and
with the repository secrets that the test and documentation jobs inherit.

Both paths build their images inside the same run that consumes them, so a pull
request is always tested against the images its own diff produces, and there is
nothing to re-run by hand.

Because `GITHUB_SHA` and `GITHUB_REF_NAME` point at the base branch under
`pull_request_target`, `setup` resolves the revision under test explicitly and
passes it to every job, each of which checks out that revision rather than the
default one.

The required check is the `all` barrier job, which fails if any job it depends on
failed or was cancelled. It renames itself on the path that does not apply, so
only the real run claims the protected name.

## Publishing to Docker Hub

`publish-images.yaml` is what refreshes the `sogno/dpsim` images that the
documentation and the outside world consume. It runs on the self-hosted runner,
needs the Docker Hub credentials, and triggers on pushes to `master`, where it
applies the same per-image path filters and republishes only what changed. Its
manual dispatch takes an `image` input for republishing a single image on
demand.

## Workflows outside CI

- `sonar_cloud.yaml` and `sonar_cloud_scan.yaml` are a deliberately separate
  pair. The scan holds `SONAR_TOKEN` and is triggered by the completion of the
  unprivileged build workflow; folding the build into `CI` would mean that any
  unrelated failing job suppresses the analysis.
- `llm-review.yml` and `llm-review-collect.yml` are described under
  [LLM Pull Request Review](../llm-pr-review/).
- `build_test_linux_rocky_profiling.yaml` is dispatch only.
- `documentation.yaml` is called by `CI` on `master`, and can also be dispatched
  on its own with an `image` input.
