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
| `checks.yaml` | pre-commit, the notebook output rule, and the `pyproject.toml` extras resolution |
| `test-linux.yaml` | the notebook test suite with coverage, the notebook result comparison, and the compiled WSCC 9 bus examples |
| `run_villas_example.yaml` | one VILLASnode example per matrix entry |
| `build-nix.yaml` | the Nix build |
| `packaging-python.yaml` | source distribution, wheels, and the PyPI uploads |
| `documentation.yaml` | the reference documentation and both deployment targets |

The compilation jobs, the Windows jobs and the cppcheck job stay in `ci.yaml`
itself rather than moving into a reusable workflow. This is deliberate and it is
worth keeping that way: a required status check is identified by its job name,
and a job called through a reusable workflow reports as `caller job / inner job`
instead. Moving one of them would silently invalidate the branch protection
entry that requires it. The compilation jobs are a matrix whose `name` is
`${{ matrix.title }}`; because that name interpolates a matrix value, GitHub does
not append the usual `(matrix, values)` suffix, so each job reports under exactly
the name the branch protection rule expects.

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
ref name, and the run then builds against the SHA tag. This needs a token that
may write packages, so it happens on pushes to `master` and on pull requests from
a branch of this repository; a pull request from a fork is covered below.

Manual dispatch takes a `rebuild_images` input that forces every image to be
rebuilt.

## Images for pull requests from forks

A fork pull request gets a read-only token, so it cannot publish the images it
would need in order to be tested against its own changes.
`prepare-images-fork.yaml` closes that gap. It triggers on `pull_request_target`,
restricted to the image definition paths, and its first job carries the `fork-pr`
environment. That environment has required reviewers, and the approval is the
only thing standing in front of a privileged run of code that came from a fork,
so it should not be granted without reading the change.

The gate job resolves the head revision and nothing else; the image build runs
behind it. No repository secret is passed into that workflow, so the only
credential in reach is a `GITHUB_TOKEN` limited to reading contents and writing
packages.

The main `CI` run does not wait for that approval. It probes the registry for
the images, falls back to Docker Hub when they are not published yet, and logs a
notice. Approve the `Prepare Images (fork)` run and re-run `CI` to have the
pull request tested against its own images.

The probe reads the registry anonymously, so the packages under
`ghcr.io/<owner>/dpsim/` have to be public.

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
