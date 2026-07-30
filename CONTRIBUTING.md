<!--
SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
SPDX-License-Identifier: MPL-2.0
-->

# Contributing to DPsim

Thanks for your interest in contributing! Below are the essentials to get a pull request merged. For full detail see the [Contributing guide](https://dpsim.fein-aachen.org/docs/contributing/).

## Quick start

1. Fork the repository and clone your fork.
1. Run `pre-commit install` to activate automated checks (formatting, notebook output stripping).
1. Create a branch with a descriptive prefix (`feature/`, `fix/`, `docs/`).
1. Make your changes and commit using the conventional commit style (`feat:`, `fix:`, `docs:`, ...) with a sign-off:

   ```shell
   git commit -s -m "fix: correct node voltage initialization in DiakopticsSolver"
   ```

1. Keep your branch up to date by rebasing; do not merge the target branch into your branch:

   ```shell
   git fetch upstream
   git rebase upstream/master
   git push --force-with-lease
   ```

1. Open a pull request against `sogno-platform/dpsim:master`.

## Key requirements

- **Linear history**: never merge the target branch into your feature branch; rebase instead.
- **Sign-off**: every commit must include `Signed-off-by` via `git commit -s` ([DCO](https://developercertificate.org/)).
- **Formatting**: clang-format 17 for C++, black for Python, markdownlint for Markdown; all enforced by pre-commit.
- **SPDX headers**: new files must include a license header (see [the contributing guide](https://dpsim.fein-aachen.org/docs/contributing/)).
- **No `std::cout` in C++**: use `SPDLOG_LOGGER_*` macros.
- **No saved notebook outputs**: strip outputs from `.ipynb` files before committing; pre-commit and CI both enforce this.
- **Contributions in forks only**: do not push feature branches to the main repository.

## AI-assisted contributions

AI assistants are welcome here, and some of us use them. On licensing, DPsim follows the Linux Foundation [policy guidance on generative AI tools](https://www.linuxfoundation.org/legal/generative-ai): the tool's terms must not restrict its output in ways that conflict with the [MPL-2.0](https://mozilla.org/MPL/2.0/), and where the output carries pre-existing third-party material you need permission to contribute it and must supply the corresponding notice and attribution. That guidance invites projects to add their own, and the rest is ours. None of it is peculiar to AI: code adapted from a forum answer or another project has always carried the same obligations.

- **You are the author**: open a pull request only for code you can explain. Read it, build it, run the tests. The sign-off is already a [DCO](https://developercertificate.org/) claim that you have the right to submit the work.
- **Say which AI tools touched which parts**: in the pull request description. No need for a long explanation, but "I used AI" is not enough. No AI co-author trailers, the sign-off already names the author.
- **We care about correct results**: a curve that looks right is not a validated result. Compare against a reference case before you claim it works, whether you wrote the code or an assistant did.
- **Keep changes reviewable in one sitting**: split them when possible, and answer what reviewers actually asked.
- **Write your text yourself**: pull request descriptions, comments and issues, explaining the process in your words. Only people can be accountable for what is posted under their name.
- **We judge engagement, never writing style**: unpolished English is fine and we will not guess which words a model wrote. We may close a thread when we cannot tell what is being claimed, when the author goes quiet, or when the author is only passing our questions to an assistant. Closing is not personal, come back when you can talk it through yourself.
- **Report bugs you have seen**: if an assistant hands you a list of suspected bugs, do not open an issue for each one. Report the ones you reproduced, one per issue, with the steps to reproduce.
- **Our own LLM reviewer is disclaimed**: it posts under a bot identity, nobody checks what it writes before it appears, it cannot block a merge, and humans decide. Take what is useful and ignore the rest.
- **Keep private material out of assistants**: no unpublished research, no confidential data, no code you do not own.

This applies to pull requests and issues opened after it was merged, not retroactively.

## License

By contributing you agree your work is released under the [Mozilla Public License 2.0](https://mozilla.org/MPL/2.0/).
