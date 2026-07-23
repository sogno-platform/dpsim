# Contributing to DPsim

Thanks for your interest in contributing! Below are the essentials to get a pull request merged. For full detail see the [Development Guidelines](https://dpsim.fein-aachen.org/docs/development/guidelines/).

## Quick start

1. Fork the repository and clone your fork.
1. Run `pre-commit install` to activate automated checks (formatting, linear history guard, notebook output stripping).
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

- **Linear history**: merge commits in a feature branch are blocked by CI; rebase instead.
- **Sign-off**: every commit must include `Signed-off-by` via `git commit -s` ([DCO](https://developercertificate.org/)).
- **Formatting**: clang-format 17 for C++, black for Python, markdownlint for Markdown; all enforced by pre-commit.
- **SPDX headers**: new files must include a license header (see [guidelines](https://dpsim.fein-aachen.org/docs/development/guidelines/)).
- **No `std::cout` in C++**: use `SPDLOG_LOGGER_*` macros.
- **No saved notebook outputs**: strip outputs from `.ipynb` files before committing; pre-commit and CI both enforce this.
- **Contributions in forks only**: do not push feature branches to the main repository.

## License

By contributing you agree your work is released under the [Mozilla Public License 2.0](https://mozilla.org/MPL/2.0/).
