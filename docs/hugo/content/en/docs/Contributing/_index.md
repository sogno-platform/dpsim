---
title: "Contributing"
linkTitle: "Contributing"
weight: 6
menu:
  main:
    weight: 26
description: >
  How to get a change into DPsim.
aliases: ["/docs/development/","/docs/contribution-guidelines/","/docs/roadmap/"]
---

Contributions of all kinds are welcome, including code, documentation, examples, models, bug
reports and reviews. Open a [pull request](https://github.com/sogno-platform/dpsim/pulls) or get
in touch through [GitHub Discussions](https://github.com/sogno-platform/dpsim/discussions).

These pages cover the process. For how the code is organised and the conventions it follows, see
the [developer guide]({{< ref "/docs/Developer Guide" >}}).

## Quick start

1. Fork the repository and clone your fork.
1. Run `pre-commit install` to activate the automated checks (formatting, notebook output stripping).
1. Create a branch with a descriptive prefix (`feature/`, `fix/`, `docs/`).
1. Commit with a sign-off, using the conventional commit style:

    ```bash
    git commit -s -m "fix: correct node voltage initialization in DiakopticsSolver"
    ```

1. Keep your branch up to date by rebasing; do not merge the target branch into your branch:

    ```bash
    git fetch upstream
    git rebase upstream/master
    git push --force-with-lease
    ```

1. Open a pull request from your fork against `sogno-platform/dpsim:master`.

## Pull Requests

There are no strict formal requirements besides the following:

1. **Developer Certificate of Origin (DCO)**

    We require a Developer Certificate of Origin. See more [here](https://github.com/lf-energy/tac/blob/main/process/contribution_guidelines.md#contribution-sign-off).

1. **Code Formatting with `pre-commit`**

    We enforce code formatting automatically using `pre-commit`. Please run `pre-commit install` the first time you clone the repository to run `pre-commit` before each commit automatically. If you forgot to do this, you will need to use the command `pre-commit run --all-files` one time to format your changes.

1. **Development in Forks Only**

    We accept contributions made in forks only. The main repository is not intended for contributor-specific branches.

1. **SPDX Headers**

    Use SPDX headers to indicate copyright and licensing information, especially when introducing new files to the codebase. For example:

    ```cpp
    /* Author: John Smith <John.Smith@example.com>
     * SPDX-FileCopyrightText: 2025 Example.com
     * SPDX-License-Identifier: MPL-2.0
     */
    ```

    Keep the SPDX tags on adjacent lines with no blank line between them, as the tooling
    reads them as a block.

1. **Linear History (no merge commits)**

    DPsim maintains a linear git history.
    Never merge the target branch into your feature branch; rebase instead:

    ```bash
    git rebase upstream/master
    git push --force-with-lease
    ```

1. **No Saved Notebook Outputs**

    Jupyter notebooks must be committed without saved cell outputs (images, plots, printed text).
    Saved outputs bloat diffs and can break the notebook test collector, which re-executes notebooks and re-extracts their outputs.
    Strip outputs before committing:

    ```bash
    jupyter nbconvert --ClearOutputPreprocessor.enabled=True --inplace <notebook.ipynb>
    ```

    A `pre-commit` hook does this automatically once you have run `pre-commit install`; CI also rejects a pull request that changes a notebook still carrying saved outputs.

## Creating New Releases (info for maintainers)

DPsim currently uses [Semantic Versioning](https://semver.org/). The periodic creation of
new versions can help to mark significant changes and to analyze new portions of code using tools like SonarCloud.

A new version of DPsim has to be indicated as follows:

- Update `version` in `pyproject.toml`
- Update `VERSION` in the top level `CMakeLists.txt`
- Update `sonar.projectVersion` in `sonar-project.properties`
- Update `CHANGELOG.md` and include all the unreleased changes in the list
- Create a new tag with an increased version number, which can be done during the release in GitHub

### Python Packages

Due to the creation of a new tag, a new PyPi package will be deployed automatically.

Only Linux packages are currently available, other platforms will be supported in the future.

### Container Images

To release an updated Docker image, the container workflow needs to be triggered manually.

If a Pull Request changes a container image, this is not updated automatically in the container image register.

## Planning

Short-term planning for new features is done on the GitHub [Project board](https://github.com/orgs/sogno-platform/projects/1).

You can also check the [Issues List](https://github.com/sogno-platform/dpsim/issues) or the [Pull Requests](https://github.com/sogno-platform/dpsim/pulls) on GitHub.

## Provenance and responsibility

Parts of this documentation were drafted with the help of large language models. The maintainers edit
and curate that output; they do not vouch for every line as if it were written from first-hand
knowledge, and the documentation carries no warranty.

That is a statement about how it is produced, not an excuse. The working rules exist precisely
because generated prose is confidently wrong in ways that read well:

- A page is written after running its code, never from reading the API. Every tutorial has a script
  in the repository and quotes it; if the two disagree, the page is wrong.
- A claim about the code is traced to where the value is used before it is written down. Two hazard
  notes here described correct code as broken because a line was read in isolation; both were
  corrected once the value was followed through.
- Anything measurable is measured by a script that reads the source, not asserted from having read a
  lot of it. That is what `generate_model_availability.py`, `check_docs_pairing.py` and
  `check_docs_hazards.py` are for.
- No reference is cited that the repository does not already contain. Invented citations look
  exactly like real ones.

A notice in the site footer says the same to readers. If you find something wrong,
[open an issue](https://github.com/sogno-platform/dpsim/issues/new). That is more useful than
assuming a page is authoritative because it is detailed.

## How the documentation is organised

Six sections, split by what the reader is trying to do rather than by topic. Putting a page in the
right one is most of the work; the rest is deciding which half of the subject it covers.

| Section | For someone who wants to | Names C++ classes |
| --- | --- | --- |
| User Guide | install DPsim and run a simulation | no |
| Tutorials | learn by working through one idea at a time | only Python API calls |
| Developer Guide | change DPsim, or understand how it works inside | yes |
| Concepts | know the mathematics behind a model or method | **never** |
| Reference | look up what exists and where | generated |
| Contributing | contribute to the project | n/a |

### Every subject is written twice

**Concepts** carries the mathematics: what the model represents, the equations, and what they
assume. It names no class, no file and no example, so a reader could follow it while implementing
the model in something else entirely.

**Developer Guide** carries the arrangement in code: the class hierarchy, how the component
interfaces with the solver, its attributes and state layout, the traps in configuring it, and links
to the source and examples.

The two link to each other. `scripts/docs/check_docs_pairing.py` reports any Concepts page without a
counterpart and exits non-zero, with an exemption list for method pages that have no single
implementation behind them.

### What is generated rather than written

Do not hand-edit these.

- The model availability matrix comes from the component headers via
  `scripts/docs/generate_model_availability.py`. A model class it does not recognise makes it fail rather
  than silently omit the model.
- The Python and C++ API references are generated by Sphinx and Doxygen from the source, so they are
  improved by writing better doc comments, not by editing the site.
- Tutorial figures come from `scripts/docs/generate_tutorial_figures.py`. The architecture diagrams do
  **not**: they are editable draw.io SVGs and must be edited in draw.io.

### Conventions

Every page carries a `description`, which appears under its title in section listings, and a
`weight`, which sets its order in the sidebar. A page without a weight falls to alphabetical, which
is how sections drift into arbitrary order.

Terminology follows standard electrical engineering usage. Avoid words that collide with class
names: "signal" reads as `SimSignalComp` rather than as a recorded quantity, so prefer "attribute".

## Highlighting what a reader must not miss

Documentation prose is read in order, so anything that will silently cost someone an afternoon has
to break out of the prose. Use the Docsy `alert` shortcode, with one of four titles, and keep the
vocabulary small so the colours stay meaningful.

```markdown
{{%/* alert title="Requires a build with VILLASnode" color="info" */%}}
Something the reader must have before this page works at all.
{{%/* /alert */%}}
```

| Colour | Use it for | Title starts with |
| --- | --- | --- |
| `info` | a prerequisite: a build flag, an optional dependency, a package that is not installed by default | `Requires` |
| `warning` | a trap: correct code that silently does the wrong thing, a parameter that means two things, an ordering that matters | `Watch out:` |
| `danger` | a known defect in DPsim itself, not something the reader can avoid by being careful | `Defect:` or `Suspected defect:` |
| `primary` | a genuine aside that is neither a prerequisite nor a hazard | `Note` |

Use exactly these four prefixes. Earlier pages mixed `Trap:`, `Watch out:` and `Gap:` for the same
colour, which made the severity unreadable: if two words mean one thing, neither means anything.

Three rules keep this useful. Mark a hazard where the reader meets it, not only in a reference page
they may never open, so the same trap can appear on a tutorial and on an implementation page. Reserve `danger` for defects: if a reader can avoid the problem by knowing about it, it is a
`warning`, and a page full of red stops being read. And a missing capability is a `Note`, not a
`Watch out:`, because there is no hazard to avoid, only something that cannot be done.

Do not use blockquotes for this. They carry no colour and no title, so they read as ordinary text.
