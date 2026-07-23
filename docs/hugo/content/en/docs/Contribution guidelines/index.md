---
title: "Contribution Guidelines"
linkTitle: "Contribution Guidelines"
weight: 10
date: 2023-05-03
description: >
  How to contribute to DPsim.
---

We'd love to accept your patches and contributions to this project.
Please send us a [pull request](https://github.com/sogno-platform/dpsim/pulls) or get in touch with us via mail or [Slack](https://lfenergy.slack.com/archives/C054GB551TL) if you would like to contribute.

For detailed requirements on code style, pull request format, and the linear history policy, see [Development Guidelines]({{< relref "/docs/Development/guidelines" >}}).

# Quick start

1. Fork the repository and clone your fork.
1. Run `pre-commit install` to activate the automated checks (formatting, linear history guard, etc.).
1. Create a feature branch and make your changes.
1. Keep your branch up to date by rebasing; do not merge the target branch into your branch:

    ```bash
    git rebase origin/main
    git push --force-with-lease
    ```

1. Open a pull request from your fork.
