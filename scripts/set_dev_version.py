#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
# SPDX-License-Identifier: MPL-2.0
# Bumps patch + appends .devN so Test PyPI never sees a reused, stale version.

import re
import sys


def main() -> None:
    path, run_number = sys.argv[1], sys.argv[2]

    with open(path) as f:
        content = f.read()

    match = re.search(r'(?m)^version = "(\d+)\.(\d+)\.(\d+)"', content)
    if not match:
        sys.exit(f'could not find a version = "X.Y.Z" line in {path}')

    major, minor, patch = match.groups()
    dev_version = f"{major}.{minor}.{int(patch) + 1}.dev{run_number}"

    content = (
        content[: match.start()] + f'version = "{dev_version}"' + content[match.end() :]
    )

    with open(path, "w") as f:
        f.write(content)

    print(f"set version to {dev_version}")


if __name__ == "__main__":
    main()
