---
title: "Guidelines"
linkTitle: "Guidelines"
---

This is a summary of general guidelines for the development of DPsim.

# Scaling of Voltages and Currents

Voltage quantities are expressed either as phase-to-phase RMS values (denominated as `RMS3PH`) or as phase-to-ground peak values (denominated as `PEAK1PH`):

- Initialisation quantities (e.g. `initialSingleVoltage` of `SimPowerComp`) as `RMS3PH` values
- Simulation quantities in both `SP` and `DP` domain (e.g. `mIntfVoltage` of `DP::Ph1::PiLine`) as `RMS3PH values`
- Simulation quantities in the `EMT` domain (e.g. `mIntfVoltage` of `EMT::Ph3::Transformer`) as `PEAK1PH` values

Current quantities are expressed either as `RMS` or as `PEAK` values:

- Simulation quantities in both `SP` and `DP` domain (e.g. `mIntfCurrent` of `DP::Ph1::PiLine`) as `RMS` values
- Simulation quantities in the `EMT` domain (e.g. `mIntfCurrent` of `EMT::Ph3::Transformer`) as `PEAK` values

# Logging

Debug or trace should be the default log level for information that might be nice to have but not necessary for every simulation case.

Calls to the logger that might occur during simulation must use spdlog macros, like `SPDLOG_LOGGER_INFO`.

# Pull Requests

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

# Creating New Releases (info for maintainers)

DPsim currently uses to [Semantic Versioning](https://semver.org/). The periodic creation of
new versions can help to mark significant changes and to analyze new portions of code using tools like SonarCloud.

A new version of DPsim has to be indicated as follows:

- Update setup.cfg
- Update CMakeLists.txt
- Update sonar-project.properties
- Update CHANGELOG.md and include all the unreleaed changes in the list
- Create a new tag with an increased version number (can be done during the Release in GitHub)

## Python Packages

Due to the creation of a new tag, a new PyPi package will be deployed automatically.

Only Linux packages are currently available, other platforms will be supported in the future.

## Container Images

To release an updated Docker image, the container workflow needs to be triggered manually.

If a Pull Request changes a container image, this is not updated automatically in the container image register.
