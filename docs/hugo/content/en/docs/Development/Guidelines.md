---
title: "Guidelines"
linkTitle: "Guidelines"
---

This is a summary of general guidelines for the development of DPsim.

## Logging

Debug or trace should be the default log level for information that might be nice to have but not necessary for every simulation case.

Calls to the logger that might occur during simulation must use spdlog macros, like SPDLOG_LOGGER_INFO.

## Creating New Releases

Although DPsim currently does not have any conventions on versioning, the periodic creation of
new versions can help to mark significant changes and to analyze new portions of code using tools like SonarCloud.

A new version of DPsim has to be indicated as follows:
- Create a new tag with an increased version number
- Update setup.cfg
- Update CMakeLists.txt
- Update sonar-project.properties

Due to the creation of a new tag, a new PyPi package will be deployed automatically.
To release an updated Docker image, the container workflow needs to be triggered manually.