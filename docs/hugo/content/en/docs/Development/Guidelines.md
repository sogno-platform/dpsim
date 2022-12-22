---
title: "Guidelines"
linkTitle: "Guidelines"
---

This is a summary of general guidelines for the development of DPsim.

## Logging

Debug or trace should be the default log level for information that might be nice to have but not necessary for every simulation case.

Calls to the logger that might occur during simulation must use spdlog macros, like SPDLOG_LOGGER_INFO.
