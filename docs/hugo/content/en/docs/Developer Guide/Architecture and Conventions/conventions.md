---
title: "Coding Conventions"
linkTitle: "Conventions"
weight: 3
description: >
  Scaling of quantities and logging rules that code in DPsim has to follow.
---

Conventions that apply across the codebase. For the process of getting a change merged, see
[contributing]({{< ref "/docs/Contributing" >}}).

This is a summary of general guidelines for the development of DPsim.

## Scaling of Voltages and Currents

Voltage quantities are expressed either as phase-to-phase RMS values (denominated as `RMS3PH`) or as phase-to-ground peak values (denominated as `PEAK1PH`):

- Initialisation quantities (e.g. `initialSingleVoltage` of `SimPowerComp`) as `RMS3PH` values
- Simulation quantities in both `SP` and `DP` domain (e.g. `mIntfVoltage` of `DP::Ph1::PiLine`) as `RMS3PH values`
- Simulation quantities in the `EMT` domain (e.g. `mIntfVoltage` of `EMT::Ph3::Transformer`) as `PEAK1PH` values

Current quantities are expressed either as `RMS` or as `PEAK` values:

- Simulation quantities in both `SP` and `DP` domain (e.g. `mIntfCurrent` of `DP::Ph1::PiLine`) as `RMS` values
- Simulation quantities in the `EMT` domain (e.g. `mIntfCurrent` of `EMT::Ph3::Transformer`) as `PEAK` values

## Logging

Debug or trace should be the default log level for information that might be nice to have but not necessary for every simulation case.

Calls to the logger that might occur during simulation must use spdlog macros, like `SPDLOG_LOGGER_INFO`.
