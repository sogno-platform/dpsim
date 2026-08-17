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

### Which level to write at

The level is chosen by what the message is for, not by how important it feels while writing it.

| Level | What belongs at it |
| --- | --- |
| `off` | Nothing. Selected when running a case study where only the simulation results matter |
| `error` | All errors |
| `warn` | All warnings |
| `info` | Basic information logged once, before or after the simulation |
| `debug` | Information for debugging: extended static information such as initialisation values, matrix stamps, subcomponents |
| `trace` | Information produced in each simulation step |

The dividing line that matters is between `debug` and `trace`. Anything written once per simulation
can be `debug`; anything written once per step has to be `trace`, because the cost is multiplied by
the step count and a run of 100 000 steps turns a single stray line into 100 000.

### Debug and trace are compiled out

`SPDLOG_ACTIVE_LEVEL` is fixed at `SPDLOG_LEVEL_INFO` in `dpsim-models/include/dpsim-models/Logger.h`,
so `SPDLOG_LOGGER_DEBUG` and `SPDLOG_LOGGER_TRACE` calls are removed by the preprocessor and produce
no output whatever level the logger is set to at runtime. Seeing them requires changing that line and
rebuilding, which is deliberate: it keeps per-step logging out of the release binary rather than
merely disabled. Raising the level on a component alone is therefore not enough, and that surprise is
tracked in [issue #300](https://github.com/sogno-platform/dpsim/issues/300).
