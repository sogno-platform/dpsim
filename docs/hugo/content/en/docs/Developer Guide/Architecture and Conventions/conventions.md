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

The conversion factors are defined in `dpsim-models/include/dpsim-models/Definitions.h`: `RMS3PH_TO_PEAK1PH` (`sqrt(2/3)`) for voltages and `RMS_TO_PEAK` (`sqrt(2)`) for currents, with inverses `PEAK1PH_TO_RMS3PH` and `PEAK_TO_RMS` for the return trip. `EMT::Ph3::VoltageSource::updateVoltage` and `EMT::Ph3::CurrentSource::updateCurrent` apply these at the reference-to-interface-quantity boundary; `DP::Ph3::VoltageSource`/`CurrentSource` and `SP::Ph1::VoltageSource` apply neither, since DP and SP already carry the complex phasor amplitude directly (SP has no `CurrentSource`). The same boundary applies wherever a component reads a power-flow-solved node voltage, for example the SSN `buildInitialInputFromNodes()`: the `EMT::Ph3` variant scales by `RMS3PH_TO_PEAK1PH`, the `DP::Ph1`/`DP::Ph3` variant does not.

This scaling is independent of a second, easily conflated convention: how a component computes power from its own native `V`, `I`. `DP::Ph1`/`SP::Ph1` treat their native `RMS3PH`/`RMS` values as already representing the full three-phase quantity, so total power is plain `P + jQ = V * conj(I)` (e.g. `DP::Ph1::AvVoltageSourceInverterDQ`, or `SP::Ph1::Load`'s `R = V_nom^2 / P`). `EMT::Ph3` works with per-phase peak quantities, so total three-phase power needs an explicit `1.5` factor (e.g. `EMT::Ph3::AvVoltSourceInverterStateSpace`). When porting a component between domains, do not copy an EMT power-formula constant into a DP/SP component or vice versa; each domain's native scale already determines whether that factor is needed.

## Logging

Debug or trace should be the default log level for information that might be nice to have but not necessary for every simulation case.

Calls to the logger that might occur during simulation must use spdlog macros, like `SPDLOG_LOGGER_INFO`.
