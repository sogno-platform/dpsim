---
title: "EMT Ph3 Grid-Forming VSI Implementation"
linkTitle: "EMT Ph3 Grid-Forming VSI"
date: 2026-07-31
description: >
  How the grid-forming inverter is linearized, stamped and configured.
weight: 29
---

The equations and the linearization mathematics are derived under
[EMT Ph3 grid-forming voltage source inverter]({{< ref "/docs/Concepts/Models/Power Electronics/emt-ph3-grid-forming-vsi.md" >}}).
This page covers only their arrangement in code.

## Class and base

`EMT::Ph3::SSN_GFM` is `final` and derives from `EMT::Ph3::TwoTerminalVTypeVariableSSNComp`. All
seventeen states are real.

## State layout

| Index | Name | Kind |
| --- | --- | --- |
| 0, 1 | `PFiltered`, `QFiltered` | power filter |
| 2, 3 | `Omega`, `Theta` | droop frequency and angle |
| 4 | `VoltageMagnitude` | voltage droop output |
| 5, 6 | `VoltageIntegratorD`, `VoltageIntegratorQ` | outer voltage control |
| 7, 8 | `CurrentIntegratorD`, `CurrentIntegratorQ` | inner current control |
| 9, 10 | `DelayVoltageD`, `DelayVoltageQ` | modulation delay |
| 11&ndash;13 | `VcA`, `VcB`, `VcC` | filter capacitor voltage, per phase |
| 14&ndash;16 | `IfA`, `IfB`, `IfC` | filter inductor current, per phase |

`Omega` and `Theta` being states rather than inputs is what makes this grid forming: the converter
carries its own frequency and angle instead of tracking a measured one through a PLL.

## Numerical linearization

The Jacobians are not written out by hand. `calculateNumericalJacobians` forms all four by central
differences of the nonlinear state and output functions, so a change to the control equations needs
no matching change to any matrix code.

The perturbation for column $j$ is `absoluteStep + relativeStep * max(1, |x_j|)`, defaulting to
`1e-8` and `1e-6` and adjustable at runtime. The `max(1, ...)` floor means the step is effectively
absolute for small states and relative for large ones, which keeps the difference well conditioned
across states whose magnitudes differ by orders.

Because the model is time varying, the state-space form and its stamp are recomputed every step
rather than cached. That is the cost of this approach and the reason it is used only where the
control is genuinely nonlinear.

## Source and examples

- Source: [header](https://github.com/sogno-platform/dpsim/blob/master/dpsim-models/include/dpsim-models/EMT/EMT_Ph3_SSN_GFM.h), [implementation](https://github.com/sogno-platform/dpsim/blob/master/dpsim-models/src/EMT/EMT_Ph3_SSN_GFM.cpp)
- [C++ example, IEEE 9-bus mixed machine and inverter](https://github.com/sogno-platform/dpsim/blob/master/dpsim/examples/cxx/Circuits/EMT_Ph3_IEEE9_SSN_InverterMix.cpp)
- [Python notebook](https://github.com/sogno-platform/dpsim/blob/master/examples/Notebooks/Circuits/EMT_Ph3_IEEE9_SSN_InverterMix.ipynb)
- Grid-following counterpart: [EMT Ph3 averaged VSI implementation]({{< ref "emt-ph3-averaged-vsi-implementation.md" >}})
