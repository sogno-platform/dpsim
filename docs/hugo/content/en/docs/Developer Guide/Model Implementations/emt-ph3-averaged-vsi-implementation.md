---
title: "EMT Ph3 Averaged VSI Implementation"
linkTitle: "EMT Ph3 Averaged VSI"
date: 2026-07-31
description: >
  How the EMT averaged inverter is arranged in code and interfaced to the solver.
weight: 28
---

The equations are derived under
[EMT Ph3 averaged voltage source inverter]({{< ref "/docs/Concepts/Models/Power Electronics/emt-ph3-averaged-vsi.md" >}}).
This page covers only their arrangement in code.

## Class and base

`EMT::Ph3::AvVoltSourceInverterStateSpace` is `final` and derives from
`EMT::Ph3::TwoTerminalVTypeVariableSSNComp`. Unlike the dynamic phasor ports of this model, every
state here is real, so it uses the plain variable state-space nodal base rather than the mixed one.
The base sets `PhaseType::ABC` in its constructor, which this component relies on.

## State layout

Fourteen real states, controls first and filter states afterwards.

| Index | Name | Kind |
| --- | --- | --- |
| 0 | `ThetaPLL` | PLL angle |
| 1 | `PhiPLL` | PLL integrator |
| 2, 3 | `PFiltered`, `QFiltered` | power filter |
| 4, 5 | `PhiD`, `PhiQ` | outer power control integrators |
| 6, 7 | `GammaD`, `GammaQ` | inner current control integrators |
| 8&ndash;10 | `VcA`, `VcB`, `VcC` | filter capacitor voltage, per phase |
| 11&ndash;13 | `IfA`, `IfB`, `IfC` | filter inductor current, per phase |

The first state is the raw PLL angle. The dynamic phasor ports track the deviation from the nominal
carrier phase instead, because there the angle is compared against a carrier and an unboundedly
growing value costs relinearization accuracy. In EMT there is no carrier to drift against, so the
raw angle is used directly.

Six real filter states here correspond to two complex envelopes in the single-phase dynamic phasor
model and six in the three-phase one. That correspondence is the practical statement of what the
envelope transform buys.

## Source and examples

- Source: [header](https://github.com/sogno-platform/dpsim/blob/master/dpsim-models/include/dpsim-models/EMT/EMT_Ph3_AvVoltSourceInverterStateSpace.h), [implementation](https://github.com/sogno-platform/dpsim/blob/master/dpsim-models/src/EMT/EMT_Ph3_AvVoltSourceInverterStateSpace.cpp)
- [C++ example](https://github.com/sogno-platform/dpsim/blob/master/dpsim/examples/cxx/Components/EMT_Ph3_AvVoltSourceInverterStateSpace.cpp)
- [Python notebook](https://github.com/sogno-platform/dpsim/blob/master/examples/Notebooks/Components/EMT_Ph3_AvVoltSourceInverterStateSpace.ipynb)
- Dynamic phasor counterparts: [DP Ph1]({{< ref "dp-ph1-averaged-vsi-implementation.md" >}}), [DP Ph3]({{< ref "dp-ph3-averaged-vsi-implementation.md" >}})
