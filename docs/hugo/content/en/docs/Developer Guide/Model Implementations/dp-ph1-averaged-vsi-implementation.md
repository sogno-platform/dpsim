---
title: "DP Ph1 Averaged VSI Implementation"
linkTitle: "DP Ph1 Averaged VSI"
date: 2026-07-31
description: >
  How the dynamic phasor averaged inverter is arranged in code and interfaced to the solver.
weight: 26
---

The equations are derived under
[DP Ph1 averaged voltage source inverter]({{< ref "/docs/Concepts/Models/Power Electronics/dp-ph1-averaged-vsi.md" >}}).
This page covers only their arrangement in code.

## Class and base

`DP::Ph1::AvVoltSourceInverterStateSpace` is `final` and derives from
`DP::Ph1::MixedVTypeVariableSSNComp`. The mixed base is what makes the model possible in this
domain: eight of the twelve states are real baseband control states and only the last four are the
real and imaginary parts of the two carrier-band envelopes, so the component cannot use the plain
complex SSN base.

## State layout

The state order is fixed by a private `StateIndex` enum, which the linearization indexes directly.

| Index | Name | Kind |
| --- | --- | --- |
| 0 | `Psi` | PLL angle deviation from the nominal carrier phase |
| 1 | `PhiPLL` | PLL integrator |
| 2, 3 | `PFiltered`, `QFiltered` | power filter |
| 4, 5 | `PhiD`, `PhiQ` | outer power control integrators |
| 6, 7 | `GammaD`, `GammaQ` | inner current control integrators |
| 8, 9 | `VcRe`, `VcIm` | filter capacitor voltage envelope |
| 10, 11 | `IfRe`, `IfIm` | filter inductor current envelope |

The base does not impose this ordering. It is told only how many real and how many complex states
there are, and sizes the packed real vector as `realStateCount + 2 * complexStateCount`. The
three-phase model orders its states the other way round, envelopes first and controls after, and is
equally valid. What the base does require is that the derived class hand it a state matrix that is
**already carrier shifted**: the steady-state solve assumes it, and a model that supplies an
unshifted matrix initializes to the wrong operating point rather than failing.

The default `initializeFromNodesAndTerminals` throws unless `realStateCount` is zero, so any model
with real control states, which includes this one, must override it.

Tracking `Psi` rather than the raw PLL angle keeps the tracked quantity bounded. The raw angle grows
without limit, which costs relinearization accuracy as a run gets longer.

## Parameters

{{% alert title="Watch out: fourteen positional parameters with no defaults" color="warning" %}}
`setParameters` takes the filter and control parameters positionally, in the order
`lf, cf, rf, rc, omegaN, kpPLL, kiPLL, omegaCutoff, pRef, qRef, kpPowerCtrl, kiPowerCtrl,
kpCurrCtrl, kiCurrCtrl`. There are fourteen of them and no defaults, so a transposed pair is easy to
introduce and produces a model that runs and is wrong rather than one that fails.
{{% /alert %}}

`initializeFromNodesAndTerminals` derives the initial state from the connected node voltage, so the
operating point comes from the powerflow rather than from user supplied states.

## Source and examples

- Source: [header](https://github.com/sogno-platform/dpsim/blob/master/dpsim-models/include/dpsim-models/DP/DP_Ph1_AvVoltSourceInverterStateSpace.h), [implementation](https://github.com/sogno-platform/dpsim/blob/master/dpsim-models/src/DP/DP_Ph1_AvVoltSourceInverterStateSpace.cpp)
- [C++ example](https://github.com/sogno-platform/dpsim/blob/master/dpsim/examples/cxx/Components/DP_Ph1_AvVoltSourceInverterStateSpace.cpp)
- [Python notebook](https://github.com/sogno-platform/dpsim/blob/master/examples/Notebooks/Components/DP_Ph1_AvVoltSourceInverterStateSpace.ipynb)
- The state-space nodal interfacing this model relies on is described under [state-space nodal]({{< ref "/docs/Concepts/state-space-nodal.md" >}})
