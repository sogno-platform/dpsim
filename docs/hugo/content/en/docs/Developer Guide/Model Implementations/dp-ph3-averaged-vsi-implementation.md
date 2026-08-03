---
title: "DP Ph3 Averaged VSI Implementation"
linkTitle: "DP Ph3 Averaged VSI"
date: 2026-07-31
description: >
  How the three-phase dynamic phasor averaged inverter is arranged in code.
weight: 27
---

The equations are derived under
[DP Ph3 averaged voltage source inverter]({{< ref "/docs/Concepts/Models/Power Electronics/dp-ph3-averaged-vsi.md" >}}).
This page covers only their arrangement in code.

## Class and base

`DP::Ph3::AvVoltSourceInverterStateSpace` is `final` and derives from
`DP::Ph1::MixedVTypeVariableSSNComp`, the same mixed base as the single-phase model. Per-phase
complex quantities are carried as `std::array<Complex, 3>`.

## State layout

Twenty states by default, ordered envelopes first and controls afterwards, or twenty-two with the
optional negative-sequence loop enabled.

| Index | Name | Kind |
| --- | --- | --- |
| 0&ndash;5 | `VcARe` &hellip; `VcCIm` | filter capacitor voltage envelope, per phase |
| 6&ndash;11 | `IfARe` &hellip; `IfCIm` | filter inductor current envelope, per phase |
| 12 | `Psi` | PLL angle deviation from the nominal carrier phase |
| 13 | `PhiPLL` | PLL integrator |
| 14, 15 | `PFiltered`, `QFiltered` | power filter |
| 16, 17 | `PhiD`, `PhiQ` | outer power control integrators |
| 18, 19 | `GammaD`, `GammaQ` | inner current control integrators |
| 20, 21 | `GammaND`, `GammaNQ` | negative-sequence current control integrators, only when enabled |

This is the reverse of the single-phase ordering, which places controls first. The base does not
care: it is given only the counts of real and complex states and sizes the packed real vector as
`realStateCount + 2 * complexStateCount`. What it does require is a state matrix that is already
carrier shifted, since the steady-state solve assumes it.

The last two states are the difference from the single-phase model beyond the per-phase filter.
Three independent phase envelopes admit a negative-sequence component that a single positive-sequence
envelope cannot represent, so the controller carries its own negative-sequence integrator pair.

## Enabling the negative-sequence loop

The constructor takes an `enableNegSeqControl` flag, `false` by default. The two references
$i_{nd,\mathrm{ref}}$ and $i_{nq,\mathrm{ref}}$ are the last two arguments of `setParameters` and
default to zero, which makes the loop a suppressor rather than an injector. The measured
$i_{rc,nd}$ and $i_{rc,nq}$ are exposed as the `irc_n_d` and `irc_n_q` attributes, and stay at zero
while the loop is disabled.

{{% alert title="Leave it off to compare against EMT::Ph3" color="info" %}}
The flag exists because the two configurations answer different questions. Off, the model has the
same 20 states and the same eigenvalue count as its `EMT::Ph3` counterpart, which is what a
cross-domain comparison needs. On, it gains 2 states and can regulate an unbalanced terminal.
{{% /alert %}}

The two integrators are appended after the control block rather than inserted next to the other
control states, so enabling the flag leaves every envelope and positive-sequence control index
unchanged. Code indexing into the state vector therefore does not need to know about the flag.

The theory behind the loop is derived under
[DP Ph3 averaged VSI]({{< ref "/docs/Concepts/Models/Power Electronics/dp-ph3-averaged-vsi.md" >}}).

## Source and examples

- Source: [header](https://github.com/sogno-platform/dpsim/blob/master/dpsim-models/include/dpsim-models/DP/DP_Ph3_AvVoltSourceInverterStateSpace.h), [implementation](https://github.com/sogno-platform/dpsim/blob/master/dpsim-models/src/DP/DP_Ph3_AvVoltSourceInverterStateSpace.cpp)
- [C++ example](https://github.com/sogno-platform/dpsim/blob/master/dpsim/examples/cxx/Components/DP_Ph3_AvVoltSourceInverterStateSpace.cpp)
- [Python notebook](https://github.com/sogno-platform/dpsim/blob/master/examples/Notebooks/Components/DP_Ph3_AvVoltSourceInverterStateSpace.ipynb)
- Single-phase counterpart: [DP Ph1 averaged VSI implementation]({{< ref "dp-ph1-averaged-vsi-implementation.md" >}})
