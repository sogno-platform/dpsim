---
title: "Switch and Load Implementation"
linkTitle: "Switches and Loads"
date: 2026-07-31
description: >
  How the switch and load models are arranged in code, and the traps in configuring them.
weight: 21
---

The models are derived under [switches]({{< ref "/docs/Concepts/Models/switches.md" >}}) and
[loads]({{< ref "/docs/Concepts/Models/loads.md" >}}). This page covers only their arrangement in
code. Availability per domain is in
[model availability]({{< ref "/docs/Reference/model-availability.md" >}}).

## Switches

`Switch` implements `Base::Ph1::Switch` (or `Base::Ph3::Switch`) and stamps one admittance chosen by
`mIsClosed`, using `MNAStampUtils::stampAdmittance` so the grounded-terminal cases are handled
centrally. `SeriesSwitch` folds a series resistance into the same branch.

### Switching modes

`EMT::Ph3::Switch`, `DP::Ph3::Switch` and `DP::Ph1::Switch` carry a `SwitchingMode` enum — `Ideal`,
`CurrentZero`, `ExponentialZCSEmulation` — set through `setSwitchingMode()`, with
`setZeroCrossingTolerance()` and `setExponentialSwitchingTime()` for the two non-ideal modes. The
behaviour behind them is derived under [switches]({{< ref "/docs/Concepts/Models/switches.md" >}}).

The distinction that runs through the implementation is **commanded state versus physical state**.
`mIsClosed` is what the command asked for; `pole_closed_a/b/c` (`pole_closed` in `DP::Ph1`) is what
the poles are actually doing, and `mnaIsClosed()` reports the commanded state only in `Ideal` mode
and the pole state otherwise. `Base::Ph{1,3}::Switch::closeSwitch()` / `openSwitch()` are virtual for
exactly this reason: the override records the command and lets the post-step decide when the poles
follow. `Base::Ph1::Switch::close()` / `open()` are non-virtual delegates to them, so `SwitchEvent`
and the Python bindings need no change.

The non-ideal modes return `supportsPrecomputedSystemMatrices() == false` and drive the matrix
through `MNAVariableCompInterface::hasParameterChanged()` instead — per pole in `CurrentZero`, per
resistance value in `ExponentialZCSEmulation`, which means one refactorisation per step of the ramp.
`effective_resistance_*` is the value actually stamped and is the attribute to log when debugging a
transition; `exponential_progress`, `exponential_transition_active` and the start/end times are the
rest of the diagnostics.

{{% alert title="Watch out: the stamped resistance is one step ahead" color="warning" %}}
`updateExponentialTransition()` evaluates the ramp at `time + mTimeStep`, because the resistance it
writes is the one the *next* solve uses. `mTimeStep` is captured in `mnaCompInitialize`, so a switch
that never went through MNA initialisation would ramp against a zero step size.
{{% /alert %}}

In DP, `reconstructInstantaneousCurrent()` restores the carrier removed by the DP formulation. Note
the scaling asymmetry between the two DP variants: `DP::Ph3` envelopes are phase-peak and are built
from power flow with `RMS3PH_TO_PEAK1PH`, while `DP::Ph1` keeps the power-flow scaling, so the
reconstructed `i_instantaneous` of `DP::Ph1` is in envelope scaling. The zero-crossing instant does
not depend on it, which is why the detection is shared.

### varResSwitch

`varResSwitch` also implements `MNAVariableCompInterface`, which is what allows it to change
the system matrix during a run. Its `hasParameterChanged` is called each step and drives the
transition:

- Opening multiplies the resistance by `mDeltaResOpen` each step until it passes the target open
  value, then clamps to it and reports the transition finished.
- Closing uses `mDeltaResClosed`, which is `0`, so the first step takes the resistance to zero, the
  clamp catches it and sets the closed value. Closing is therefore immediate by construction, not by
  a separate code path.

{{% alert title="Watch out: setInitParameters is mandatory" color="warning" %}}
`setInitParameters(timestep)` must be called before the simulation, because the growth factor is
derived from the step size as `0.5 * timestep / 0.001 + 1`. It also captures the configured
resistances as the transition targets, since the live attributes are overwritten during the ramp. If
it is not called, `mDeltaResOpen` keeps its default of `1.5`, which is the value for a 1 ms step and
wrong for any other.
{{% /alert %}}

Its `initializeFromNodesAndTerminals` carries a comment saying it is not used.

New models should prefer `ExponentialZCSEmulation` on the plain `Switch`, which covers the same
ground with a parameterised duration and without rewriting its own resistance attributes.

## Loads

`RXLoad` is a `CompositePowerComp`. In `initializeFromNodesAndTerminals` it converts the powers to
element values and builds sub-components:

- a resistor, **only if** the active power is non-zero
- an inductor if the reactance is positive, a capacitor if negative, and **nothing** if the reactive
  power is zero

{{% alert title="Watch out: a zero power silently drops a branch" color="warning" %}}
Each is registered with `addMNASubComponent` and connected between ground and the load terminal. The
conditionals are the trap: a load configured with `P` or `Q` at zero silently omits that branch. It
does not error, and the missing branch is only visible as a load that draws less than expected.
{{% /alert %}}

`PQLoadCS` wraps a current source and sets its reference in `updateSetPoint` from
`conj(S / mNomVoltage)`. The nominal voltage, not the terminal voltage, is deliberate; the line
using the terminal voltage is present but commented out. Changing it would make the component
nonlinear and require an iterative solve.

`Shunt` takes a conductance and a susceptance directly and additionally carries per-unit attributes,
since it is the form the powerflow solver consumes.

## Source

- Switches: `{SP,DP,EMT}_Ph{1,3}_Switch`, `DP_Ph3_SeriesSwitch`, `EMT_Ph3_SeriesSwitch`, `{DP,SP}_Ph1_varResSwitch` under `dpsim-models/src/`
- Loads: `DP_Ph1_RXLoad`, `EMT_Ph3_RXLoad`, `DP_Ph1_RXLoadSwitch`, `DP_Ph1_PQLoadCS`, `SP_Ph1_Load`, `{SP,DP}_Ph1_Shunt`, `EMT_Ph3_Shunt`
