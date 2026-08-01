---
title: "Injection and Compensation Implementation"
linkTitle: "Injection and Compensation"
date: 2026-07-31
description: >
  How the external network, the static compensator and the solid state transformer are built.
weight: 23
---

The models are derived under
[network injection and compensation]({{< ref "/docs/Concepts/Models/network-injection-and-compensation.md" >}}).
This page covers only the code.

## `NetworkInjection`

A `CompositePowerComp` wrapping a single `VoltageSource` sub-component. It owns no equations of its
own; it exists so that the external network is a named component rather than a bare source, and so
that the driving waveform can be swapped without changing the network description.

`setParameters` is overloaded by the kind of generator wanted behind it: a constant phasor for a
fixed source, a start frequency with a rate of change for a ramp, and an initial phasor with a
modulation frequency for a modulated one. Which overload is called determines which
`SignalGenerator` the sub-source is given; see
[signal component implementation]({{< ref "signal-components.md" >}}).

Because the source is ideal, adding an impedance to represent a finite short circuit level is the
caller's job. Nothing in the component does it.

## `SVC`

Not composite. It computes a susceptance each step and realises it by reconfiguring an internal
reactive element, so it implements the variable-component interface and forces a refactorisation
whenever the value changes.

`updateSusceptance` performs both lags with the trapezoidal rule, using precomputed constants
`Fac1 = dt / (2 Tr)`, `Fac2 = dt Kr / (2 Tr)` and `Fac3 = dt / (2 Tm)`. The measurement lag is
applied first, then the error is formed in per unit against `mNomVolt`, then the susceptance follows
from the previous value and the present and previous error.

The result is clamped to `mBMax` and `mBMin` before use, and the internal element is only rebuilt
when the value actually changed. The sign of the clamped susceptance selects which element is
formed: positive gives an inductance `1 / (omega * B * mBN)`, negative a capacitance
`B * mBN / (-omega)`. `mBN` is the base susceptance, so `B` is per unit.

{{% alert title="Watch out: mMechMode selects a different control law" color="warning" %}}
`mMechMode` switches the component to the discrete branch entirely. That path ignores the continuous
regulator and instead moves `mTapPos` by one step when the error exceeds `mDeadband`, bounded by
`mMinPos` and `mMaxPos`. The two modes share the component but not the control law, so a parameter
that matters in one is inert in the other.
{{% /alert %}}

{{% alert title="Suspected defect: magnitude taken from the real part only" color="danger" %}}
Note that the voltage magnitude is taken as `abs(real(V))` of the interface voltage rather than the
magnitude of the complex envelope. For a dynamic phasor quantity those differ, and the difference is
not negligible when the envelope has a significant imaginary part.
{{% /alert %}}

## `SolidStateTransformer`

A `CompositePowerComp` that represents each side as a current source rather than as a coupled
winding pair. `setParameters(nomV1, nomV2, Pref, Q1ref, Q2ref)` takes the two nominal voltages and
three power set points; the active power is common to both sides, while the reactive powers are set
per side.

Values are held in per unit internally, so the nominal voltages are the base rather than a turns
ratio. There is no magnetising branch, no leakage impedance and no angle dependence, which is the
representation the concept page describes and not an omission.

## Source

- [`DP_Ph1_NetworkInjection`](https://github.com/sogno-platform/dpsim/blob/master/dpsim-models/src/DP/DP_Ph1_NetworkInjection.cpp), and the SP and EMT variants alongside it
- [`DP_Ph1_SVC`](https://github.com/sogno-platform/dpsim/blob/master/dpsim-models/src/DP/DP_Ph1_SVC.cpp)
- [`SP_Ph1_SolidStateTransformer`](https://github.com/sogno-platform/dpsim/blob/master/dpsim-models/src/SP/SP_Ph1_SolidStateTransformer.cpp)

Availability per domain is in
[model availability]({{< ref "/docs/Reference/model-availability.md" >}}).
