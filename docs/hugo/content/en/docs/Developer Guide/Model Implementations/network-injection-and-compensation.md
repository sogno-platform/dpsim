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

## `SVC` in the time domain

`DP::Ph1::SVC` is the dynamic model. Not composite. It computes a susceptance each step and realises
it by reconfiguring an internal reactive element, so it implements the variable-component interface
and forces a refactorisation whenever the value changes.

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

## `SVC` in the power flow

`SP::Ph1::SVC` is a separate class with no code in common with the dynamic one. It is a
`SimPowerComp<Complex>` and a `PFSolverInterfaceBus` with one terminal and no MNA interface, so it
exists only for a power flow and takes no part in a time-domain solve.

It stamps nothing into the admittance matrix. `setParameters(ratedApparentPower, ratedVoltage,
setPointVoltage, qLimMax, qLimMin)` records the voltage set point and the reactive band and fixes
`mPowerflowBusType` to `PQ`; the active power is identically zero. What the solver sees is therefore
a reactive injection `Q_set` and a set point `V_set`, and `updateReactivePowerInjection` is the
setter that moves the injection. Attribute names follow `SynchronGenerator`: `V_set`, `V_set_pu`,
`Q_set`, `Q_set_pu`, `Q_max`, `Q_min` and their per-unit counterparts.

`calculatePerUnitParameters(baseApparentPower, baseOmega)` divides the set point by the base voltage
and the band by the base apparent power. Unset limits are $\pm\infty$ and stay $\pm\infty$ in per
unit. The method throws `std::invalid_argument` if either base is still zero, rather than dividing
and producing an infinite or undefined set point.

{{% alert title="Watch out: the base voltage comes from the solver, not from the caller" color="warning" %}}
`ratedVoltage` in `setParameters` is logged and otherwise unused. A compensator is not a voltage
source and must not seed a zone's voltage level, so `PFSolver::propagateAndVerifyBaseVoltage`
resolves the zone first and only then calls `setBaseVoltage` with the node's resolved value. A
standalone `SVC` that never went through a solver has no base voltage, which is what the guard in
`calculatePerUnitParameters` catches.
{{% /alert %}}

The regulation itself lives in the solver rather than in the component: something has to call
`updateReactivePowerInjection` between Newton solves for the set point to be held. Until that outer
loop exists, an `SVC` in a system is inert. `PFSolver::initialize` does not collect it into any of
its component lists, `determinePFBusType` does not read its bus type, and its `Q_set` stays at zero,
which is the same case as a node with nothing attached.

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
- [`SP_Ph1_SVC`](https://github.com/sogno-platform/dpsim/blob/master/dpsim-models/src/SP/SP_Ph1_SVC.cpp)
- [`SP_Ph1_SolidStateTransformer`](https://github.com/sogno-platform/dpsim/blob/master/dpsim-models/src/SP/SP_Ph1_SolidStateTransformer.cpp)

Availability per domain is in
[model availability]({{< ref "/docs/Reference/model-availability.md" >}}).
