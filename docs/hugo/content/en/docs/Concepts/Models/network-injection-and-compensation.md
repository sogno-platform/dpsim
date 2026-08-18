---
title: "Network Injection and Compensation"
linkTitle: "Injection and Compensation"
date: 2026-07-31
description: >
  Representing the rest of the grid, and the compensators that regulate voltage against it.
weight: 8
---

Every simulated network stops somewhere. What lies beyond the boundary has to be represented by
something, and once it is, the question of holding voltage at a bus becomes a question about what
that representation will support.

## The external network

The rest of the grid is represented as an ideal voltage source behind no impedance: a bus whose
voltage is imposed and whose current is whatever the network draws. This is the slack of the
powerflow carried into the time domain, and it supplies unlimited power at a fixed voltage and
frequency.

That idealisation is the right one when the external system is genuinely much stiffer than what is
being studied, and it is misleading when it is not. A stiff boundary suppresses exactly the
behaviour that a weak grid study is about: it holds the voltage the compensator is supposed to be
regulating and absorbs the power swings the converters are supposed to be sharing. Placing an
impedance between the source and the network is what makes the boundary finite, and the short
circuit ratio it produces is a modelling decision rather than a detail.

Because the imposed voltage comes from a signal generator rather than a constant, the boundary can
also be driven: a frequency ramp to study the response to rate of change of frequency, or a
modulated frequency to probe a control loop. The boundary then becomes the disturbance source rather
than the reference.

## Static reactive compensation

A static compensator regulates bus voltage by varying a shunt susceptance. It exchanges reactive
power only, so it can raise or lower voltage but supplies no energy.

The regulator measures the bus voltage through a first-order lag, forms the per-unit error against a
reference, and drives the susceptance through a further first-order lag with gain $K_r$ and time
constant $T_r$,

```math
T_r \dot{B} = K_r \, \frac{V_{meas} - V_{ref}}{V_{nom}} - B ,
```

both lags integrated with the trapezoidal rule so that the controller and the network advance
consistently.

Two properties follow from the physics rather than from the controller. The susceptance is bounded
at both ends by the installed capacitive and inductive ratings, and the regulator saturates against
those bounds rather than failing; a compensator sitting on its limit is providing everything it has
and the voltage error persists. And because the device is a susceptance rather than a source, the
reactive power it delivers falls with the square of the voltage. It is weakest exactly when the
voltage is lowest, which is when it is most needed.

The measurement lag matters more than it appears. It sits inside the regulator loop, so it is not
merely a smoothing of the reported value; making it small to track faster couples the compensator to
noise, and making it large delays the response into a range where it can interact with nearby
machine controls.

## Compensation in the power flow

Before any of that runs, the compensator has to appear in the power flow that sets the operating
point. There it is not a differential equation but a boundary condition, and since the device
exchanges no active power, the only question is what the reactive half of that condition states.

Two statements are available. Fixing the voltage and letting the solution supply whatever reactive
power holds it is the familiar voltage-controlled bus. It says what the compensator is for, but it
says it without bound: the reactive power that comes out is whatever the network needs, and nothing
in the formulation knows the device is rated. Where the requirement exceeds the rating, the result is
a converged case that could not be built.

Fixing the reactive power instead keeps the constant-power equations the rest of the network already
uses, and moves the regulation outside the Newton iteration. An outer loop adjusts the injection
until the bus reaches its reference, clamped to the installed band, and the clamped case is the same
one the dynamic model reaches: the device sits at its limit and the voltage error persists. The
inner solve stays a problem the solver is already good at, and the rating enters where it can be
enforced.

One property of the physical device does not survive the substitution. A susceptance delivers
reactive power in proportion to $V^2$, while a specified injection does not vary with voltage at all.
The two agree at the regulated voltage and diverge away from it, so a compensator that reached its
reference hands over an operating point the time domain will recognise, and one pinned at a limit
does not.

## Discrete compensation

Where the compensation is switched rather than continuous, the control is a different kind. The
regulator compares the voltage error against a deadband, and only if the error exceeds it does it
move by one discrete step, in the direction that reduces the error, subject to end stops.

The deadband is not a refinement but the central element. Without it any measurement noise drives
continual switching, and the switching is mechanical and finite in life. With it, the steady-state
voltage is not the reference but anywhere within a band around it, which is the accepted cost.

## Power flow control

A device that connects two systems through a converter pair rather than through a magnetic circuit
does not transfer voltage; it transfers power. Both sides are then specified as power exchanges
rather than by a turns ratio: an active power to be moved from one side to the other, and a reactive
power at each terminal set independently.

The distinction from a conventional transformer is that the sides are decoupled. The reactive power
on one side is not a consequence of the other, the transfer does not depend on the angle across the
device, and the two systems need not share a frequency. What is not free is the active power, which
is common to both terminals up to losses; specifying it independently on each side would ask the
device to create or destroy energy.
