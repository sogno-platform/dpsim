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
