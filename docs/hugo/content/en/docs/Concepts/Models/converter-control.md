---
title: "Converter Control"
linkTitle: "Converter Control"
date: 2026-07-31
description: >
  Phase tracking, angle generation and the cascaded control that drives a converter.
weight: 12
---

A converter model needs an angle to transform between the phase frame and its control frame, and a
control law that decides what to synthesise. The two questions are separable, and the answer to the
first is what distinguishes a grid-following converter from a grid-forming one.

## Tracking an angle: the phase-locked loop

A phase-locked loop drives the estimated angle until the measured voltage sits on the chosen axis of
the control frame. The error signal is the off-axis component, which is zero exactly when the frame
is aligned, and it is fed to a proportional-integral controller whose output is a frequency
correction.

With the integrator state $\phi$ and the frequency error input $e$, the loop is

```math
\dot{\phi} = k_i e, \qquad
\omega = \omega_{nom} + k_p e + \phi, \qquad
\dot{\theta} = \omega .
```

The nominal frequency enters as a feed-forward term rather than being learned, so the loop only has
to supply the deviation from it. That keeps the integrator near zero in normal operation and is why
a loop initialised at nominal frequency locks quickly.

The proportional gain sets how fast the loop follows a phase step and the integral gain how fast it
removes a standing frequency error. Making them large tracks disturbances the converter should
arguably ignore: a phase-locked loop that follows a fault as fast as it can is not obviously
desirable, since the converter then propagates the disturbance into its own control frame.

The important structural point is that a converter with a phase-locked loop takes its angle from the
network. It cannot operate without a voltage to lock to, which is what "grid following" means.

## Generating an angle: the oscillator

The alternative is to carry the angle as a state and advance it at a commanded frequency,

```math
\dot{\theta} = \omega_{ref},
```

with no measurement involved. The converter then imposes a phase rather than following one, which is
what "grid forming" means, and it continues to operate into a network with no other voltage source.

The difference between the two is one equation, but it determines whether the converter can start a
de-energised network or support frequency, and whether it has any defined behaviour when the grid
voltage collapses.

## Cascaded control

Above the angle sits a cascade, ordered from slowest to fastest.

The outer loop compares measured active and reactive power against their references. The measurement
is low-pass filtered first, because the instantaneous power computed from the terminal quantities
carries components at twice the fundamental under any unbalance, and feeding those into a controller
produces a modulation the converter should not emit. The filter cut-off therefore bounds how fast
this loop can be, independently of its gains.

The inner loop regulates the filter current to the reference the outer loop produced. It must be
substantially faster than the outer loop for the cascade to behave as intended: the outer loop is
designed assuming its commanded current is achieved essentially immediately, and that assumption
fails if the two bandwidths approach each other. The usual consequence is not instability but an
interaction that appears as a poorly damped oscillation at neither loop's design frequency.

Both loops are proportional-integral in the control frame, where a balanced fundamental quantity is
constant, so an integrator can drive the steady-state error to zero. This is the reason for working
in a rotating frame at all: the same controller applied to a sinusoid in the phase frame would leave
a standing error, because an integrator cannot track a moving target.

## Grid-forming voltage control

A grid-forming converter replaces the outer power loop with a voltage magnitude and frequency law.
Droop characteristics relate active power to frequency and reactive power to voltage, which lets
several converters share load without communicating: each responds to the same measured deviation,
and the split follows from the droop gains.

Below that, a voltage loop regulates the filter capacitor voltage and hands a current reference to
the same inner current loop as before. The inner loop is therefore common to both control
philosophies; only what sits above it changes.
