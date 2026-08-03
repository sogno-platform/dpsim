---
title: "Signal Processing Blocks"
linkTitle: "Signal Processing"
date: 2026-07-31
description: >
  Integrators, filters and the generators that drive time-varying sources.
weight: 14
---

Alongside the network components, a simulation contains blocks that carry no current and connect to
no node. They compute a value from another value, and they exist because controllers and sources are
built out of them.

## Integration

An integrator advances a state from its input using the trapezoidal rule,

```math
x[k] = x[k-1] + \frac{\Delta t}{2} \left( u[k] + u[k-1] \right),
```

which is the same rule the network solver applies to reactive elements, so a control loop and the
circuit it acts on are integrated consistently. Using a cruder rule for the controller would put an
error into the loop that no amount of tuning removes.

Not every block needs that accuracy. An angle accumulator advancing at a commanded frequency is
often stepped with the rectangular rule instead,

```math
\theta[k] = \theta[k-1] + \Delta t \, \omega[k],
```

which is a step behind but adds no dependence on the previous input. The distinction is worth
knowing when comparing an angle against one produced elsewhere, because the two rules differ by half
a step of phase.

## Finite impulse response filtering

A finite impulse response filter forms its output as a weighted sum of the most recent inputs,

```math
y[k] = \sum_{i=0}^{N-1} h_i \, u[k-i],
```

holding those inputs in a circular buffer of length $N$. Because the output depends only on past
inputs and never on past outputs, the filter cannot become unstable whatever the coefficients are,
and its phase response can be made exactly linear. The price is that a given sharpness needs a long
filter, which costs both memory and delay.

The delay is the part that matters in a control loop: a filter of length $N$ contributes roughly
$N/2$ steps of it. Inside a feedback path that delay is a phase lag, and it erodes stability margin
just as surely as raising a gain would.

## Signal generators

A source that varies over time takes its value from a generator. Four behaviours cover most uses: a
constant, a sinusoid at a fixed frequency and amplitude, a sinusoid whose frequency ramps between
two values, and one whose frequency is modulated continuously.

The frequency ramp is the one with a subtlety. A ramp is described by a start frequency, an end
frequency and a rate of change, and it is tempting to generate it by evaluating $\sin(\omega(t)\,t)$
with a time-varying $\omega$. That is wrong: the argument of the sine is the accumulated phase, not
the product of the present frequency and the elapsed time, and the two differ whenever the frequency
is not constant. The phase must be accumulated,

```math
\theta[k] = \theta[k-1] + 2\pi f[k] \, \Delta t ,
```

so that the instantaneous frequency is the derivative of the phase by construction. Generating a
ramp the naive way produces a signal whose actual frequency sweeps at twice the intended rate.

Accumulating phase makes the result depend on the step size and on the history of the run. Where an
exactly reproducible waveform is wanted, independent of when the simulation started or what steps it
took, the phase can instead be computed in closed form from the ramp parameters, which for a linear
ramp is a quadratic in time.
