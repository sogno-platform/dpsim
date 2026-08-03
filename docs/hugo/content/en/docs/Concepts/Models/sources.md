---
title: "Sources"
linkTitle: "Sources"
date: 2026-07-31
description: >
  Ideal and non-ideal sources, and what each costs the solver.
weight: 7
---

A source imposes a quantity on the network. Which quantity it imposes, and whether it does so
exactly, determines how it enters the system of equations and what it costs.

## Current sources are free, voltage sources are not

A current source imposes a known current into a node. Its contribution is entirely on the right hand
side of the nodal equations, and the system matrix does not know it exists.

A voltage source imposes a relation between two node voltages, which is not a nodal equation at all.
Nodal analysis has one equation per node expressing current balance, and there is no current
variable for an ideal voltage source to appear in. The system is extended with the source current as
an unknown and with the constraint that fixes the voltage difference, as described under
[nodal analysis]({{< ref "/docs/Concepts/nodal-analysis.md" >}}). The matrix grows by one row and
column per source, and the added diagonal entry is zero, so the extended matrix is no longer
positive definite and cannot be factorised by methods that assume it is.

This asymmetry is the reason so many models are formulated as current injections even when what they
physically represent is a voltage behind an impedance.

## The Norton equivalent

A voltage source with a series resistance can avoid the extension entirely. Source transformation
replaces a voltage $V$ behind a resistance $R$ with a current $V/R$ in parallel with the same
resistance,

```math
I_{eq} = \frac{V}{R}, \qquad G = \frac{1}{R},
```

which contributes a conductance to the matrix and a current to the right hand side. No extra
unknown, no zero on the diagonal, and the matrix stays the shape it would have had without the
source.

The two representations are equivalent at the terminals, exactly, for any $R$ that is not zero. The
choice is therefore numerical rather than physical, and the cost is that the source is no longer
ideal: its terminal voltage falls with the current drawn. Where a genuinely stiff source is wanted,
$R$ has to be made small, and a small $R$ means a large conductance, which is the same conditioning
trade-off that appears in [switches]({{< ref "switches.md" >}}).

## Sources that change over time

The simplest time-varying source takes its value from a generator, as described under
[signal processing blocks]({{< ref "signal-processing.md" >}}).

A ramp source is more specific: it holds one value, then moves to a second over a defined interval,
and holds that. The subtlety is what happens when the ramp changes not only the magnitude and phase
but also the frequency. Interpolating a frequency linearly and applying it as if it had always been
in force produces a phase discontinuity at both ends of the ramp, because phase is the integral of
frequency and not its product with time. Blending the frequency contribution in and out smoothly
over the ramp interval avoids that, at the price that the frequency during the transition is not the
linear interpolation it appears to be.

A profile source takes its value from a recorded sequence instead of from a formula, stepping
through samples as the simulation advances. It is the right choice when the excitation comes from a
measurement, and it carries the obvious constraint that the sample rate and the simulation step must
be reconciled: a profile is silent about what happens between its samples, and the simulation will
ask.

## Controlled sources

A controlled source takes its reference from another quantity in the simulation rather than from a
parameter or a clock. This is what allows a component to be built out of sources: a converter
imposes a voltage its control law computed, and an interface between two solvers imposes a value the
other side produced.

The distinction from a time-varying source is that the reference is not known in advance. Since the
reference is read as an input rather than solved simultaneously, it is the value from the previous
step, which introduces a delay of one step into whatever loop the source closes. For a control loop
that is usually acceptable and always worth knowing about; for a coupling between two solvers it is
the central property of the method, and it is the subject of
[branches]({{< ref "branches.md" >}}) where the same delay is used deliberately.
