---
title: "Switches"
linkTitle: "Switches"
date: 2026-07-31
description: >
  Two-resistance switching and the variable-resistance switch used for faults.
weight: 5
---

A switch in a nodal formulation is not an ideal open or short. Both would be singular: an ideal
short shorts two node equations together, and an ideal open leaves a node with no path to ground.
Switches are therefore represented by a finite resistance that takes one of two values.

## The two-resistance model

The switch contributes a single admittance between its two terminals,

```math
G = \begin{cases}
1 / R_{closed} & \text{closed} \\
1 / R_{open} & \text{open}
\end{cases}
```

stamped as a conductance between the two terminal nodes, with the usual reduction when one terminal
is grounded. Typical values are far apart, of the order of milliohms closed and megohms open, so the
switch is a near short or a near open without ever being singular.

The consequence of this choice is that the ratio $R_{open} / R_{closed}$ lands directly in the
condition number of the system matrix. Making the contrast arbitrarily large to approach an ideal
switch degrades the accuracy of every node voltage in the network, not only those near the switch.
The values are a numerical compromise, not a physical measurement.

Because the admittance appears in the system matrix rather than in the right hand side, changing
state requires the matrix to be refactorised. This is why a network that switches often costs more
than one that does not, even though the model itself is trivial.

## Why a step change in resistance is a problem

Opening a switch that carries inductive current asks the network to interrupt that current within
one time step. The inductor opposes it, and with the trapezoidal companion model the result is a
numerical oscillation across the switch: the current alternates sign at the step frequency and
decays slowly, contaminating the solution for many steps after the event.

This is a property of the discretisation, not of the physical circuit. The physical arc that would
form across real contacts dissipates that energy; a two-valued resistance has no equivalent
mechanism.

## The variable-resistance switch

The variable-resistance switch removes the oscillation by refusing to make the change in a single
step. On opening, the resistance is multiplied by a fixed factor each step,

```math
R[k+1] = \alpha \, R[k], \qquad \alpha > 1,
```

until it reaches the target open value, after which it is held there. The current therefore decays
geometrically over several steps rather than being interrupted at once, which is close to what an
arc does and which the trapezoidal companion model can follow without ringing.

The growth factor is tied to the step size so that the transition covers a comparable interval of
time rather than a comparable number of steps. Closing is not ramped: the resistance is taken
straight to its closed value, because energising a path through a small resistance does not produce
the same interruption problem.

The cost is that the system matrix changes on every step of the transition rather than once, so each
of those steps requires a refactorisation. The switch is worth its cost where the interruption is
severe, typically a fault applied at a machine terminal or a transformer winding, and unnecessary
for ordinary load switching.

## Series switching

Where a switch is combined with the series resistance it energises, the two are represented as one
element rather than as a switch plus a resistor. This keeps the branch to a single admittance and
avoids introducing an internal node that carries no physical meaning and adds an equation to the
system.
