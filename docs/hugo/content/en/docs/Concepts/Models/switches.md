---
title: "Switches"
linkTitle: "Switches"
date: 2026-08-19
description: >
  Two-resistance switching, why a step change in resistance is a problem, and the current-zero and
  exponential ZCS-emulation modes that avoid it.
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

Closing has the mirror image of the same problem. Energising a path that terminates in capacitance
asks the network to move the capacitor voltage discontinuously, and the trapezoidal companion model
of the capacitor answers with the same alternating error.

This is a property of the discretisation, not of the physical circuit. A real breaker interrupts at a
current zero and closes at a voltage zero, and the arc across real contacts dissipates what is left;
a two-valued resistance switched at an arbitrary instant has neither mechanism.

## What a current zero means per domain

The remedy a real breaker uses is to switch where the discontinuity is not there in the first place.
Whether that is available depends on what the domain represents.

In EMT the state variables are instantaneous quantities, so the current zero is directly visible.
Waiting for it means the interrupted current is zero to within one time step and there is nothing
left for the companion model to disagree about. This is physically and numerically correct.

In DP the state variables are complex envelopes of a carrier at the reference frequency
$\omega_s$, and the instantaneous current can be reconstructed from them,

```math
i_k(t) = \mathrm{Re}\{ I_k(t) \, e^{j \omega_s t} \},
```

per phase for `DP::Ph3` and for the single envelope of `DP::Ph1`. This locates the current zero
correctly. What it does not do is remove the transient: the history terms of the neighbouring
inductors are expressed in the envelope, not in the instantaneous quantity, and they are not zero at
that instant. The envelope itself still has to jump, and the trapezoidal companion model cannot
follow it.

{{% alert title="CurrentZero in DP is not EMT-like behaviour" color="warning" %}}
The `CurrentZero` mode exists in DP and does find the correct instant, which is useful when the
switching time itself is the quantity of interest. It does not deliver the continuity that the same
mode gives in EMT, and the name invites that expectation. If you need continuous behaviour in DP,
use the exponential mode below.
{{% /alert %}}

## Exponential ZCS emulation

The alternative is to stop asking for a discontinuity at all. The switch resistance is moved
continuously between its two values along a geometric path,

```math
R(\alpha) = R_{closed} \left( \frac{R_{open}}{R_{closed}} \right)^{\alpha}
= \exp\!\big( \ln R_{closed} + \alpha \, ( \ln R_{open} - \ln R_{closed} ) \big),
```

with $\alpha$ advancing linearly in time over a switching duration $T_{sw}$ that is a model
parameter. Opening runs $\alpha$ from $0$ to $1$, closing runs it from $1$ to $0$; both directions
use the same law, so a switch configured this way is symmetric. Because the resistance stamped in a
step is the one the *next* solve uses, $\alpha$ is evaluated at $t + \Delta t$.

A geometric path rather than a linear one is what makes this work over nine decades of resistance: it
spends comparable numbers of steps in each decade, so the change per step stays a bounded ratio
instead of being negligible at first and violent at the end.

As the name suggests, this is an emulation of zero-current switching, not an arc model. There is no
physical arc voltage, no energy balance and no reignition. The parameter to choose is $T_{sw}$, and
it should be long enough that the ramp covers several time steps and short enough to stay
insignificant on the timescale being studied.

The cost is that the system matrix changes on every step of the transition rather than once, so each
of those steps requires a refactorisation. Both non-ideal modes therefore report
`supportsPrecomputedSystemMatrices() == false` and take the variable-component path through the
solver, while `Ideal` keeps the cheap precomputed two-state path.

On closing, DP ZCS suppresses unrealistic transients, also based on companion models history
terms. This also suppresses physically real closing inrush, so we still have only an
approximation. EMT closing inrush is real, not an artifact.

## Choosing a mode

| Mode | Opening | Closing | Use it when |
| --- | --- | --- | --- |
| `Ideal` | immediate | immediate | the switching transient does not matter, or the branch carries no inductive current and feeds no capacitance |
| `CurrentZero` | at the current zero | immediate | EMT, where it is the physically correct interruption; in DP only when the instant matters and the transient does not |
| `ExponentialZCSEmulation` | ramped over $T_{sw}$ | ramped over $T_{sw}$ | the transient has to be absent, in particular for DP in either direction |

The modes are available in `EMT::Ph3::Switch`, `DP::Ph3::Switch` and `DP::Ph1::Switch`. Every switch
defaults to `Ideal`

## The variable-resistance switch

`varResSwitch` (`DP::Ph1` and `SP::Ph1`) is the older, narrower form of the same idea. On opening it
multiplies the resistance by a fixed factor each step,

```math
R[k+1] = \alpha \, R[k], \qquad \alpha > 1,
```

until it passes the target open value, after which it is held there. Closing is immediate by construction. The
growth factor is derived from the step size, and the transition rewrites the configured
$R_{open}$ / $R_{closed}$ attributes as it runs, so the duration is whatever the factor happens to
produce rather than a value that is set.

`ExponentialZCSEmulation` supersedes it: same geometric path, but with an explicit switching
duration, without mutating the configured resistances, in both switching directions, and available
in three-phase and EMT variants.

## Series switching

Where a switch is combined with the series resistance it energises, the two are represented as one
element rather than as a switch plus a resistor. This keeps the branch to a single admittance and
avoids introducing an internal node that carries no physical meaning and adds an equation to the
system.
