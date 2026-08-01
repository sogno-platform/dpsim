---
title: "Alternative Solution Methods"
linkTitle: "Alternative Solvers"
date: 2026-07-31
description: >
  Differential-algebraic, ordinary differential and torn-network formulations, and when each is preferable to nodal analysis.
weight: 8
---

[Nodal analysis]({{< ref "nodal-analysis.md" >}}) is not the only way to advance a network in time.
Three other formulations exist, each answering a different objection to it.

## Differential-algebraic formulation

Nodal analysis discretises each dynamic element separately, replacing it with a companion model, and
then solves an algebraic system. The step size is chosen in advance and applies to everything.

A differential-algebraic formulation instead writes the whole system as it stands,

```math
\boldsymbol{F}\!\left(t, \boldsymbol{x}, \dot{\boldsymbol{x}}\right) = \boldsymbol{0},
```

mixing the differential equations of the dynamic elements with the algebraic constraints of the
network, and hands it to an integrator that chooses its own order and step size to meet a requested
error tolerance. Each component contributes its residual, the part of $\boldsymbol{F}$ it is
responsible for, rather than a companion model.

What this buys is error control. The user asks for an accuracy rather than a step size, and the
integrator takes small steps through fast transients and long ones through quiet intervals. What it
costs is determinism: the number of steps and the work per step are not known in advance, which
rules the method out for real-time execution and makes run times hard to predict. It is a method for
producing reference results, not for meeting a deadline.

The formulation also admits systems that nodal analysis handles awkwardly, because a constraint does
not have to be rewritten as an admittance to participate.

## Ordinary differential formulation for a component

A weaker version of the same idea applies to one component rather than the whole network. The
network is still solved by nodal analysis at a fixed step, but a single component whose internal
dynamics are stiff or awkward is integrated by a dedicated solver between network steps.

The component exposes its state derivative, the solver advances it over the network step, and the
result re-enters the network as an ordinary companion contribution. This keeps the deterministic
outer loop while allowing one component to be integrated more carefully than the rest. The
limitation is the same one as any staggered coupling: the component sees the network's terminal
conditions from the beginning of the step, so the exchange is only first-order accurate however
carefully the inner integration is done.

## Tearing a network

The third objection is about size rather than accuracy. Factorising the system matrix costs
super-linearly in the number of nodes, so one large network is more expensive than the sum of its
parts.

Diakoptics exploits this. A small set of branches is removed, chosen so that what remains falls into
independent subnetworks. Each subnetwork is factorised on its own, which is cheap, and the removed
branches are restored by solving a small dense system in the currents through them,

```math
\boldsymbol{v} = \boldsymbol{Y}^{-1}\boldsymbol{i}
                 - \boldsymbol{Y}^{-1}\boldsymbol{K}
                   \left( \boldsymbol{Z}_{tear} + \boldsymbol{K}^{\mathsf{T}}\boldsymbol{Y}^{-1}\boldsymbol{K} \right)^{-1}
                   \boldsymbol{K}^{\mathsf{T}}\boldsymbol{Y}^{-1}\boldsymbol{i},
```

where $\boldsymbol{K}$ records which nodes each removed branch connected and
$\boldsymbol{Z}_{tear}$ holds their impedances. The bracketed matrix has the dimension of the number
of torn branches, so the method pays off exactly when a network can be split by cutting few
branches.

The result is not an approximation. Unlike the delay-based decoupling described under
[branches]({{< ref "Models/branches.md" >}}), which introduces a travel time and therefore an error,
tearing reproduces the solution of the intact network to within round-off, because the removed
branches are restored within the same step. The trade is that the subnetworks cannot be advanced
independently: the small system couples them, so they must be solved together each step.

Choosing where to tear is the part with no general answer. Too few cuts leave the subnetworks large,
too many make the dense system dominate, and a cut through a strongly coupled part of the network
produces subnetworks whose solutions are dominated by the correction.
