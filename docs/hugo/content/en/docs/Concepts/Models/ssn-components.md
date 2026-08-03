---
title: "State-Space Nodal Components"
linkTitle: "SSN Components"
date: 2026-07-31
description: >
  How an individual component is written as a state-space model and turned into a nodal stamp.
weight: 13
---

The [state-space nodal]({{< ref "/docs/Concepts/state-space-nodal.md" >}}) method solves a component
simultaneously with the network instead of coupling it through a delayed injection. This page covers
the other half: how a single component is written so that the method applies to it, and what the
resulting stamp is.

## What a component supplies

Each component provides a continuous-time model in a state, an input and an output of its own
choosing,

```math
\dot{\boldsymbol{x}} = \boldsymbol{A}\boldsymbol{x} + \boldsymbol{B}\boldsymbol{u}, \qquad
\boldsymbol{y} = \boldsymbol{C}\boldsymbol{x} + \boldsymbol{D}\boldsymbol{u}.
```

The choice of what the vectors mean is the entire modelling step. For a component that behaves as an
admittance the input is the terminal voltage and the output is the terminal current; for one that
behaves as an impedance the roles are exchanged. Everything after this is mechanical.

## Discretisation

The state equation is integrated with the trapezoidal rule, which gives the discrete pair

```math
\boldsymbol{A}_d = \left( \boldsymbol{I} - \tfrac{\Delta t}{2}\boldsymbol{A} \right)^{-1}
                   \left( \boldsymbol{I} + \tfrac{\Delta t}{2}\boldsymbol{A} \right), \qquad
\boldsymbol{B}_d = \left( \boldsymbol{I} - \tfrac{\Delta t}{2}\boldsymbol{A} \right)^{-1}
                   \tfrac{\Delta t}{2} \boldsymbol{B}.
```

Substituting the discrete state into the output equation separates the output into a part that
depends on the present input and a part that does not,

```math
\boldsymbol{y} = \boldsymbol{W}\boldsymbol{u} + \boldsymbol{y}_{hist}, \qquad
\boldsymbol{W} = \boldsymbol{C}\boldsymbol{B}_d + \boldsymbol{D}, \qquad
\boldsymbol{y}_{hist} = \boldsymbol{C}\left( \boldsymbol{A}_d \boldsymbol{x} + \boldsymbol{B}_d \boldsymbol{u} \right).
```

This is exactly a companion model. $\boldsymbol{W}$ is an equivalent admittance that goes into the
system matrix and $\boldsymbol{y}_{hist}$ is an equivalent source that goes into the right hand
side. The difference from element-by-element companion models is only that $\boldsymbol{W}$ is
derived from the component's own state-space description rather than written by hand, so a component
with internal states and cross-coupling between phases needs no special treatment.

## The simplest case reproduces the classical result

Take a three-phase inductor. The natural choice is the current as state, the voltage as input and
the current as output, giving

```math
\boldsymbol{A} = \boldsymbol{0}, \quad
\boldsymbol{B} = \boldsymbol{L}^{-1}, \quad
\boldsymbol{C} = \boldsymbol{I}, \quad
\boldsymbol{D} = \boldsymbol{0}.
```

With $\boldsymbol{A} = \boldsymbol{0}$ the discretisation collapses to
$\boldsymbol{A}_d = \boldsymbol{I}$ and $\boldsymbol{B}_d = \tfrac{\Delta t}{2}\boldsymbol{L}^{-1}$,
so the equivalent admittance is $\tfrac{\Delta t}{2}\boldsymbol{L}^{-1}$ and the history term is the
previous current plus the previous voltage contribution. For a single phase that is
$\Delta t / 2L$, the familiar trapezoidal companion model of an inductor.

This is worth doing once because it shows the machinery adds no approximation of its own. A
component whose model is a plain inductor gets exactly the stamp it would have had.

## Where it earns its cost

The method is worth using when the component cannot be decomposed into independent elements. A
series RLC branch written as three separate companion models introduces two internal nodes; written
as one state-space model it introduces none, and the resulting stamp is a full matrix that captures
the coupling directly. The same applies to any component whose phases are coupled through a
non-diagonal inductance or through a control law.

The cost is that $\boldsymbol{W}$ is dense over the component's terminals, where element models
produce sparse contributions, and that a matrix inverse of the size of the state vector is required
whenever the model changes.

The derivation above is the instantaneous case. The same component model is discretised differently
in an envelope domain; see
[SSN across domains]({{< ref "/docs/Concepts/ssn-domain-formulation.md" >}}).

## Fixed and varying models

If $\boldsymbol{A}$, $\boldsymbol{B}$, $\boldsymbol{C}$ and $\boldsymbol{D}$ are constant, the
discrete matrices are computed once and the system matrix never changes on account of the component.

If the model depends on the operating point, it must be re-formed and re-discretised as the
operating point moves, and the system matrix refactorised with it. A saturating inductor whose
inductance is a piecewise linear function of flux is the simple case; a converter whose control law
is nonlinear is the general one.

## Initialization

The steady state at a given frequency follows from the continuous model directly,

```math
\boldsymbol{x} = \left( j\omega \boldsymbol{I} - \boldsymbol{A} \right)^{-1} \boldsymbol{B} \boldsymbol{u},
```

which is the state-space equivalent of evaluating a phasor impedance. A component initialized this
way starts in steady state rather than settling into it, provided the model is linear at the
operating point.
