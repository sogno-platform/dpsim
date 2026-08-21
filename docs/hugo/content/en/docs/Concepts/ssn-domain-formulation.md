---
title: "State-Space Nodal Across Domains"
linkTitle: "SSN Across Domains"
date: 2026-07-31
description: >
  Why the same state-space component is discretised differently in an instantaneous and an envelope domain.
weight: 5
---

A component's state-space model does not change between simulation domains. The same
$\boldsymbol{A}$, $\boldsymbol{B}$, $\boldsymbol{C}$ and $\boldsymbol{D}$ describe the same physics
whichever domain solves them. What changes is the operator that is discretised, and that difference
propagates all the way to whether the resulting nodal stamp is real or complex.

The construction of the stamp itself is unchanged and is derived under
[state-space nodal components]({{< ref "/docs/Concepts/Models/ssn-components.md" >}}).

## The instantaneous case

Here the state equation is integrated as written,

```math
\dot{\boldsymbol{x}} = \boldsymbol{A}\boldsymbol{x} + \boldsymbol{B}\boldsymbol{u},
```

the trapezoidal rule gives a real discrete pair, and the equivalent admittance
$\boldsymbol{W} = \boldsymbol{C}\boldsymbol{B}_d + \boldsymbol{D}$ is real.

## The envelope case

In an envelope domain the state is a complex envelope $\tilde{\boldsymbol{x}}$ carrying an implicit
$e^{j\omega_s t}$. Differentiating that product contributes the carrier term derived under
[dynamic phasors]({{< ref "dyn-phasors.md" >}}), so the operator seen by the envelope is

```math
\frac{d}{dt} \; \longrightarrow \; \frac{d}{dt} + j\omega_s ,
```

and the system that must be discretised is not $\boldsymbol{A}$ but
$\boldsymbol{A} - j\omega_s \boldsymbol{I}$.

## The real-augmented form

Rather than integrate a complex system, split the envelope into real and imaginary parts. The
shifted operator becomes a real system of twice the size,

```math
\begin{bmatrix} \dot{\boldsymbol{x}}_{re} \\ \dot{\boldsymbol{x}}_{im} \end{bmatrix}
=
\begin{bmatrix} \boldsymbol{A} & \omega_s \boldsymbol{I} \\
                -\omega_s \boldsymbol{I} & \boldsymbol{A} \end{bmatrix}
\begin{bmatrix} \boldsymbol{x}_{re} \\ \boldsymbol{x}_{im} \end{bmatrix}
+
\begin{bmatrix} \boldsymbol{B} & \boldsymbol{0} \\ \boldsymbol{0} & \boldsymbol{B} \end{bmatrix}
\begin{bmatrix} \boldsymbol{u}_{re} \\ \boldsymbol{u}_{im} \end{bmatrix}.
```

The off-diagonal $\pm\omega_s \boldsymbol{I}$ blocks are the carrier rotation, and the block
structure $\begin{bmatrix} P & -Q \\ Q & P \end{bmatrix}$ is the real representation of the complex
number $P + jQ$. Discretising this real system with the same trapezoidal rule and recombining the
blocks recovers the complex discrete pair for a complex-linear component, from which the equivalent
admittance and the history term follow exactly as in the instantaneous case.

For a complex-linear envelope model, the resulting real block can be folded into one complex
admittance. A model that mixes real control states with complex-envelope states need not preserve
the $\begin{bmatrix} P & -Q \\ Q & P \end{bmatrix}$ structure. Its equivalent admittance must then
remain a full real block rather than being reduced to a complex scalar. Setting $\omega_s = 0$
still collapses the carrier-shifted state equations back to the instantaneous formulation; the
difference in the Norton representation comes from the component's axis coupling, not from a
different integration method.

## Why this matters for accuracy

The carrier rotation is handled analytically, inside $\boldsymbol{A}$, rather than numerically by
resolving the oscillation with small steps. For a component whose envelope varies slowly against a
fast carrier, the step size is then set by the envelope's own bandwidth rather than by the carrier
frequency. That is the entire accuracy argument for using an envelope domain here, and it fails for
exactly the reason it succeeds: content outside the retained band around $\omega_s$ has no
representation at all.
