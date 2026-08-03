---
title: "Nodal Analysis"
linkTitle: "Nodal Analysis"
date: 2020-03-18
description: >
  How a network becomes a system of equations, and how components stamp into it.
weight: 1
---

A circuit with $b$ branches has $2b$ unknowns since there are $b$ voltages and $b$ currents.
Hence, $2b$ linear independent equations are required to solve the circuit.
If the circuit has $n$ nodes and $b$ branches, it has

* Kirchoff's current law (KCL) equations
* Kirchoff's voltage law (KVL) equations
* Characteristic equations (Ohm's Law)

There are only $n-1$ KCLs since the nth equation is a linear combination of the remaining $n-1$.
At the same time, it can be demonstrated that if we can imagine a very high number of closed paths in the network, only $b-n+1$ are able to provide independent KVLs.
Finally there are $b$ characteristic equations, describing the behavior of the branch, making a total of $2b$ linear independent equations.

The nodal analysis method reduces the number of equations that need to be solved simultaneously.
$n-1$ voltage variables are defined and solved, writing $n-1$ KCL based equations.
A circuit can be solved using Nodal Analysis as follows

* Select a reference node (mathematical ground) and number the remaining $n-1$ nodes, that are the independent voltage variables
* Represent every branch current $i$ as a function of node voltage variables $v$ with the general expression $i = g(v)$
* Write $n-1$ KCL based equations in terms of node voltage variable.

The resulting equations can be written in matrix form and have to be solved for $v$.

```math
\boldsymbol{Y} \boldsymbol{v} = \boldsymbol{i}
```

## Assembling the System by Stamping

The matrix $\boldsymbol{Y}$ and the right hand side $\boldsymbol{i}$ are never written out by
inspecting the whole circuit at once. Each element contributes its own fixed pattern of entries,
and the system is formed by adding those contributions together. A conductance $G$ between nodes
$k$ and $l$ adds $G$ to the diagonal entries $Y_{kk}$ and $Y_{ll}$ and subtracts it from the
off-diagonal entries $Y_{kl}$ and $Y_{lk}$; if one terminal is the reference node, only the
single diagonal entry appears. A current source injecting $I$ into node $k$ from node $l$ adds
$I$ to $i_k$ and subtracts it from $i_l$.

This additive assembly is what makes the method practical. An element needs to know only the
indices of the nodes it is attached to, never anything about the rest of the network, and the
same element contributes the same pattern regardless of what it is connected to. It also
explains the structure of the result: each row corresponds to one node and holds a non-zero
entry only for nodes reachable through a single element, so $\boldsymbol{Y}$ is symmetric for
networks of passive elements and sparse for any network of realistic size.

## Dynamic Elements and the Companion Model

The formulation above assumes every branch current can be written as a function of node voltages
alone. Inductors and capacitors do not satisfy this, since their currents depend on derivatives.
They are brought into the same form by discretising the differential relation over one time step.
Applying the trapezoidal rule to a capacitor between the instants $t - \Delta t$ and $t$ gives

```math
i(t) = \frac{2C}{\Delta t} v(t) - \left( \frac{2C}{\Delta t} v(t - \Delta t) + i(t - \Delta t) \right),
```

and the same treatment of an inductor gives

```math
i(t) = \frac{\Delta t}{2L} v(t) + \left( \frac{\Delta t}{2L} v(t - \Delta t) + i(t - \Delta t) \right).
```

Both have the form of a conductance in parallel with a current source. The conductance depends
only on the element value and the time step, so it is a constant contribution to
$\boldsymbol{Y}$; the current source depends only on quantities from the previous step, which are
known when the current step begins, so it is a contribution to $\boldsymbol{i}$. This pair is
called the companion model of the element, and the current source term is called its history
term. Once every dynamic element has been replaced by its companion model, the network at each
instant is a purely resistive one and the nodal formulation applies unchanged.

## Extending the Formulation for Voltage Sources

An ideal voltage source cannot be stamped as a conductance, because its current is not determined
by the voltage across it. The formulation is extended by admitting the source current as an
additional unknown and adding the equation that constrains its terminal voltage. The system
becomes

```math
\begin{bmatrix} \boldsymbol{Y} & \boldsymbol{A} \\ \boldsymbol{A}^{\mathsf{T}} & \boldsymbol{0} \end{bmatrix}
\begin{bmatrix} \boldsymbol{v} \\ \boldsymbol{j} \end{bmatrix}
=
\begin{bmatrix} \boldsymbol{i} \\ \boldsymbol{u} \end{bmatrix},
```

where $\boldsymbol{j}$ collects the unknown source currents, $\boldsymbol{u}$ the prescribed
source voltages, and $\boldsymbol{A}$ has a single $+1$ and $-1$ per source marking its terminals.
This extension is known as modified nodal analysis, and it is the form actually solved. The
augmented matrix is no longer positive definite and carries zeros on part of its diagonal, which
is why the solver has to be one that tolerates that rather than one specialised to the passive
case.

## Solving Over Time

Within a simulation the same system is solved once per time step. The left hand side depends only
on the element values, the topology and the time step, none of which change from one step to the
next in the ordinary case, so the matrix is factorised once and each step reuses that
factorisation with a new right hand side. The cost of a step is then a forward and backward
substitution rather than a full solve, which is what makes the method viable for large networks
and for real time.

Two situations invalidate the factorisation. A switching event changes the topology and therefore
the matrix, so the affected configuration has to be factorised again; simulations that switch
frequently often pre-compute a factorisation for each configuration instead. A non-linear element
makes the entries themselves depend on the solution, which requires iterating within the step
until the node voltages and the element operating points agree.
