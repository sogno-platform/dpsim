---
title: "Powerflow"
linkTitle: "Powerflow"
date: 2020-03-18
---

The power flow problem is about the calculation of voltage magnitudes and angles for one set of buses.
The solution is obtained from a given set of voltage magnitudes and power levels for a specific model of the network configuration.
The power flow solution exhibits the voltages and angles at all buses and real and reactive flows can be deduced from the same.

# Power System Model

Power systems are modeled as a network of buses (nodes) and branches (lines).
To a network bus, components such a generator, load, and transmission substation can be connected.
Each bus in the network is fully described by the following four electrical quantities:

* $\vert V_{k} \vert$: the voltage magnitude
* $\theta_{k}$: the voltage phase angle
* $P_{k}$: the active power
* $Q_{k}$: the reactive power

There are three types of networks buses: VD bus, PV bus and PQ bus.
Depending on the type of the bus, two of the four electrical quantities are specified as shown in the table below.

| Bus Type  | Known                             | Unknown                           |
| ---       |  ---                              |  ---                              |
| $VD$      | $\vert V_{k} \vert, \theta_{k}$   | $P_{k}, Q_{k}$                    |
| $PV$      | $P_{k}, \vert V_{k} \vert$        | $Q_{k}, \theta_{k}$               |
| $PQ$      | $P_{k}, Q_{k}$                    | $\vert V_{k} \vert, \theta_{k}$   |

# Single Phase Power Flow Problem

The power flow problem can be expressed by the goal to bring a mismatch function $\vec{f}$ to zero.
The value of the mismatch function depends on a solution vector $\vec{x}$:

```math
\vec{f}(\vec{x}) = 0
```

As $\vec{f}(\vec{x})$ will be nonlinear, the equation system will be solved with Newton-Raphson:

```math
-\textbf{J}(\vec{x}) \Delta \vec{x} = \vec{f} (\vec{x})
```

where $\Delta \vec{x}$ is the correction of the solution vector and $\textbf{J}(\vec{x})$ is the Jacobian matrix.
The solution vector $\vec{x}$ represents the voltage $\vec{V}$ by polar or cartesian quantities.
The mismatch function $\vec{f}$ will either represent the power mismatch $\Delta \vec{S}$ in terms of

```math
\left [ \begin{array}{c} \Delta \vec{P} \\ \Delta \vec{Q} \end{array} \right ]
```

or the current mismatch $\Delta \vec{I}$ in terms of

```math
\left [ \begin{array}{c} \Delta \vec{I_{real}} \\ \Delta \vec{I_{imag}} \end{array} \right ]
```

where the vectors split the complex quantities into real and imaginary parts.
Futhermore, the solution vector $\vec{x}$ will represent $\vec{V}$ either by polar coordinates

```math
\left [ \begin{array}{c} \vec{\delta} \\ \vert \vec{V} \vert \end{array} \right ]
```

or rectangular coordinates

```math
\left [ \begin{array}{c} \vec{V_{real}} \\ \vec{V_{imag}} \end{array} \right ]
```

This results in four different formulations of the powerflow problem:

* with power mismatch function and polar coordinates
* with power mismatch function and rectangular coordinates
* with current mismatch function and polar coordinates
* with current mismatch function and rectangular coordinates

To solve the problem using NR, we need to formulate $\textbf{J} (\vec{x})$ and $\vec{f} (\vec{x})$ for each powerflow problem formulation.

## Powerflow Problem with Power Mismatch Function and Polar Coordinates

### Formulation of Mismatch Function

The injected power at a node $k$ is given by:

```math
S_{k} = V_{k} I _{k}^{*}
```

The current injection into any bus $k$ may be expressed as:

```math
I_{k} = \sum_{j=1}^{N} Y_{kj} V_{j}
```

Substitution yields:

```math
\begin{align}
S_{k} &= V_{k} \left ( \sum_{j=1}^{N} Y_{kj} V_{j} \right )^{*} \nonumber \\
      &= V_{k} \sum_{j=1}^{N} Y_{kj}^{*} V_{j} ^{*} \nonumber
\end{align}
```

We may define $G_{kj}$ and $B_{kj}$ as the real and imaginary parts of the admittance matrix element $Y_{kj}$ respectively, so that $Y_{kj} = G_{kj} + jB_{kj}$.
Then we may rewrite the last equation:

```math
\begin{align}
S_{k} &= V_{k} \sum_{j=1}^{N} Y_{kj}^{*} V_{j}^{*} \nonumber \\
      &= \vert V_{k} \vert \angle \theta_{k} \sum_{j=1}^{N} (G_{kj} + jB_{kj})^{*} ( \vert V_{j} \vert \angle \theta_{j})^{*} \nonumber \\
      &= \vert V_{k} \vert \angle \theta_{k} \sum_{j=1}^{N} (G_{kj} - jB_{kj}) ( \vert V_{j} \vert \angle - \theta_{j}) \nonumber \\
      &= \sum_{j=1} ^{N} \vert V_{k} \vert \angle \theta_{k} ( \vert V_{j} \vert \angle - \theta_{j}) (G_{kj} - jB_{kj}) \nonumber \\
      &= \sum_{j=1} ^{N} \left ( \vert V_{k} \vert \vert V_{j} \vert \angle (\theta_{k} - \theta_{j}) \right ) (G_{kj} - jB_{kj}) \nonumber \\
      &= \sum_{j=1} ^{N} \vert V_{k} \vert \vert V_{j} \vert \left ( cos(\theta_{k} - \theta_{j}) + jsin(\theta_{k} - \theta_{j}) \right ) (G_{kj} - jB_{kj})
\end{align}
```

If we now perform the algebraic multiplication of the two terms inside the parentheses, and collect real and imaginary parts, and recall that $S_{k} = P_{k} + jQ_{k}$, we can express (1) as two equations: one for the real part, $P_{k}$, and one for the imaginary part, $Q_{k}$, according to:

```math
\begin{align}
{P}_{k} = \sum_{j=1}^{N} \vert V_{k} \vert \vert V_{j} \vert \left ( G_{kj}cos(\theta_{k} - \theta_{j}) + B_{kj} sin(\theta_{k} - \theta_{j}) \right ) \\
{Q}_{k} = \sum_{j=1}^{N} \vert V_{k} \vert \vert V_{j} \vert \left ( G_{kj}sin(\theta_{k} - \theta_{j}) - B_{kj} cos(\theta_{k} - \theta_{j}) \right )
\end{align}
```

These equations are called the power flow equations, and they form the fundamental building block from which we solve the power flow problem.

We consider a power system network having $N$ buses. We assume one VD bus, $N_{PV}-1$ PV buses and $N-N_{PV}$ PQ buses.
We assume that the VD bus is numbered bus $1$, the PV buses are numbered $2,...,N_{PV}$, and the PQ buses are numbered $N_{PV}+1,...,N$.
We define the vector of unknown as the composite vector of unknown angles $\vec{\theta}$ and voltage magnitudes $\vert \vec{V} \vert$:

```math
\begin{align}
\vec{x} = \left[ \begin{array}{c} \vec{\theta} \\ \vert \vec{V} \vert \\ \end{array} \right ]
        = \left[ \begin{array}{c} \theta_{2} \\ \theta_{3} \\ \vdots \\ \theta_{N} \\ \vert V_{N_{PV+1}} \vert \\ \vert V_{N_{PV+2}} \vert \\ \vdots \\ \vert V_{N} \vert \end{array} \right]
\end{align}
```

The right-hand sides of equations (2) and (3) depend on the elements of the unknown vector $\vec{x}$.
Expressing this dependency more explicitly, we rewrite these equations as:

```math
\begin{align}
P_{k} = P_{k} (\vec{x}) \Rightarrow P_{k}(\vec{x}) - P_{k} &= 0 \quad \quad k = 2,...,N \\
Q_{k} = Q_{k} (\vec{x}) \Rightarrow Q_{k} (\vec{x}) - Q_{k} &= 0 \quad \quad k = N_{PV}+1,...,N
\end{align}
```

We now define the mismatch vector $\vec{f} (\vec{x})$ as:

```math
\begin{align}
\vec{f} (\vec{x}) = \left [ \begin{array}{c} f_{1}(\vec{x}) \\ \vdots \\ f_{N-1}(\vec{x}) \\ ------ \\ f_{N}(\vec{x}) \\ \vdots \\ f_{2N-N_{PV} -1}(\vec{x}) \end{array} \right ]
    = \left [ \begin{array}{c} P_{2}(\vec{x}) - P_{2} \\ \vdots \\ P_{N}(\vec{x}) - P_{N} \\ --------- \\ Q_{N_{PV}+1}(\vec{x}) - Q_{N_{PV}+1} \\ \vdots \\ Q_{N}(\vec{x}) - Q_{N} \end{array} \right]
    = \left [ \begin{array}{c} \Delta P_{2} \\ \vdots \\ \Delta P_{N} \\ ------ \\ \Delta Q_{N_{PV}+1} \\ \vdots \\ \Delta Q_{N} \end{array} \right ]
    = \vec{0}
\end{align}
```

That is a system of nonlinear equations.
This nonlinearity comes from the fact that $P_{k}$ and $Q_{k}$ have terms containing products of some of the unknowns and also terms containing trigonometric functions of some the unknowns.

### Formulation of Jacobian

As discussed in the previous section, the power flow problem will be solved using the Newton-Raphson method. Here, the Jacobian matrix is obtained by taking all first-order partial derivates of the power mismatch functions with respect to the voltage angles $\theta_{k}$ and magnitudes $\vert V_{k} \vert$ as:

```math
\begin{align}
J_{jk}^{P \theta} &= \frac{\partial P_{j} (\vec{x} ) } {\partial \theta_{k}} = \vert V_{j} \vert \vert V_{k} \vert \left ( G_{jk} sin(\theta_{j} - \theta_{k}) - B_{jk} cos(\theta_{j} - \theta_{k} ) \right ) \\
J_{jj}^{P \theta} &= \frac{\partial P_{j}(\vec{x})}{\partial \theta_{j}} = -Q_{j} (\vec{x} ) - B_{jj} \vert V_{j} \vert ^{2} \\
J_{jk}^{Q \theta} &= \frac{\partial Q_{j}(\vec{x})}{\partial \theta_{k}} = - \vert V_{j} \vert \vert V_{k} \vert \left ( G_{jk} cos(\theta_{j} - \theta_{k}) + B_{jk} sin(\theta_{j} - \theta_{k}) \right ) \\
J_{jj}^{Q \theta} &= \frac{\partial Q_{j}(\vec{x})}{\partial \theta_{k}} = P_{j} (\vec{x} ) - G_{jj} \vert V_{j} \vert ^{2} \\
J_{jk}^{PV} &= \frac{\partial P_{j} (\vec{x} ) } {\partial \vert V_{k} \vert } = \vert V_{j} \vert \left ( G_{jk} cos(\theta_{j} - \theta_{k}) + B_{jk} sin(\theta_{j} - \theta_{k}) \right ) \\
J_{jj}^{PV} &= \frac{\partial P_{j}(\vec{x})}{\partial \vert V_{j} \vert } = \frac{P_{j} (\vec{x} )}{\vert V_{j} \vert} + G_{jj} \vert V_{j} \vert \\
J_{jk}^{QV} &= \frac{\partial Q_{j} (\vec{x} ) } {\partial \vert V_{k} \vert } = \vert V_{j} \vert \left ( G_{jk} sin(\theta_{j} - \theta_{k}) + B_{jk} cos(\theta_{j} - \theta_{k}) \right ) \\
J_{jj}^{QV} &= \frac{\partial Q_{j}(\vec{x})}{\partial \vert V_{j} \vert } = \frac{Q_{j} (\vec{x} )}{\vert V_{j} \vert} - B_{jj} \vert V_{j} \vert \\
\end{align}
```

The linear system of equations that is solved in every Newton iteration can be written in matrix form as follows:

```math
\begin{align}
-\left [ \begin{array}{cccccc}
    \frac{\partial \Delta P_{2} }{\partial \theta_{2}} & \cdots & \frac{\partial \Delta P_{2} }{\partial \theta_{N}} &
    \frac{\partial \Delta P_{2} }{\partial \vert V_{N_{G+1}} \vert} & \cdots & \frac{\partial \Delta P_{2} }{\partial \vert V_{N} \vert} \\
    \vdots & \ddots & \vdots & \vdots & \ddots & \vdots \\
    \frac{\partial \Delta P_{N} }{\partial \theta_{2}} & \cdots & \frac{\partial \Delta P_{N}}{\partial \theta_{N}} &
    \frac{\partial \Delta P_{N}}{\partial \vert V_{N_{G+1}} \vert } & \cdots & \frac{\partial \Delta P_{N}}{\partial \vert V_{N} \vert} \\
    \frac{\partial \Delta Q_{N_{G+1}} }{\partial \theta_{2}} & \cdots & \frac{\partial \Delta Q_{N_{G+1}} }{\partial \theta_{N}} &
    \frac{\partial \Delta Q_{N_{G+1}} }{\partial \vert V_{N_{G+1}} \vert } & \cdots & \frac{\partial \Delta Q_{N_{G+1}} }{\partial \vert V_{N} \vert} \\
    \vdots & \ddots & \vdots & \vdots & \ddots & \vdots \\
    \frac{\partial \Delta Q_{N}}{\partial \theta_{2}} & \cdots & \frac{\partial \Delta Q_{N}}{\partial \theta_{N}} &
    \frac{\partial \Delta Q_{N}}{\partial \vert V_{N_{G+1}} \vert } & \cdots & \frac{\partial \Delta Q_{N}}{\partial \vert V_{N} \vert}
\end{array} \right ]
\left [ \begin{array}{c} \Delta \theta_{2} \\ \vdots \\ \Delta \theta_{N} \\ \Delta \vert V_{N_{G+1}} \vert \\ \vdots \\ \Delta \vert V_{N} \vert \end{array} \right ]
= \left [ \begin{array}{c} \Delta P_{2} \\ \vdots \\ \Delta P_{N} \\ \Delta Q_{N_{G+1}} \\ \vdots \\ \Delta Q_{N} \end{array} \right ]
\end{align}
```

## Solution of the Problem

The solution update formula is given by:

```math
\begin{align}
  \vec{x}^{(i+1)} = \vec{x}^{(i)} + \Delta \vec{x}^{(i)} = \vec{x}^{(i)} - \textbf{J}^{-1} \vec{f} (\vec{x}^{(i)})
\end{align}
```

To sum up, the NR algorithm, for application to the power flow problem is:

1. Set the iteration counter to $i=1$. Use the initial solution $V_{i} = 1 \angle 0^{\circ}$
1. Compute the mismatch vector $\vec{f}({\vec{x}})$ using the power flow equations
1. Perform the following stopping criterion tests:
   * If $\vert \Delta P_{i} \vert < \epsilon_{P}$ for all type PQ and PV buses and
   * If $\vert \Delta Q_{i} \vert < \epsilon_{Q}$ for all type PQ
   * Then go to step 6
   * Otherwise, go to step 4.
1. Evaluate the Jacobian matrix $\textbf{J}^{(i)}$ and compute $\Delta \vec{x}^{(i)}$.
1. Compute the update solution vector $\vec{x}^{(i+1)}$. Return to step 3.
1. Stop.
