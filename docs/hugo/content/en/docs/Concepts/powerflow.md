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

Of these four, DPsim currently implements the **power mismatch function with polar
coordinates**, detailed below. It is the formulation used by both the dense
(`PFSolverPowerPolar`) and sparse (`PFSolverPowerPolarSparse`) solvers; the other three
are not implemented.

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

The formulas above use the voltage magnitude $\vert V_k \vert$ as the unknown. The DPsim
implementation instead uses the *relative* voltage increment $\Delta \vert V_k \vert / \vert V_k \vert$,
which scales every voltage-magnitude column ($J^{PV}$, $J^{QV}$) by $\vert V_k \vert$. For
example $J_{jj}^{PV}$ becomes $P_j(\vec{x}) + G_{jj} \vert V_j \vert^2$. This is paired with
the multiplicative voltage update $\vert V_k \vert \leftarrow \vert V_k \vert (1 + \Delta \vert V_k \vert / \vert V_k \vert)$,
so the solution is identical; only the scaling of the voltage columns differs.

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

## Convergence and Step Control

The iteration is governed by two parameters:

* **Tolerance** (default $10^{-8}$): the run is converged once every entry of the
  mismatch vector satisfies $\vert f_{i}(\vec{x}) \vert <$ tolerance (an infinity-norm
  test over all $\Delta P$ and $\Delta Q$ components).
* **Maximum iterations** (default $20$): an upper bound on the number of Newton steps
  per power flow solve.

To improve robustness far from the solution, the full Newton step is **scaled by a
single factor** $\alpha \in (0, 1]$ rather than damped component-wise. The factor is the
largest value that keeps the per-step changes within fixed bounds:

```math
\alpha = \min \left( 1,\ \frac{\Delta\theta_{max}}{\max_k \vert \Delta\theta_k \vert},\ \frac{\Delta V_{max}}{\max_k \vert \Delta V_k / V_k \vert} \right)
```

with $\Delta\theta_{max} = 0.2\ \text{rad}$ and $\Delta V_{max} = 0.1\ \text{pu}$. Because
the whole step is scaled by one factor, the Newton search direction is preserved, so
$\alpha = 1$ near the solution and quadratic convergence is retained; $\alpha < 1$ only
bounds large early steps. Voltage magnitudes are updated multiplicatively
($V_k \leftarrow V_k (1 + \alpha\, \Delta V_k / V_k)$), consistent with the relative
voltage increment used in the Jacobian.

# Solver Implementations

DPsim ships two implementations of the Newton-Raphson power flow solver with power
mismatch and polar coordinates. Both produce identical results (to round-off); they
differ only in how the Jacobian is stored and factorized:

* `PFSolverPowerPolar` (dense): assembles a dense Jacobian and computes a fresh
  factorization every Newton iteration. This is the default.
* `PFSolverPowerPolarSparse` (sparse): assembles the Jacobian into a sparse matrix
  whose sparsity pattern is fixed (derived once from the network admittance matrix).
  The symbolic factorization (ordering) is analyzed once and reused; only the numeric
  values are recomputed each Newton iteration. The first iteration of every power flow
  solve does a full factorization with pivoting, and subsequent iterations refactorize
  while reusing that ordering (via KLU when available). This scales better on large,
  sparse grids.

The dense solver is used by default. To opt in to the sparse solver:

```python
sim.set_pf_solver_use_sparse(True)
```

The flag is ignored and the dense solver is used if DPsim was built without a sparse
linear solver. The benchmark notebook
`examples/Notebooks/Grids/PF_Sparse_vs_Dense.ipynb` runs a range of network sizes both
ways, verifies the converged voltages match, and compares run time.

# Generator Reactive Power Limits

A PV bus assumes its generator can produce whatever reactive power the Newton-Raphson
solution asks for, holding $\vert V_k \vert$ at its setpoint. Real generators cannot: Q
is bounded by $Q_{min}$ and $Q_{max}$. DPsim can enforce these bounds with a
bidirectional PV↔PQ outer loop:

1. Run the inner Newton-Raphson solve to convergence (as described above).
1. For every PV bus, compute the generator's actual reactive output. If it exceeds
   $Q_{max}$ or falls below $Q_{min}$, pin the injection at the violated limit and
   convert the bus to PQ.
1. For every bus pinned this way in an earlier pass, check whether the constraint has
   relaxed: if $\vert V_k \vert$ has moved past its original setpoint in the releasing
   direction, restore voltage control and convert the bus back to PV.
1. Repeat from step 1 until no bus switches, an outer-iteration cap is hit, or a
   per-bus switch counter trips (an anti-oscillation guard, since a bus can otherwise
   toggle PV↔PQ indefinitely near the boundary).

Enforcement is opt-in and defaults off, so a system with no limits configured behaves
exactly as before:

```python
sim.set_pf_solver_enforce_q_limits(True)
```

$Q_{min}$/$Q_{max}$ are set per generator via `SynchronGenerator.set_parameters(...,
q_limit_max=..., q_limit_min=...)`; the defaults are $\pm\infty$ (unlimited).
Generators sharing a bus have their limits summed. The two limits are enforced
independently, with no assumption about sign or relative magnitude: asymmetric bounds
(e.g. $Q_{max}=150$ MVAr, $Q_{min}=-30$ MVAr) and same-sign bounds (e.g. a generator
restricted to $Q \in [20, 150]$ MVAr, always producing, or $Q \in [-150, -20]$ MVAr,
always absorbing) are both enforced correctly.

**Limitation: no P-dependent capability curve.** $Q_{min}$ and $Q_{max}$ are constants
set once per generator, not a function of active power output $P$. A real synchronous
generator's reactive capability is a "D-curve" bounded by three physically distinct
mechanisms: the stator (armature) current limit $\sqrt{P^2+Q^2} \le S_{rated}$, the
rotor (field) current / heating limit on the over-excited (Q-providing) side, and the
under-excitation limiter (UEL) / steady-state stability limit on the under-excited
(Q-absorbing) side. All three tighten as $P$ approaches rated output, and the over- and
under-excited bounds come from unrelated physical limits, so the true feasible region is
neither symmetric in $Q$ nor independent of $P$. DPsim does not model this curve; a
generator's $Q$ headroom is the same regardless of how much $P$ it is producing at the
time. Flat per-generator limits are a common baseline in power-flow tools generally, so
this is not a regression, but a P-dependent capability curve is not currently
implemented.

The notebook `examples/Notebooks/Grids/PF_Generator_Qlimits.ipynb` validates the
switching behavior on a small hand-wired case (binding and non-binding limits, dense vs.
sparse agreement).
