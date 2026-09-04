---
title: "Power Flow Solvers"
linkTitle: "Power Flow Solvers"
weight: 3
description: >
  The Newton-Raphson implementations DPsim ships and how they are configured.
---

What DPsim implements. For the underlying formulation, the mismatch function and the Jacobian,
see [power flow]({{< ref "/docs/Concepts/powerflow.md" >}}).

## Solver Implementations

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

## Generator Reactive Power Limits

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
