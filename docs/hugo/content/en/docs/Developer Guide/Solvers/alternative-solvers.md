---
title: "Alternative Solver Implementation"
linkTitle: "Alternative Solvers"
date: 2026-07-31
description: >
  The DAE, ODE and diakoptics solvers, and how the linear backend under MNA is chosen.
weight: 3
---

The methods are derived under
[alternative solution methods]({{< ref "/docs/Concepts/alternative-solvers.md" >}}). This page
covers the code and the configuration.

## `DAESolver`

Wraps the IDA integrator from Sundials. `initialize` builds the state and derivative vectors,
registers each component's residual function, then creates the solver with `IDACreate`, passes the
solver instance as user data so the residual callbacks can reach it, and sets scalar relative and
absolute tolerances with `IDASStolerances`.

Components take part by implementing `DAEInterface` and contributing their residual. The offset
vector recorded at the top of the file defines how each component's block is laid out within the
global residual.

Two practical notes. The solver chooses its own steps, so a run's cost is not predictable and it
cannot be used under a real-time timer. And several `std::cout` calls remain in the initialization
path, so it prints to standard output independently of the logger.

## `ODESolver` and `ODEintSolver`

`ODESolver` wraps CVODE from Sundials for a single component, sizing the problem from
`mOdePreState` and attaching a dense linear solver. `ODEintSolver` does the same job with boost's
odeint, calling `comp->odeint(y, ydot, t)`.

Both integrate one component across a network step while the network itself stays on its fixed step,
so the coupling is staggered and first-order accurate regardless of the inner integrator's order.

## `DiakopticsSolver`

Constructed with the system and an explicit list of components to tear, which must implement
`MNATearInterface`. `system.splitSubnets` performs the partition, `initSubnets` builds the
per-subnetwork node and component lists, and `mNodeSubnetMap` records which subnetwork owns each
node.

`createTearMatrices` is specialised per value type, and the sizes differ in a way worth noting: the
`Real` specialisation allocates `tearComponents * phaseMultiplier`, while the `Complex` one
allocates **twice** that, because a complex quantity is carried as a real-augmented pair. The phase
multiplier is 3 when the subnetwork phase type is `ABC` and 1 otherwise, taken from the first node
of the system.

The removed-branch system is dense and small. A comment in the source notes that the reduction could
still be sped up by exploiting the block diagonal structure of the inverse, so the present
implementation is correct rather than optimal.

## Linear backends under MNA

{{% alert title="Requires the matching build options" color="info" %}}
The nodal solver does not implement its factorisation. `MNASolverFactory` selects an adapter, and
`mSupportedSolverImpls` is compiled conditionally, so which of the implementations below exist
depends entirely on how DPsim was configured. The GPU adapters need a CUDA build.
{{% /alert %}}

| Implementation | Adapter | Notes |
| --- | --- | --- |
| `KLU` | `KLUAdapter` | Default, and the fallback when the choice is `Undef` |
| `SparseLU` | `SparseLUAdapter` | Eigen's sparse LU |
| `DenseLU` | `DenseLUAdapter` | Dense, for small systems |
| `CUDADense` | `GpuDenseAdapter` | Requires a CUDA build |
| `CUDASparse` | `GpuSparseAdapter` | Requires a CUDA build |
| `CUDAMagma` | `GpuMagmaAdapter` | Requires a CUDA build with Magma |
| `Plugin` | loaded at runtime | For a solver outside the tree |

`DirectLinearSolverConfiguration` tunes the chosen backend, and not every option applies to every
one:

- `SCALING_METHOD`: none, sum or max
- `FILL_IN_REDUCTION_METHOD`: `AMD`, `AMD_NV`, `AMD_RA` or `COLAMD`. The `NV` and `RA` variants take
  the set of time-varying entries into account when ordering, which is what makes partial
  refactorization effective for a network with switching elements.
- `PARTIAL_REFACTORIZATION_METHOD`: none, factorization path, or refactorization restart. This is the
  lever that matters when a switch or a variable component changes the matrix every step.
- `USE_BTF`: block triangular form on or off

The defaults are chosen for a general network. The combination of an ordering that knows about
varying entries with partial refactorization is what makes repeated switching affordable, and it is
inert if the matrix never changes.

## Source

Under `dpsim/src/`: `DAESolver.cpp`, `ODESolver.cpp`, `ODEintSolver.cpp`, `DiakopticsSolver.cpp`,
and the six `*Adapter.cpp` files.
