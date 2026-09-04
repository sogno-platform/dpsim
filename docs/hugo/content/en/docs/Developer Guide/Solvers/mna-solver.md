---
title: "The MNA Solver"
linkTitle: "MNA Solver"
weight: 1
date: 2026-07-31
description: >
  The default solver: how it assembles, factorises and steps the system.
---

`MnaSolver<VarType>` is the solver almost every simulation uses. The method it implements is derived
under [nodal analysis]({{< ref "/docs/Concepts/nodal-analysis.md" >}}); what a component must
provide to take part is under
[interfacing with the MNA solver]({{< ref "../Writing a Model/mnainterface.md" >}}). This page is
the solver itself.

Note the spelling: the class is `MnaSolver` even though the file is `MNASolver.h`.

## Setting up

`initialize` runs the sequence described under
[component and solver initialization]({{< ref "../Writing a Model/initialization.md" >}}):
identify the topology objects, create sub-components, collect virtual nodes,
`assignMatrixNodeIndices`, size the matrices, initialize the components, then assemble.

Which assembly function runs depends on the network:

- `initializeSystemWithPrecomputedMatrices` when the switch combinations are few enough to
  enumerate. Every combination gets its own factorised matrix up front, so a switching event
  becomes a lookup rather than a refactorisation.
- `initializeSystemWithVariableMatrix` when a component changes its own stamp continuously and
  enumeration is impossible.
- `initializeSystemWithParallelFrequencies` for a harmonic study, where several frequencies are
  solved side by side.

`resolveSystemMatrixRecomputationMode` chooses between them when the mode is `Auto`;
`SystemMatrixRecomputationMode::Enabled` and `Disabled` force it either way.

## Stepping

`solve` does the same four things every step.

It zeroes the right-hand side and sums the stamps the components' pre-step tasks produced, which is
why a component that fails to declare its dependencies can find its contribution missing rather
than wrong. It calls `updateSwitchStatus`, which produces an index into the precomputed matrices.
It solves through the linear solver for that index. Then it hands the solution to the components'
post-step tasks.

The switch index is the point of the precomputed strategy: with the factorisations already built,
a switching event costs a different lookup rather than new numerical work. That is what makes a
network with frequent switching affordable, and it is why the number of switches is bounded in
practice, since the enumeration grows as two to the power of that number.

`solveWithSystemMatrixRecomputation` is the other path. It asks `hasVariableComponentChanged` each
step and rebuilds and refactorises only when something reports a change, which is the expensive but
general case used by variable components such as the SSN models.

Both paths hold the time step fixed. Changing it during a run is covered under
[variable time step]({{< ref "variable-time-step.md" >}}), which needs the recomputation path,
since the precomputed factorisations carry the step they were built with.

## Iterative components

After the solve, the solver checks whether any synchronous generator reports `requiresIteration`.
If so it repeats the solve step until none does, which is how the predictor-corrector and two-stage
machine models reach the implicit solution rather than its explicit approximation. Models that do
not request iteration cost nothing here.

This loop is the reason a machine model can be iterative without the whole solver being iterative.

## Linear backends

The solver does not implement its own factorisation; it selects an adapter through
`MnaSolverFactory`. The choices and their tuning are described under
[alternative solvers]({{< ref "alternative-solvers.md" >}}), which also covers the ordering and
partial-refactorisation options that matter most when the matrix changes every step.

## Instrumentation

`Solver::mLogSolveTimes` records the wall-clock duration of each solve into `mSolveTimes`, which is
the measurement to use when comparing backends or step sizes rather than timing the whole run.

## Source

`dpsim/src/MNASolver.cpp`, `dpsim/src/MNASolverDirect.cpp`, and
`dpsim/include/dpsim/MNASolverFactory.h`.
