---
title: "Variable Time Step in MNA"
linkTitle: "Variable Time Step"
date: 2026-08-26
description: >
  Changing the time step during a run, which components follow it, and refining the step around
  scheduled events.
weight: 2
---

An extension of the [MNA solver]({{< ref "mna-solver.md" >}}) rather than a solver of its own:
assembly, stepping and the linear backends are unchanged, and a run that never changes its step
never enters this path.

`setTimeStep()` is an initialisation setting. Components derive their companion models from the step
in `mnaCompInitialize()` and keep the result, so writing the step later changes only how far the
simulation advances per step and leaves every discretisation behind.

`Simulation::updateTimeStep(Real)` changes it properly, between steps.

## How a change reaches the components

```
Simulation::updateTimeStep()   writes the step, calls every solver
  Solver::updateTimeStep()     base, records the step
  MnaSolver::updateTimeStep()  asks every component, re-stamps, refactorises
    MNAInterface::mnaUpdateTimeStep()   per component
```

A component returns `false` when it cannot follow. `MnaSolver::updateTimeStep()` returns how many
declined and names the declining types in the log, so a run that keeps a stale discretisation says
so rather than looking merely degraded. `Simulation::updateTimeStep()` passes that count back to the
caller.

`CompositePowerComp` asks every sub-component and then its own
`mnaParentUpdateTimeStep()`, which defaults to `false`. A composite that owns no step-dependent
state overrides it to return `true`; one that owns a controller or its own discretisation converts
that state first. The result is the conjunction, so a composite declines if any part of it does.

{{% alert title="Watch out: the base matrix is not refreshed by a step change on its own" color="warning" %}}
`recomputeSystemMatrix()` runs only when a *variable* component reports a change, and a step change
is not that. `MnaSolverDirect::refreshStaticMatrixStamp()` therefore re-stamps the base matrix,
which carries the static components with the old step, and factorises **fully** rather than through
`partialRefactorize()`, whose entry list covers only the variable components.
{{% /alert %}}

With precomputed switch matrices there is nothing to re-stamp from — every switched matrix was
factorised with the old step — so the change is refused with a warning. Enable system-matrix
recomputation to change the step.

## What a component has to do

Three shapes cover almost everything:

- **Nothing depends on the step.** Return `timeStep > 0`. Resistors, ideal sources, series switches.
- **A companion model.** Re-derive the factors and rebuild the history term from the *present*
  voltage and current. `DP::Ph1::Inductor` and `ResIndSeries` call `initVars()` for this but
  preserve `mIntfCurrent` across the call: `initVars()` overwrites it, which is right at startup
  where the current comes from the load flow and wrong mid-run where it is the state. The `Ph3`
  inductor and capacitor need no such care, since their `initVars()` leaves the interface current
  alone.
- **A rotating machine.** Re-derive the constants, and re-anchor the rotor angle by
  `mBase_OmMech * (oldTimeStep - timeStep)`. `mnaCompPreStep()` advances `mThetaMech` to the end of
  the step and then passes the start time, so the dq transform argument carries a `w0*dt` term. That
  term is constant while the step is, and jumps once when it changes, rotating the machine against
  the network. The correction takes no factor `w`, so the real slip against the shift frame
  survives. The VBR models additionally correct the history EMF for the new coefficients through
  `adjustVBRHistoryForNewTimeStep()`.

## Refining the step around events

```cpp
sim.setEventRefinement(fineTimeStep, leadTime, followTime);
```

The fine step is used from `leadTime` before a scheduled event until `followTime` after it, and the
step given to `setTimeStep()` outside that window. `fineTimeStep <= 0` disables it. Event times are
collected when the event is added, so no trigger logic is needed. `applyEventRefinement()` evaluates
the window at the start of each step and goes through `updateTimeStep()`, so declining components
are reported as usual.

## Scope

- **DP only.** `mnaUpdateTimeStep()` defaults to `false`, so an EMT or SP run declines everything
  and warns. Coverage is `DP::Ph1` and `DP::Ph3`: passive elements, ideal sources, the passive
  containers, the switches, and the reduced-order and PCM/TPM machines.
- **Still declining in DP**, deliberately: the converter families, `SynchronGeneratorTrStab`,
  `SynchronGeneratorIdeal`, `PQLoadCS`, `RXLoadSwitch`, `VoltageSourceRamp` and
  `Ph3::SynchronGeneratorDQODE`, whose ODE solver holds its own step-dependent state.
- **Scheduled events only.** A switching action driven from inside a component, such as a protection
  trip, never enters the event queue and opens no window.

## The test

`DP_VarTimeStep` (`dpsim/examples/cxx/Components/`) checks six things and returns a non-zero exit
code on failure:

| check | measured |
|---|---|
| a change to the step already in use is a no-op | deviation `0` |
| a coarse-to-fine change matches a run held fine | `4.4e-10` |
| declining components are counted | `0` passive, `1` with a ramped source |
| a refined window recovers a switching transient | peak error `2.9e-2` at the base step, `5.3e-8` refined |
| a `Ph3` network converts completely | `0` declining |
| a machine keeps its rotor angle | terminal voltage deviation `2.3e-11` |

The machine check is a 24 kV SMIB with a 4th-order VBR generator. With the rotor re-anchoring
removed the same check reads `2.3e-1`, so it measures that correction rather than passing by
construction.
