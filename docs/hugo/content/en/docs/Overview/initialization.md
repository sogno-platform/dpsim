---
title: "Component and Solver Initialization"
linkTitle: "Initialization"
date: 2026-06-18
description: >
  How DPsim initializes components and solvers before the first simulation timestep.
---

Initialization is the phase between constructing the system topology and running the first timestep.
Its job is to size the system matrices, derive initial state from power-flow results, register MNA tasks, and stamp static conductances.
Two constraints drive its structure:

- The system matrix size depends on the total number of simulation nodes, including **virtual nodes** declared by composite components and their sub-components. All virtual nodes must therefore be known *before* the matrices are allocated.
- Component parameter values (impedances, initial phasors) depend on terminal voltages and powers, which are only available *after* a power-flow solve.

These two constraints impose an ordering that is captured in the solver's initialization sequence.

---

# MNA Solver Initialization Sequence

`MnaSolver::initialize()` executes the following steps in order.

{{< mermaid >}}
flowchart TD
    start([Simulation::run]) --> init[MnaSolver::initialize]
    init --> s1["S1: identifyTopologyObjects()\nSort into mMNAComponents,\nmSimSignalComps, ..."]
    s1 --> s2["S2: createSubComponents() pre-pass\nRecursively instantiate sub-components\nso all virtual nodes exist"]
    s2 --> s3["S3: collectVirtualNodes()\nassignMatrixNodeIndices()\nMatrix size is now fixed"]
    s3 --> s4["S4: createEmptyVectors()\ncreateEmptySystemMatrix()"]
    s4 --> s5a["S5a: initializeFromNodesAndTerminals(freq)\nfor each SimPowerComp"]
    s5a --> s5b["S5b: initialize(omega, dt)\nfor each SimSignalComp"]
    s5b --> s5c["S5c: mnaInitialize(omega, dt, v)\nfor each MNAInterface component"]
    s5c --> cond{mSteadyStateInit?}
    cond -- yes --> s6["S6: steadyStateInitialization()\nIterate MNA until phasors converge"]
    s6 --> s7
    cond -- no --> s7["S7: setBehaviour(MNASimulation)\non all components"]
    s7 --> s8["S8: initializeSystem()\nStamp static elements,\ncompute LU factorizations"]
    s8 --> done([Ready for timesteps])
{{< /mermaid >}}

## Step 1 — Identify topology objects

`identifyTopologyObjects()` iterates over `SystemTopology::mComponents` and sorts each component into one of four lists:

| List | Contents |
|------|----------|
| `mMNAComponents` | Static MNA power components |
| `mMNAIntfVariableComps` | Variable-stamp MNA components (e.g. under `MNAVariableCompInterface`) |
| `mMNAIntfSwitches` | Components with a switch interface |
| `mSimSignalComps` | Signal components (`SimSignalComp`) |

Ground nodes are excluded here.

## Step 2 — Create sub-components (pre-pass)

Before the matrix can be sized, every composite component's sub-component tree must be fully instantiated so that all virtual nodes are visible.
The solver calls `createSubComponents()` recursively on every MNA component:

- Only sub-components *newly registered* by this call are recursed into, because eagerly-constructed sub-components (created in the constructor before `connect()` has run) are not yet safe to recurse into.
- This step is a pre-pass only — it must not set parameter values derived from terminal data or frequency.

For details on the three-stage composite lifecycle (`createSubComponents`, `initializeParentFromNodesAndTerminals`, `mnaCompInitialize`), see [Subcomponent Handling]({{< ref "./subcomponents.md" >}}).

## Step 3 — Collect virtual nodes and assign indices

`collectVirtualNodes()` visits every component and calls `virtualNodes()` to collect all virtual `SimNode` objects, then appends them to the solver's node list.
`assignMatrixNodeIndices()` then assigns a contiguous integer index to every simulation node (real and virtual), which determines the row/column layout of the system matrices.

After this step the matrix size is fixed.

## Step 4 — Allocate empty matrices

`createEmptyVectors()` and `createEmptySystemMatrix()` allocate the left-side vector, right-side vector, system matrix (dense or sparse depending on the solver variant), and switch-variant copies.
For sparse solvers, `mBaseSystemMatrix` and `mLuFactorizations` are also allocated here, with one variant per switch combination.

## Step 5 — Initialize components (`initializeComponents`)

This step has three sub-passes over the component lists.

### 5a — Power components: `initializeFromNodesAndTerminals`

For every `SimPowerComp<VarType>` in `mMNAComponents` and `mMNAIntfVariableComps`:

1. `checkForUnconnectedTerminals()` validates connectivity.
1. If `mInitFromNodesAndTerminals` is set (the default), `initializeFromNodesAndTerminals(mSystem.mSystemFrequency)` is called.

This is where components read their terminal voltages and powers and derive physical parameters (impedances, initial phasor values, per-unit quantities).
For composite components `initializeFromNodesAndTerminals()` is `final` in `CompositePowerComp` and sequences the three lifecycle stages automatically; non-composite power components override it directly.

### 5b — Signal components: `initialize(omega, timeStep)`

Each `SimSignalComp` in `mSimSignalComps` receives `initialize(mSystem.mSystemOmega, mTimeStep)`.
This is the hook for signal-domain components (regulators, governors, PSS blocks) to allocate their state buffers, set initial values, and wire up attribute connections.

> **Naming constraint.** Do not use `initialize(Real)` or `initialize(Real, Real)` as a user-facing initialization hook for power components — those signatures match the solver's signal-comp hook and will be called by the solver rather than by the component author's intent. Use `initializeFromNodesAndTerminals()` or a named method such as `initializeStates()` instead.

### 5c — MNA components: `mnaInitialize`

Each MNA component (including switches) receives `mnaInitialize(omega, timeStep, leftVector)`.
In `MNASimPowerComp` this method:

1. Clears and re-registers `MNAPreStep` / `MNAPostStep` tasks according to the `hasPreStep` / `hasPostStep` flags.
1. Initializes `mRightVector` to zero with the correct size.
1. Calls `mnaCompInitialize(omega, timeStep, leftVector)` on the component.

In `mnaCompInitialize`, component classes call `updateMatrixNodeIndices()` and perform any one-time MNA setup that requires the final node layout (e.g. allocating per-component history vectors sized to the system).

Nodes are initialized last via `SimNode::initialize()`, which zeros the node voltage.

## Step 6 — Optional steady-state initialization

If `mSteadyStateInit` is set, `steadyStateInitialization()` iterates the MNA solve until the phasor solution converges.
The flag `mIsInInitialization` is set to `true` for this sub-phase so that components can distinguish initialization solves from simulation solves via `mBehaviour` (see below).

## Step 7 — Set simulation behaviour

After initialization solves are complete, the solver calls `setBehaviour(TopologicalPowerComp::Behaviour::MNASimulation)` on every `TopologicalPowerComp` and `setBehaviour(SimSignalComp::Behaviour::Simulation)` on every `SimSignalComp`.

The `Behaviour` enum (defined in `TopologicalPowerComp`) has three values:

| Value | When active | Typical use |
|-------|-------------|-------------|
| `Behaviour::Initialization` | During PF steady-state init pass | Components may disable transient update equations |
| `Behaviour::PFSimulation` | During PFSolver run | Activates power-flow-specific stamping |
| `Behaviour::MNASimulation` | After `initialize()` completes | Normal simulation; components should be in their run-time mode |

Components that need different behaviour between initialization and simulation check `mBehaviour` in their pre/post-step methods or in `mnaCompPreStep`.

## Step 8 — Initialize system matrices (`initializeSystem`)

`initializeSystem()` selects one of three paths:

- **Parallel frequencies** (`initializeSystemWithParallelFrequencies`): stamps each frequency into a separate thread.
- **Variable matrix** (`initializeSystemWithVariableMatrix`): used by `MnaSolverSysRecomp`; saves static switch matrices as base matrices and adds variable elements on top.
- **Precomputed matrices** (`initializeSystemWithPrecomputedMatrices`): the common path. Calls `switchedMatrixStamp()` for each switch combination, which iterates over all static MNA components and calls `mnaApplySystemMatrixStamp()` and `mnaApplyRightSideVectorStamp()`. LU factorizations are computed for each variant.

After this step the solver is ready to execute timesteps.

---

# Component Class Hierarchy and Init Hooks

The following diagram shows which initialization methods live in which class, and the override points for component authors.

{{< mermaid >}}
classDiagram
    class TopologicalPowerComp {
        +Behaviour mBehaviour
        +setBehaviour(b)
    }
    class SimPowerComp~T~ {
        +initialize(Matrix frequencies)
        +initializeFromNodesAndTerminals(Real freq)
        +virtualNodes()
    }
    class MNASimPowerComp~T~ {
        +mnaInitialize(omega, dt, v) final
        +mnaCompInitialize(omega, dt, v)*
        +mnaCompApplySystemMatrixStamp()*
        +mnaCompPreStep()*
        +mnaCompPostStep()*
    }
    class CompositePowerComp~T~ {
        +createSubComponents()*
        +initializeFromNodesAndTerminals(freq) final
        +initializeParentFromNodesAndTerminals(freq)*
        +mnaParentInitialize(omega, dt, v)*
        +mnaParentPreStep()*
        +mnaParentPostStep()*
    }
    class SimSignalComp {
        +initialize(Real omega, Real dt)*
    }
    TopologicalPowerComp <|-- SimPowerComp
    SimPowerComp <|-- MNASimPowerComp
    MNASimPowerComp <|-- CompositePowerComp
{{< /mermaid >}}

Methods marked `*` are the virtual override points for component authors.
Methods marked `final` must not be overridden; the base class sequences them correctly.

---

# Component Method Contracts

The table below summarizes which initialization method has which responsibilities. A tick means the operation *belongs* in that method; a cross means it must not appear there.

| Responsibility | Constructor / `setParameters` | `createSubComponents` | `initializeFromNodesAndTerminals` | `mnaCompInitialize` |
|---|:---:|:---:|:---:|:---:|
| Declare virtual node count | ✓ | — | — | — |
| Allocate sub-component objects | — | ✓ | — | — |
| `connect()` sub-components to virtual nodes | — | ✓ | — | — |
| `addMNASubComponent()` registration | — | ✓ | — | — |
| Read terminal voltage / power | ✗ | ✗ | ✓ | — |
| Read system frequency | ✗ | ✗ | ✓ (via argument) | ✓ (via `omega`) |
| Compute impedance / admittance | — | ✗ | ✓ | — |
| Call `setParameters()` on sub-components | — | — | ✓ | — |
| Call `updateMatrixNodeIndices()` | — | — | — | ✓ |
| Allocate per-step vectors (history, right vector) | — | — | — | ✓ |
| Register MNA tasks (handled by base class) | — | — | — | ✓ (via `mnaCompInitialize`) |

## Common pitfalls

- **Accessing terminals in the constructor or `createSubComponents`**: terminal data (initial voltage, connected power) is not yet populated. The topology is set up but power-flow has not run.
- **Accessing `mFrequencies(0,0)` in `createSubComponents`**: the system frequency matrix is set on `SimPowerComp` via `initialize(Matrix)` which only runs later. Use the `frequency` argument passed to `initializeParentFromNodesAndTerminals` or the `omega` argument in `mnaCompInitialize`.
- **Zero-valued shunt branches**: a capacitor or reactor with zero admittance injects a zero row/column into the system matrix, which makes the LU factorization singular. Guard with a strict `> 0` check and omit the branch rather than inserting a zero stamp.
- **Late-registered virtual nodes**: any virtual node that appears for the first time *after* `collectVirtualNodes()` (Step 3) will not have a matrix index assigned, and the solver will crash or silently produce wrong results. All virtual nodes must be declared in the constructor or `setParameters()`.

---

# Composite Component Initialization Sequence

The following diagram shows how the solver and a composite component interact during initialization. For further details see [Subcomponent Handling]({{< ref "./subcomponents.md" >}}).

{{< mermaid >}}
sequenceDiagram
participant MNA as MnaSolver
participant CC as CompositePowerComp
participant SC as SubComponent

Note over MNA,SC: Step 2 - pre-pass (topology only)
MNA->>CC: createSubComponents()
CC->>SC: make_shared + connect() + addMNASubComponent()

Note over MNA,SC: Step 3 - matrix sizing
MNA->>CC: collectVirtualNodes()
MNA->>MNA: assignMatrixNodeIndices()

Note over MNA,SC: Step 5a - parameterization
MNA->>CC: initializeFromNodesAndTerminals(freq)
CC->>CC: createSubComponents() idempotent guard
CC->>CC: initializeParentFromNodesAndTerminals(freq)
CC->>SC: initialize(frequencies)
CC->>SC: initializeFromNodesAndTerminals(freq)

Note over MNA,SC: Step 5c - MNA setup
MNA->>CC: mnaInitialize(omega, dt, v)
CC->>SC: mnaInitialize(omega, dt, v)
CC->>CC: mnaParentInitialize(omega, dt, v)
{{< /mermaid >}}

---

# PFSolver Initialization

`PFSolver::initialize()` follows a simpler sequence because it operates only on single-phase SP components with no sub-component tree and does not need a `createSubComponents` pre-pass.

{{< mermaid >}}
flowchart TD
    pf[PFSolver::initialize] --> p1[Classify components\ninto generator/load/line/... lists]
    p1 --> p2[setBaseApparentPower\nCompute per-unit base]
    p2 --> p3[assignMatrixNodeIndices]
    p3 --> p4[initializeComponents\ninitializeFromNodesAndTerminals\ncalculatePerUnitParameters]
    p4 --> p5[determinePFBusType\nPQ / PV / VD]
    p5 --> p6[determineNodeBaseVoltages]
    p6 --> p7[composeAdmittanceMatrix\nBuild Y-bus]
    p7 --> done([Ready to solve power flow])
{{< /mermaid >}}

`PFSolver::setSolverAndComponentBehaviour()` is the equivalent of Step 7 for the MNA solver: it calls `setBehaviour(Behaviour::PFSimulation)` or `setBehaviour(Behaviour::Initialization)` on all components to allow them to switch stamping modes.

---

# Known Design Issues (issue #59)

The following areas were identified in [GitHub issue #59](https://github.com/sogno-platform/dpsim/issues/59) as needing improvement.

## `SimPowerComp::initialize(Matrix frequencies)` naming clash

`SimPowerComp<T>::initialize(Matrix frequencies)` is called by the solver to propagate frequency information down the component tree.
It is *not* a hook for component authors — a component that overrides it takes over responsibility for calling the base class version, which is easy to forget.
The recommended path is:

- For power components, use `initializeFromNodesAndTerminals()` or `initializeParentFromNodesAndTerminals()`.
- For signal components, use the `initialize(Real omega, Real timeStep)` hook provided by `SimSignalComp`.
- For anything else (e.g. setting up state-space matrices), add a named helper called from one of the above.

The base implementation of `SimPowerComp::initialize(Matrix)` should be renamed to something that cannot be accidentally overridden (e.g. `propagateFrequencies()`), and an `override` guard should be added to catch accidental overrides.

## Sub-component construction in constructors

Some components create and register sub-components eagerly in their constructor before `connect()` has been called on those sub-components.
This works today because the solver's `createSubComponents` pre-pass skips already-registered sub-components, but it couples topology creation to object construction and makes components harder to reason about.
The long-term goal is to migrate all sub-component construction to `createSubComponents()`, giving a clear rule: the constructor only allocates and the topology stage wires.

## Signal component `initialize` not sequenced with power flow

Signal components receive `initialize(omega, timeStep)` *after* `initializeFromNodesAndTerminals` on power components but *before* the MNA tasks are registered.
If a signal component's initial state depends on the power-flow solution (e.g. an exciter initializing to match the generator terminal voltage), it must read the relevant attribute values directly — there is no formal mechanism today to express this dependency in the initialization sequence.
A future improvement would be to give signal components access to the settled power-flow solution before their `initialize` is called.
