---
title: "Task Scheduling"
linkTitle: "Scheduling"
weight: 3
date: 2020-05-01
description: >
  How DPsim builds, orders, and executes the task graph each timestep.
---

Within each simulation timestep, DPsim executes a set of **tasks**: discrete units of computation contributed by components, the solver, interfaces, and loggers.
Before the first timestep the scheduler collects all tasks, resolves their data dependencies into a directed acyclic graph, and produces an ordered schedule.
That schedule is then replayed on every timestep with no further graph analysis.

---

# Tasks

## The Task base class

Every task is an instance of a class that inherits from `CPS::Task`
(`dpsim-models/include/dpsim-models/Task.h`).
Each subclass implements one member function:

```cpp
virtual void execute(Real time, Int timeStepCount) = 0;
```

To participate in scheduling, a task declares its data dependencies through three attribute lists that are populated in the task's constructor:

| List | Meaning |
|------|---------|
| `mAttributeDependencies` | Attributes this task *reads* in `execute()` |
| `mModifiedAttributes` | Attributes this task *writes* in `execute()` |
| `mPrevStepDependencies` | Attributes whose value from the *previous* timestep this task needs |

All three lists hold `AttributeBase::Ptr` objects, the same pointers used throughout the component model.
See [Attributes]({{< ref "../Attributes/index.md" >}}) for details on the attribute system.

Only attributes can participate in scheduling.
Plain C++ member variables (a `Real`, a `Matrix`, an internal state struct) are invisible to the scheduler, so no dependency edge can be formed around them.
The same constraint applies to simulation data recording: both the CSV logger (`DataLogger`) and the real-time data logger (`RealTimeDataLogger`) implement `DataLoggerInterface`, whose `logAttribute()` member function only accepts `AttributeBase::Ptr`.
The VILLASnode interface works the same way.
Any value that needs to cross a task boundary, be recorded to a data file, or be exchanged with an external tool must be stored in an `Attribute<T>`.

The component text logger (`CPS::Logger`, backed by spdlog) is a separate mechanism used for human-readable debug and diagnostic output.
It is not part of the scheduling system and can print any value regardless of whether it is an attribute.

For practical rules on when a variable should be an attribute versus a plain member variable, see [Attribute Usage Guidelines]({{< ref "../../Development/attribute-usage.md" >}}).

## Common component task conventions

The names below are component and solver conventions, not scheduler-level concepts.
The scheduler only sees the attribute dependencies a task declares; it has no notion of a "PreStep" or "PostStep" and never orders tasks by these names.

MNA components typically define two task classes per component:

| Task | Typical responsibility |
|------|----------------------|
| `MnaPreStep` | Component-specific preparation before the matrix solve, often updating internal state and stamping the right-hand-side contribution |
| `MnaPostStep` | Component-specific update after the matrix solve, often reading the solution vector to update interface voltages and currents |

This is a common pattern rather than a fixed rule; the exact work each task does is component-specific.
Signal-domain components (regulators, governors, control blocks) define their own task list via `getTasks()`; many separate previous-step state handling from output updates, for example a `PreStep` that copies state from the previous step and a `Step` that updates the block outputs.

The solver itself contributes a task that solves the MNA system; individual components do not depend on it by name, they depend on `leftVector` instead (see below).

---

# Building the schedule

## Task collection

`Simulation::prepSchedule()` collects all tasks before the first timestep from three top-level sources:

- **Solvers**: each solver contributes its task list via `Solver::getTasks()`. For MNA solvers this list bundles:
   - the matrix-solve task,
   - MNA component pre-/post-step tasks from `MNASimPowerComp::mnaTasks()` (built during solver initialization via `mnaAddPreStepDependencies()` / `mnaAddPostStepDependencies()`),
   - signal-domain component tasks returned by `SimSignalComp::getTasks()`,
   - optional solver-side tasks, such as state-space extraction, when enabled.
- **Interfaces**: each interface contributes its own tasks via `Interface::getTasks()`. These typically depend on the attributes exchanged with external systems.
- **Loggers**: each logger contributes a logging task via `Logger::getTask()`, depending on the logged attributes so values are written after the producing tasks have run.

All tasks are placed in a flat `Task::List` and handed to the scheduler.

## Dependency resolution

`Scheduler::resolveDeps()` (`dpsim/src/Scheduler.cpp`) translates the attribute-level declarations into directed edges between tasks.
For every attribute in `mModifiedAttributes`, it finds all tasks that list that attribute in their `mAttributeDependencies` and adds an edge:

```
task A  writes  attr_X
task B  reads   attr_X
────────────────────────
 edge:  A → B   (A must run before B)
```

A special `Root` sentinel task is inserted as a sink for all `mPrevStepDependencies` entries.
Its role is explained in the pruning step below.

## Topological sort and pruning

`Scheduler::topologicalSort()` first runs a backward breadth-first search (BFS) from `Root`, marking every task that transitively contributes to a simulation output.
Tasks not reachable in this pass are **dropped** from the schedule because they produce data no downstream consumer reads in the current timestep.

Kahn's algorithm then processes the remaining tasks in dependency order and appends them to the schedule.
The result is a flat, ordered list in which every task appears after all of its current-step predecessors.

The `Root` sentinel matters here: it holds a reference to an external attribute updated by an interface or by the solver, so the backward BFS reaches it and keeps every task that writes previous-timestep state, even when that output is only consumed in the next timestep.

## Level scheduling

For parallel execution the ordered list is converted into levels by `Scheduler::levelSchedule()`.
Each task is assigned to the level one greater than the highest-level task it depends on:

```
level 0 │ T1   T2   T3        ← no dependencies; can all start at once
level 1 │    T4       T5      ← depend only on level-0 tasks
level 2 │       T6            ← depends on T4 or T5
```

Tasks within the same level have no data dependencies between them and can execute in parallel.
The scheduler guarantees that all tasks in level *k* finish before any task in level *k+1* starts.

![image](task_graph_levels.svg)

## Scheduler variants

| Class | Parallelism strategy |
|-------|---------------------|
| `SequentialScheduler` | Single-threaded; follows topological order |
| `ThreadLevelScheduler` | Distributes each level across *N* worker threads |
| `ThreadListScheduler` | Distributes tasks greedily across *N* threads |
| `OpenMPLevelScheduler` | Uses `#pragma omp parallel for` per level |

The scheduler is chosen at `Simulation` construction time; `SequentialScheduler` is the default.

## Per-timestep execution

`Scheduler::step(time, timeStepCount)` is called once per timestep.
For the sequential scheduler:

```cpp
for (auto& task : mSchedule)
    task->execute(time, timeStepCount);
```

Parallel schedulers distribute tasks across threads within each level and synchronize with a barrier before advancing to the next level.

---

# Developer guide: adding tasks to a component

## Signal components

Signal components inherit from `SimSignalComp` and return their tasks from `getTasks()`.
The usual pattern is to define inner `Task` classes whose constructors populate the dependency lists, then instantiate them in `getTasks()`:

```cpp
class MyComponent : public SimSignalComp {
public:
    const Attribute<Real>::Ptr mInput;       // written by upstream component
    const Attribute<Real>::Ptr mOutput;      // read by downstream component
    const Attribute<Real>::Ptr mOutputPrev;  // state carried across timesteps

    class PreStep : public Task {
    public:
        explicit PreStep(MyComponent& comp)
            : Task(**comp.mName + ".PreStep"), mComp(comp) {
            mPrevStepDependencies.push_back(mComp.mOutput);
            mModifiedAttributes.push_back(mComp.mOutputPrev);
        }
        void execute(Real time, Int timeStepCount) override {
            **mComp.mOutputPrev = **mComp.mOutput;
        }
    private:
        MyComponent& mComp;
    };

    class Step : public Task {
    public:
        explicit Step(MyComponent& comp)
            : Task(**comp.mName + ".Step"), mComp(comp) {
            mAttributeDependencies.push_back(mComp.mInput);
            mModifiedAttributes.push_back(mComp.mOutput);
        }
        void execute(Real time, Int timeStepCount) override {
            mComp.signalStep(time, timeStepCount);
        }
    private:
        MyComponent& mComp;
    };

    Task::List getTasks() override {
        return { std::make_shared<PreStep>(*this),
                 std::make_shared<Step>(*this) };
    }
};
```

`PreStep` uses `mPrevStepDependencies` for `mOutput` because it reads the value produced *last* timestep, not the value that `Step` will produce *this* timestep.
Using `mAttributeDependencies` here would create a same-step dependency on `Step` and force `PreStep` after `Step`, which is backwards.

## MNA power components

MNA components inherit from `MNASimPowerComp<VarType>`.
Instead of `getTasks()`, they implement two hook functions that `MNASimPowerComp` calls when it builds the `MnaPreStep` and `MnaPostStep` tasks during solver initialization.

```cpp
void DP::Ph1::MyComponent::mnaAddPreStepDependencies(
    AttributeBase::List& prevStepDependencies,
    AttributeBase::List& attributeDependencies,
    AttributeBase::List& modifiedAttributes) {

    prevStepDependencies.push_back(mIntfCurrent);   // read from previous step
    modifiedAttributes.push_back(mRightVector);     // stamp right-hand side
}

void DP::Ph1::MyComponent::mnaAddPostStepDependencies(
    AttributeBase::List& prevStepDependencies,
    AttributeBase::List& attributeDependencies,
    AttributeBase::List& modifiedAttributes,
    Attribute<Matrix>::Ptr& leftVector) {

    attributeDependencies.push_back(leftVector);  // wait for matrix solve
    modifiedAttributes.push_back(mIntfVoltage);
    modifiedAttributes.push_back(mIntfCurrent);
}
```

`PostStep` must always list `leftVector` in `attributeDependencies`.
This creates the edge from the solver's matrix-solve task to every component's `PostStep`, ensuring the solution vector is available before voltages and currents are extracted.

## Dependency declaration checklist

- Every attribute *read* inside `execute()` must appear in `mAttributeDependencies` or `mPrevStepDependencies`.
- Every attribute *written* inside `execute()` must appear in `mModifiedAttributes`.
- State carried from the previous timestep goes in `mPrevStepDependencies`, not `mAttributeDependencies`.
- `MnaPostStep` must list `leftVector` in `attributeDependencies`.
- No attribute should appear in both `mAttributeDependencies` and `mPrevStepDependencies` for the same task.

Missing a declaration does not always cause a crash; it silently produces incorrect results or a wrong execution order, which is harder to debug.
Two common failure modes follow from the pruning step:

- A `PreStep` or `PostStep` task is dropped entirely because none of its declared modified attributes is needed by another task, a logger, an interface, or a previous-step dependency. The simulation then runs but its results are always wrong.
- The same task appears to work only when a particular variable is logged or exchanged by an interface, because that logger or interface adds a dependency on the attribute and keeps the producing task reachable. The results then depend on logger or interface configuration even though the physical model did not change.

Declare dependencies conservatively.
