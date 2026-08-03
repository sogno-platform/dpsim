---
title: "Adding Tasks to a Component"
linkTitle: "Adding Tasks"
weight: 6
description: >
  Giving a component pre-step and post-step tasks and declaring their dependencies.
---

How to attach tasks to a component. For how the scheduler consumes them, see
[scheduling]({{< ref "/docs/Developer Guide/Attributes and Scheduling/Scheduling/index.md" >}}).

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

{{% alert title="Watch out: a missing declaration produces wrong results, not a crash" color="warning" %}}
Missing a declaration does not always cause a crash; it silently produces incorrect results or a wrong execution order, which is harder to debug.
Two common failure modes follow from the pruning step:

- A `PreStep` or `PostStep` task is dropped entirely because none of its declared modified attributes is needed by another task, a logger, an interface, or a previous-step dependency. The simulation then runs but its results are always wrong.
- The same task appears to work only when a particular variable is logged or exchanged by an interface, because that logger or interface adds a dependency on the attribute and keeps the producing task reachable. The results then depend on logger or interface configuration even though the physical model did not change.

Declare dependencies conservatively.
{{% /alert %}}
