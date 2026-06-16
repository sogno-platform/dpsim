---
title: "Subcomponent Handling"
linkTitle: "Subcomponent Handling"
date: 2022-12-14
---

In DPsim, there are many components which can be broken down into individual subcomponents. Examples are the `PiLine`, consisting of an inductor, three resistors, and two capacitors, or the `NetworkInjection` which contains a voltage source.
On the C++ class level, these subcomponents are represented by member variables within the larger component class. In this guide, all components which have subcomponents are called **composite components**.

# Creating Composite Components

While normal components are usually subclasses of `SimPowerComp<T>` or `MNASimPowerComp<T>`, there exists a special base class for composite
components called `CompositePowerComp<T>`. This class provides multiple methods and parameters for configuring how the subcomponents should be
handled with respect to the `MNAPreStep` and `MNAPostStep` tasks.
The main idea here is that the subcomponents do not register their own MNA tasks, but instead their MNA methods like `mnaPreStep` and `mnaPostStep` are called explicitly in the tasks of the composite component.
In the constructor of `CompositePowerComp<T>`, the parameters `hasPreStep` and `hasPostStep` can
be set to automatically create and register a `MNAPreStep` or `MNAPostStep` task that will call the `mnaCompPreStep` or `mnaCompPostStep` method on execution.
Additionally, all subcomponents should be registered as soon as they are created using the `addMNASubComponent`-method. This method takes
multiple parameters defining how and in what order the subcomponent's pre- and post- steps should be called, as well as if the subcomponent
should be stamped into the system `rightVector`.

## Initialization lifecycle

Composite components are initialized in three stages, each driven by the framework rather than left to ad-hoc ordering. (These are distinct from the electrical phases A/B/C of a three-phase component.)

1. **Topology stage** (`createSubComponents()`). Decides *which* sub-components exist and *how* they are wired: `make_shared`, `connect()` to network/virtual nodes, and `addMNASubComponent()`. This runs in a pre-pass before the MNA system matrix is sized, so any virtual nodes owned by sub-components are visible to `collectVirtualNodes()`. Because it runs before power-flow results or the simulation frequency are guaranteed to be available, `createSubComponents()` must not read terminal data (`initialSingleVoltage()`, `singleActivePower()`, ...), system frequency (`mFrequencies(0,0)`), or compute any power-/impedance-derived value. It must be idempotent - guard the body with `mSubCompCreated`.
1. **Parameterization stage** (`initializeParentFromNodesAndTerminals(Real frequency)`). Decides *what values* the sub-components carry. This is where terminal reads, frequency-dependent impedance/admittance calculations, and `setParameters()` calls on sub-components belong. It receives the simulation frequency as an honest argument, so there is no need to fall back on `mFrequencies(0,0)`.
1. **MNA-init stage** (`mnaCompInitialize()`). Unchanged; already recurses into sub-components.

`CompositePowerComp<VarType>::initializeFromNodesAndTerminals()` is `final` and owns the contract:

```cpp
void initializeFromNodesAndTerminals(Real frequency) final {
  createSubComponents();                            // idempotent safety net for paths
                                                     //   that reach this composite without
                                                     //   the solver's pre-pass having run
  initializeParentFromNodesAndTerminals(frequency); // parent derives values,
                                                     //   setParameters() on subs
  for (auto subComp : mSubComponents) {
    subComp->initialize(mFrequencies);              // propagate frequencies down
    subComp->initializeFromNodesAndTerminals(frequency);
  }
}
```

Concrete composites do not override `initializeFromNodesAndTerminals()` - they implement the pure-virtual `initializeParentFromNodesAndTerminals(Real frequency)` hook instead. The loop above re-enters this same `final` wrapper for any sub-component that is itself a composite, so initialization reaches arbitrary nesting depth (e.g. a composite that creates another composite as a sub-component, which in turn creates plain `R`/`L`/`C` sub-components) without each level having to manually call `initialize()`/`initializeFromNodesAndTerminals()` on its children.

A sub-component whose very existence (not just its value) depends on a value only available in the parameterization stage - e.g. choosing an inductor vs. a capacitor based on the sign of a computed reactive power - cannot use the "create in the topology stage, parametrize in the parameterization stage" split below. In that case, create *and* register the sub-component directly inside `initializeParentFromNodesAndTerminals()`. This is safe: nothing consumes `mSubComponents`/the MNA-registered sub-component list before `MnaSolver::initialize()` has finished running the parameterization stage for every component, and as long as the sub-component introduces no new virtual nodes (those must already be declared in the constructor or `setParameters()`, before `collectVirtualNodes()` runs).

Parameterization-stage value derivation must be numerically safe at degenerate inputs. Because the parent derives sub-component values from terminal power, voltage, and frequency, a zero rated power, zero reactance, or zero capacitance can turn an impedance/admittance expression into a division by zero and inject `NaN`/`inf` into the system matrix or source vector - which then propagates through every later time step. Guard such expressions explicitly: derive a branch current from an admittance (`V * jwC`) rather than from an impedance (`V / Z`), and create an optional shunt branch only when its value is strictly positive (a zero-valued branch is an open circuit and stamps nothing, so omitting it is numerically equivalent and avoids the singular impedance). A composite can otherwise build a wrong-but-non-crashing topology, so prefer an explicit check and log over relying on a value happening to be non-zero.

```cpp
// DP_Ph1_PiLine.cpp
DP::Ph1::PiLine::PiLine(String uid, String name, Logger::Level logLevel)
  : Base::Ph1::PiLine(mAttributes),
  // Call the constructor of CompositePowerComp and enable automatic pre- and post-step creation
  CompositePowerComp<Complex>(uid, name, true, true, logLevel)
{
  //...
}

void DP::Ph1::PiLine::createSubComponents() {
  if (mSubCompCreated)
    return;
  mSubCompCreated = true;

  // Create series sub components
  mSubSeriesResistor = std::make_shared<DP::Ph1::Resistor>(**mName + "_res", mLogLevel);

  // Setup mSubSeriesResistor... (only from values already known from this
  // component's own setParameters()/constructor/Attributes - no terminal or
  // frequency reads here)

  // Register the resistor as a subcomponent. The resistor's pre- and post-step will be called before the pre- and post-step of the parent,
  // and the resistor does not contribute to the `rightVector`.
  addMNASubComponent(mSubSeriesResistor, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, false);

  mSubSeriesInductor = std::make_shared<DP::Ph1::Inductor>(**mName + "_ind", mLogLevel);

  // Setup mSubSeriesInductor...

  // Register the inductor as a subcomponent. The inductor's pre- and post-step will be called before the pre- and post-step of the parent,
  // and the inductor does contribute to the `rightVector`.
  addMNASubComponent(mSubSeriesInductor, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);
  //...
}

void DP::Ph1::PiLine::initializeParentFromNodesAndTerminals(Real frequency) {
  //...
  // Frequency-dependent values are derived here, where `frequency` is an
  // honest argument, instead of in createSubComponents().
  Real omega = 2. * PI * frequency;
  Complex impedance = {**mSeriesRes, omega * **mSeriesInd};
  //...
}
```

# Orchestrating MNA Method Calls

By choosing which methods to override in the composite component class, subcomponent handling can either be offloaded to the `CompositePowerComp` base class or manually implemented in the new component class. By default, `CompositePowerComp` provides all
methods demanded by `MNAInterface` in such a way that the subcomponents' MNA-methods are properly called. To also allow for the composite
component class to perform further actions in these MNA-methods, there exist multiple methods prefixed with `mnaParent`, e.g. `mnaParentPreStep` or `mnaParentAddPostStepDependencies`.
These parent methods will usually be called after the respective method has been called on the subcomponents. For the `mnaPreStep` and
`mnaPostStep` methods, this behavior can be set explicitly in the `addMNASubComponent` method.

If a composite component requires a completely custom implementation of some MNA-method, e.g. for skipping certain subcomponents or for
calling the subcomponent's methods in a different order, the composite component class can still override the original MNA-method with the `mnaComp` prefix instead of the
`mnaParent` prefix. This will prevent the `CompositePowerComp` base class from doing any subcomponent handling in this specific MNA-method,
so the subcomponent method calls have to be performed explicitly if desired. Given this, the following two implementations of the `mnaAddPreStepDependencies` method are equivalent:

```cpp
void DP::Ph1::PiLine::mnaParentAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) {
  // Only add the dependencies of the composite component, the subcomponent's dependencies are handled by the base class
  prevStepDependencies.push_back(mIntfCurrent);
  prevStepDependencies.push_back(mIntfVoltage);
  modifiedAttributes.push_back(mRightVector);
}
```

```cpp
void DP::Ph1::PiLine::mnaCompAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) {
  // Manually add pre-step dependencies of subcomponents
  for (auto subComp : mSubcomponentsMNA) {
    subComp->mnaAddPreStepDependencies(prevStepDependencies, attributeDependencies, modifiedAttributes);
  }

  // Add pre-step dependencies of component itself
  prevStepDependencies.push_back(mIntfCurrent);
  prevStepDependencies.push_back(mIntfVoltage);
  modifiedAttributes.push_back(mRightVector);
}
```
