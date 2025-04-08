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
should be stamped into the system `rightVector`:

```cpp
// DP_Ph1_PiLine.cpp
DP::Ph1::PiLine::PiLine(String uid, String name, Logger::Level logLevel)
  : Base::Ph1::PiLine(mAttributes),
  // Call the constructor of CompositePowerComp and enable automatic pre- and post-step creation
  CompositePowerComp<Complex>(uid, name, true, true, logLevel)
{
  //...
}

void DP::Ph1::PiLine::initializeFromNodesAndTerminals(Real frequency) {
  //...
  // Create series sub components
  mSubSeriesResistor = std::make_shared<DP::Ph1::Resistor>(**mName + "_res", mLogLevel);

  // Setup mSubSeriesResistor...

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
