---
title: "Attributes"
linkTitle: "Attributes"
weight: 3
date: 2020-05-01
---

In DPsim, an attribute is a special kind of variable which usually stores a scalar or matrix value used in the simulation.
Examples for attributes are the voltage of a node, the reference current of a current source, or the left and right vectors of the MNA matrix system.
In general, attributes are instances of the `Attribute<T>` class, but they are usually stored and accessed through a custom smart pointer of type
`const AttributeBase::Ptr` (which expands to `const AttributePointer<AttributeBase>`).

Through the template parameter `T` of the `Attribute<T>` class, attributes can have different value types, most commonly `Real`, `Complex`, `Matrix`, or `MatrixComp`. Additionally, attributes can fall into one of two categories:
**Static** attributes have a fixed value which can only be changed explicitly through the attribute's `set`-method or through a mutable reference obtained through `get`.
**Dynamic** attributes on the other hand can dynamically re-compute their value from other attributes every time they are read. This can for example be used to create a scalar attribute of type `Real` whose value always contains the magnitude of another, different attribute of type `Complex`.

Any simulation component or class which inherits from `IdentifiedObject` contains an instance of an `AttributeList`.
This list can be used to store all the attributes present in this component and later access them via a `String` instead of having to use the member variable directly.
For reasons of code clarity and runtime safety, the member variables should still be used whenever possible.

## Creating and Storing Attributes
Normally, a new attribute is created by using the `create` or `createDynamic` method of an `AttributeList` object.
These two methods will create a new attribute of the given type and insert it into the `AttributeList` under the given name. After the name, `create` can take an additional parameter of type `T` which will be used as the initial value for this attribute.
Afterwards, a pointer to the attribute is returned which can then be stored in a component's member variable. Usually this is done in the
component's constructor in an initialization list:

```cpp
/// Component class Base::Ph1::PiLine

public:
  // Definition of attributes
  const Attribute<Real>::Ptr mSeriesRes;
  const Attribute<Real>::Ptr mSeriesInd;
  const Attribute<Real>::Ptr mParallelCap;
  const Attribute<Real>::Ptr mParallelCond;

// Component constructor: Initializes the attributes in the initialization list
Base::Ph1::PiLine(CPS::AttributeList::Ptr attributeList) :
  mSeriesRes(attributeList->create<Real>("R_series")),
  mSeriesInd(attributeList->create<Real>("L_series")),
  mParallelCap(attributeList->create<Real>("C_parallel")),
  mParallelCond(attributeList->create<Real>("G_parallel")) { };
```

When a class has no access to an `AttributeList` object (for example the `Simulation` class), attributes can instead be created through the
`make` methods on `AttributeStatic<T>` and `AttributeDynamic<T>`:

```cpp
// Simulation class
Simulation::Simulation(String name,	Logger::Level logLevel) :
	mName(AttributeStatic<String>::make(name)),
	mFinalTime(AttributeStatic<Real>::make(0.001)),
	mTimeStep(AttributeStatic<Real>::make(0.001)),
	mSplitSubnets(AttributeStatic<Bool>::make(true)),
	mSteadyStateInit(AttributeStatic<Bool>::make(false)),
	//...
{
	// ...
}
```

## Working with Static Attributes
As stated above, the value of a static attribute can only be changed through the attribute's `set`-method or by writing its value through a mutable reference obtained by calling `get`. This means that the value will not change between consecutive reads. Because of the performance benefits static
attributes provide over dynamic attributes, attributes should be static whenever possible.

The value of a static attribute can be read by using the attribute's `get`-function (i.e. `attr->get`) or by applying the `*` operator on the already dereferenced pointer (i.e. `**attr`), which is overloaded to also call the `get` function. Both methods return a **mutable reference** to the attribute's value of type `T&`:

```cpp
AttributeBase::Ptr attr = AttributeStatic<Real>::make(0.001);
Real read1 = attr->get(); //read1 = 0.001
Real read2 = **attr; //read2 = 0.001
Real& read3 = **attr; //read3 = 0.001
```

The value of an attribute can be changed by either writing to the mutable reference obtained from `get`, or by calling the `set`-method:
```cpp
AttributeBase::Ptr attr = AttributeStatic<Real>::make(0.001);
Real read1 = **attr; //read1 = 0.001
**attr = 0.002;
Real read2 = **attr; //read2 = 0.002
attr->set(0.003);
Real read3 = **attr; //read3 = 0.003
```

## Working with Dynamic Attributes
In general, dynamic attributes can be accessed via the same `get` and `set`-methods described above for static attributes. However,
dynamic attributes can additionally have **dependencies** on other attributes which affect the behavior of these methods.
Usually, this is used to dynamically compute the attribute's value from the value of another attribute. In the simplest case, a dynamic
attribute can be set to **reference** another (static or dynamic) attribute using the `setReference`-method. After this method has been called,
the dynamic attribute's value will always reflect the value of the attribute it references:
```cpp
AttributeBase::Ptr attr1 = AttributeStatic<Real>::make(0.001);
AttributeBase::Ptr attr2 = AttributeDynamic<Real>::make();

attr2->setReference(attr1);

Real read1 = **attr2; //read1 = 0.001
**attr1 = 0.002;
Real read2 = **attr2; //read2 = 0.002
```

When working with references between multiple dynamic attributes, the direction in which the references are defined can be important:
References should always be set in such a way that the reference relationships form a one-way chain. Only the last attribute in such a reference chain (which itself does not reference anything) should be modified by external code (i.e. through mutable references or the `set`-method). This ensures that changes are always reflected in all attributes in the chain. For example, the following setup might lead to errors because it overwrites an existing reference:
```cpp
// Overwriting an existing reference relationship
AttributeBase::Ptr A = AttributeDynamic<Real>::make();
AttributeBase::Ptr B = AttributeDynamic<Real>::make();
AttributeBase::Ptr C = AttributeDynamic<Real>::make();

B->setReference(A); // Current chain: B -> A
B->setReference(C); // Current chain: B -> C, reference on A is overwritten

**C = 0.1; // Change will not be reflected in A
```

Correct implementation:

```cpp
AttributeBase::Ptr A = AttributeDynamic<Real>::make();
AttributeBase::Ptr B = AttributeDynamic<Real>::make();
AttributeBase::Ptr C = AttributeDynamic<Real>::make();

B->setReference(A); // Current chain: B -> A
C->setReference(B); // Current chain: C -> B -> A

**A = 0.1; // Updating the last attribute in the chain will update A, B, and C

```

Aside from setting references, it is also possible to completely recompute a dynamic attribute's value every time it is read. This can for example be used to create attributes which reference a single matrix coefficient of another attribute, or which represent the magnitude or phase of a complex attribute.
Dynamic attributes which depend on one other attribute in this way are also called **derived** attributes, and they can be created by calling one
of the various `derive...` methods on the original attribute:
```cpp
AttributeBase::Ptr attr1 = AttributeStatic<Complex>::make(Complex(3, 4));
AttributeBase::Ptr attr2 = attr1->deriveMag();

Real read1 = **attr2; //read1 = 5
**attr1 = Complex(1, 0);
Real read2 = **attr2; //read2 = 1
```
There is also a general `derive`-method which can take a custom `getter` and `setter` lambda function for computing the derived attribute from its dependency.
For more complex cases involving dependencies on multiple attributes, the `AttributeDynamic` class has a method called `addTask` which can be used to add arbitrary computation tasks which are executed when the attribute is read or written to. For more information, check the method comments in `Attribute.h`.

## Using Attributes for Logging and Interfacing
When setting up a simulation, there are some methods which require an instance of `AttributeBase::Ptr` as a parameter. Examples for this
are the logger methods (e.g. `DataLogger::logAttribute`) and [interface]({{< ref "../interfaces.md" >}}) methods (e.g. `InterfaceVillas::exportAttribute`). To obtain the
required attribute pointer, one can either directly access the public member variables of the component the attribute belongs to, or use the component's `attribute(String name)` method which will look up the attribute in the component's `AttributeList`:

```cpp
auto r1 = DP::Ph1::Resistor::make("r_1");
r1->setParameters(5);

auto logger = DataLogger::make("simName");
// Access the attribute through the member variable
logger->logAttribute("i12", r1->mIntfCurrent);

auto intf = std::make_shared<InterfaceVillas>(config);
// Access the attribute through the AttributeList
intf->exportAttribute(r1->attribute('i_intf'), 0, true, true);

// Access the attribute through the member variable and use deriveCoeff to convert it to a scalar value
intf->exportAttribute(r1->mIntfVoltage->deriveCoeff<Complex>(0, 0), 0, true);

```

When creating a simulation in Python, the component's member variables are usually not accessible, so the `attr`-method has to be used for all accesses:
```python
# dpsim-mqtt.py
intf = dpsimpyvillas.InterfaceVillas(name='dpsim-mqtt', config=mqtt_config)
intf.import_attribute(evs.attr('V_ref'), 0, True)
intf.export_attribute(r12.attr('i_intf').derive_coeff(0, 0), 0)
```

## Using Attributes to Schedule Tasks

Attributes are also used to determine dependencies of tasks on data, which is information required by the scheduler.
For the usual `MNAPreStep` and `MNAPostStep` tasks, these dependencies are configured in the `mnaAddPreStepDependencies` and `mnaAddPostStepDependencies` methods:

```cpp
void DP::Ph1::Inductor::mnaAddPostStepDependencies(
    AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector
  ) {
    attributeDependencies.push_back(leftVector);
	modifiedAttributes.push_back(mIntfVoltage);
	modifiedAttributes.push_back(mIntfCurrent);
}
```
Here, the MNA post step depends on the solution vector of the system, `leftVector`, and modifies `mIntfVoltage` and `mIntfCurrent`.
Therefore, this task needs to be scheduled after the system solution that computes `leftVector` and before tasks that require the voltage and current interface vectors of the inductance, e.g. the task logging these values.
