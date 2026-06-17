---
title: "Attribute Usage Guidelines"
linkTitle: "Attribute Usage"
---

# Attribute Usage Guidelines

This page gives practical rules for deciding when a model variable should be a DPsim attribute. For details on the attribute mechanism itself, see [Attributes](https://dpsim.fein-aachen.org/docs/overview/attributes/).

## Rule of Thumb

Use an attribute if the value must be visible to DPsim infrastructure, for example logging, interfaces, Python access, string-based lookup, or scheduling.

Otherwise, prefer a normal C++ member variable or a local variable.

## Quick Decision Checklist

Before adding a new attribute, ask:

1. Does it need to be logged?
1. Does it need to be imported or exported?
1. Does it need to be accessed from Python or by name?
1. Is it used as a scheduler dependency?
1. Is it an externally relevant model input, output, state, or setpoint?
1. Is a normal C++ variable insufficient?

If the answer to all questions is no, do not make it an attribute.

## Use an Attribute For

Use an attribute if the value:

* should be logged
* should be imported or exported through an interface
* should be accessed from Python or generic code by name
* is read or modified by scheduled tasks
* is an externally relevant model input, output, state, or setpoint
* is a derived view of another attribute, for example one matrix coefficient

Typical examples are interface voltages and currents, source references, controller setpoints, and values exchanged through VILLASnode.

## Do Not Use an Attribute For

Prefer a normal C++ variable if the value:

* is only used inside one method
* is a temporary intermediate result
* is a cached coefficient or solver helper
* is a fixed implementation detail
* duplicates another existing attribute
* never needs logging, interface access, Python access, or scheduling

Do not create attributes for every variable in the model equations.

## Choose the Simplest Attribute Type

If decided that a value should be an attribute, choose the simplest suitable attribute type.

Prefer a static attribute when the value is stored directly by the component:

```cpp
const Attribute<Real>::Ptr mPower;

MyComponent::MyComponent(const String& name)
    : IdentifiedObject(name),
      mPower(mAttributes->create<Real>("P", 0.0)) {}
```

Use dynamic, referenced, or derived attributes only when the value must depend on another attribute.

For example, use a derived attribute when exporting one coefficient of a matrix or vector attribute:

```cpp
intf->exportAttribute(component->mIntfCurrent->deriveCoeff<Complex>(0, 0), 0, true);
```

Avoid long chains of dynamic, referenced, or derived attributes unless they are really needed.

## Access Attributes in C++

In model code, prefer typed attribute members:

```cpp
mPower->set(power);
const Real power = **mPower;
```

Use string-based lookup mainly in generic code, logging, interfaces, tests, or Python-style access:

```cpp
logger->logAttribute("P", component->mPower);
intf->exportAttribute(component->attribute("P"), 0, true);
```

When assigning a new value, prefer `set()` if update tasks should be triggered or if the assignment should be explicit. Direct mutable access through `**attribute` can be used for simple static attributes, but it does not express this intent as clearly.

## Examples

### Private member variable: no need to use an attribute

This value is stored as a member because it is used by several functions of the class. It is still internal to the implementation: it does not need to be logged, imported or exported, used by the scheduler, or accessed by name.

```cpp
class MyComponent : public IdentifiedObject {
private:
  Real mConductance = 0.0; // used internally by several methods
};
```

### Externally visible output: use an attribute

This value is a model output. It may be useful for logging, plotting, interfaces, tests, or Python access, so it should be registered as an attribute.

```cpp
class MyComponent : public IdentifiedObject {
public:
  const Attribute<Real>::Ptr mPower;

  explicit MyComponent(const String& name)
      : IdentifiedObject(name),
        mPower(mAttributes->create<Real>("P", 0.0)) {}

  void updatePower(Real power) {
    mPower->set(power);
  }
};
```

### Runtime setpoint: use an attribute

This value is a model input or setpoint. It may be changed from outside the component, for example through Python, an interface, or a test setup.

```cpp
class MySource : public IdentifiedObject {
public:
  const Attribute<Real>::Ptr mVoltageRef;

  explicit MySource(const String& name)
      : IdentifiedObject(name),
        mVoltageRef(mAttributes->create<Real>("V_ref", 0.0)) {}

  void setParameters(Real voltageRef) {
    mVoltageRef->set(voltageRef);
  }
};
```

### Derived scalar value: use a derived attribute

This value is not stored separately. It is a scalar view of an existing vector or matrix attribute, which avoids duplicating data manually.

```cpp
intf->exportAttribute(component->mIntfCurrent->deriveCoeff<Complex>(0, 0), 0, true);
```
