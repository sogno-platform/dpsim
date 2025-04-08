---
title: "Interfacing with the MNA Solver"
linkTitle: "Interfacing with the MNA Solver"
date: 2022-12-19
---

The various solver classes based on `MNASolver` are used to perform [Nodal Analysis]({{< ref "../Concepts/nodal-analysis.md" >}}) during a DPsim simulation. For components to be able to influence the input variables of the MNA, they have to implement certain methods defined in the `MNAInterface` interface class. While it is possible to individually implement `MNAInterface` for every
component, the behavior of many components can be unified in a common base class. This base class is called `MNASimPowerComp<T>`.
Currently, it is the only class which directly implements `MNAInterface` and in turn all MNA components inherit from this class.
Much like the `CompositePowerComp` class for [Composite Components]({{< ref "./subcomponents.md" >}}), the `MNASimPowerComp` class
provides some common behavior for all MNA components, e.g. the creation and registration of the `MNAPreStep` and `MNAPostStep` tasks.
Additionally, `MNASimPowerComp` provides a set of virtual methods prefixed `mnaComp...` which can be implemented by the child component classes to provide their own MNA behavior. These methods are:

```cpp
virtual void mnaCompInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector);
virtual void mnaCompApplySystemMatrixStamp(SparseMatrixRow& systemMatrix);
virtual void mnaCompApplyRightSideVectorStamp(Matrix& rightVector);
virtual void mnaCompUpdateVoltage(const Matrix& leftVector);
virtual void mnaCompUpdateCurrent(const Matrix& leftVector);
virtual void mnaCompPreStep(Real time, Int timeStepCount);
virtual void mnaCompPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector);
virtual void mnaCompAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes);
virtual void mnaCompAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector);
virtual void mnaCompInitializeHarm(Real omega, Real timeStep, std::vector<Attribute<Matrix>::Ptr> leftVector);
virtual void mnaCompApplySystemMatrixStampHarm(SparseMatrixRow& systemMatrix, Int freqIdx);
virtual void mnaCompApplyRightSideVectorStampHarm(Matrix& sourceVector);
virtual void mnaCompApplyRightSideVectorStampHarm(Matrix& sourceVector, Int freqIdx);
```

`MNASimPowerComp` provides empty default implementations for all of these methods, so component classes are not forced to implement any of them.

# Controlling Common Base Class Behavior

Child component classes can control the behavior of the base class through the constructor arguments of `MNASimPowerComp`.
The two boolean variables `hasPreStep` and `hasPostStep` can be used to control whether the `MNAPreStep` and `MNAPostStep` tasks will be created and registered.
If these tasks are created, the `mnaCompPreStep` / `mnaCompPostStep` and `mnaCompAddPreStepDependencies` / `mnaCompAddPostStepDependencies` methods will be called during the component's lifecycle.
If the tasks are not created, these methods are superfluous and should not be implemented in the child class.

Currently, the `MNASimPowerComp` base class only exhibits additional behavior over the `mnaComp...` methods in the `mnaInitialize` method. In this method, the list of MNA tasks is cleared, and the new tasks are added according to the `hasPreStep` and `hasPostStep` parameters. Additionally, the right vector attribute `mRightVector` required by `MNAInterface` is set to a zero-vector with its length equal to that of the system `leftVector`.
If this behavior is not desired, e.g. for resistors which have no influence on the system right vector, the right vector can be re-set to have zero size in the `mnaCompInitialize` method:

```cpp
void DP::Ph1::Resistor::mnaCompInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
  updateMatrixNodeIndices();

  **mRightVector = Matrix::Zero(0, 0);
  //...
}
```

For all other MNA methods, the `MNASimPowerComp` base class will just call the associated `mnaComp...` method. For more details, take a look at the implementations in `MNASimPowerComp.cpp`.
