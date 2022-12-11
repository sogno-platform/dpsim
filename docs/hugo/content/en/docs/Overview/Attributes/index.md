---
title: "Attributes"
linkTitle: "Attributes"
weight: 3
date: 2020-05-01
---

Attributes, instances of the `Attribute` class, serve multiple purposes in DPsim:
- add meta information about variables, like read and write access
- in combination with tasks, they are the input required by the scheduler
- connect components by pointing an attribute of one component to another component's member variable
- add additional getter / setter lambda functions to variables
- add reflexion, which is not supported by C++ directly

Let's have a look at some code snippets that use attributes.

Classes support attributes by inheriting from `AttributeList`:

    class IdentifiedObject: virtual public AttributeList {
      ...
    }

New attributes are usually registered in the constructor of a class:

    DP::Ph1::Inductor::Inductor(String uid, String name, Logger::Level logLevel)
      ...
	    addAttribute<Real>("L", &mInductance, Flags::read | Flags::write);
    }

Here, the attribute has the name `L` and points to the member variable `mInductance`.
Besides, the attribute enables reading and writing this variable.

To access an attribute, you need to know its name and the component it belongs to.
For example, adding the voltage at node `n1` to the logger for a simulation can be done as follows:

    logger->addAttribute("v1", n1->attributeMatrixComp("v"));

Note that the voltage is a complex matrix in this case.
Since we would like to use Eigen matrix specific methods in the logger, the `attributeMatrixComp` function is used to return the attribute already casted to a complex matrix type.

Attributes are also used to determine dependencies of tasks on data, which is information required by the scheduler.
A task describing its dependencies on attributes could look like this.

    class MnaPostStep : public Task {
        public:
          MnaPostStep(Inductor& inductor, Attribute<Matrix>::Ptr leftVector) :
            Task(inductor.mName + ".MnaPostStep"),
            mInductor(inductor), mLeftVector(leftVector) {
            mAttributeDependencies.push_back(mLeftVector);
            mModifiedAttributes.push_back(mInductor.attribute("v_intf"));
            mModifiedAttributes.push_back(mInductor.attribute("i_intf"));
          }
          void execute(Real time, Int timeStepCount);
        private:
          Inductor& mInductor;
          Attribute<Matrix>::Ptr mLeftVector;
        };

Here, the MNA post step depends on the solution vector of the system, `leftVector`, and modifies `v_intf` and `i_intf`.
Therefore, this task needs to be scheduled after the system solution that computes `leftVector` and before tasks that require the voltage and current interface vectors of the inductance, e.g. the task logging these values.