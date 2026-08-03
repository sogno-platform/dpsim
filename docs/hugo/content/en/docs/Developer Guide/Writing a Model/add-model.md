---
title: "Add New Model"
aliases: ["/docs/tasks/add-model/"]
linkTitle: "Add New Model"
weight: 10
date: 2026-07-31
description: >
  Extending the simulator with new component or control models.
---

This page walks through adding a component model, using a three phase dynamic phasor inductor as
the example.

## Where the code lives

Component models live in the `dpsim-models` subproject, which builds the `CPS` library. Headers
and sources are separate trees, both organised by domain:

```text
dpsim-models
 |- include
 |   \ dpsim-models
 |       |- Base            shared base classes, one per component family
 |       |- DP              dynamic phasor
 |       |- EMT             electromagnetic transient
 |       |- SP              static phasor
 |       \ Signal           domain independent control and signal models
 \- src
     |- Base
     |- DP
     |- EMT
     |- SP
     \ Signal
```

Namespaces follow the same shape, with the phase count nested inside the domain:

```cpp
CPS::{DP,EMT,SP}::{Ph1,Ph3}::{Name}
CPS::Signal::{Name}
```

File names encode the same information, so the example model needs two files:

- `dpsim-models/include/dpsim-models/DP/DP_Ph3_Inductor.h`
- `dpsim-models/src/DP/DP_Ph3_Inductor.cpp`

declaring the class `CPS::DP::Ph3::Inductor`.

## Choosing a base class

DPsim supports several solvers, and each requires certain member functions on the component.
Which ones you implement is determined by the interfaces you inherit rather than by the solver
itself.

For an MNA component, derive from `MNASimPowerComp<VarType>`, with `Complex` as the variable
type in the `DP` and `SP` domains and `Real` in `EMT`. The MNA hooks are declared on
`MNAInterface`, which `MNASimPowerComp` implements, so that is where to look for the full set.

If the model is naturally expressed as several existing components wired together rather than as
a single stamp, derive from `CompositePowerComp` and add subcomponents instead. The pi-line is a
worked example. See [subcomponents]({{< ref "/docs/Developer Guide/Writing a Model/subcomponents.md" >}}).

If the component is better described by its own state-space model coupled to the network, see
[state-space nodal]({{< ref "/docs/Concepts/state-space-nodal.md" >}}) for that alternative.

## Attributes

Every component exposes its parameters and state through attributes, declared in the class and
registered in the constructor. Attributes are what make a value visible to the logger, to the
Python bindings and to the task scheduler.

How to declare, read and derive attributes is described under
[attributes]({{< ref "/docs/Developer Guide/Attributes and Scheduling/Attributes/index.md" >}}) and
[attribute usage]({{< ref "/docs/Developer Guide/Attributes and Scheduling/attribute-usage.md" >}}).

## Tasks and step functions

Pre-step and post-step functions are registered as tasks, and the scheduler derives the order in
which they may run from the attributes each task reads and writes. Declaring those dependencies
correctly matters: a task that modifies an attribute without declaring it can be scheduled in
the wrong order, or in parallel with a reader.

How tasks are built and how the dependency graph is derived is described under
[scheduling]({{< ref "/docs/Developer Guide/Attributes and Scheduling/Scheduling/index.md" >}}).

## Registering the new component

A new component is not picked up automatically. Three places have to be updated:

- `dpsim-models/src/CMakeLists.txt`, adding the new source file to the list, for example
  `DP/DP_Ph3_Inductor.cpp`
- `dpsim-models/include/dpsim-models/Components.h`, adding the header so that including that one
  file gives access to every component
- `dpsim/src/pybind/DPComponents.cpp`, or the matching `EMTComponents.cpp`, `SPComponents.cpp`
  or `SignalComponents.cpp`, to expose the class to Python

The Python binding follows the existing pattern in those files:

```cpp
py::class_<CPS::DP::Ph1::Resistor, std::shared_ptr<CPS::DP::Ph1::Resistor>,
           CPS::SimPowerComp<CPS::Complex>>(mDPPh1, "Resistor", py::multiple_inheritance())
    .def("set_parameters", &CPS::DP::Ph1::Resistor::setParameters, "R"_a);
```

Name the arguments using the `"R"_a` form shown above. Without it the Python signature and the
generated reference fall back to positional placeholders such as `arg0`, and callers cannot use
keyword arguments.

## Initialization

Components are initialized either from power flow results or from explicitly set initial values,
and the solver calls into the component in a defined order. Do not add an `initialize(Real)`
overload of your own for user-facing initialization; use the documented hooks instead.

See [initialization]({{< ref "/docs/Developer Guide/Writing a Model/initialization.md" >}}) for the full sequence, and
note the scaling conventions in [guidelines]({{< ref "/docs/Developer Guide/Architecture and Conventions/conventions.md" >}}),
since initialization quantities are RMS3PH while EMT simulation quantities are PEAK1PH.
