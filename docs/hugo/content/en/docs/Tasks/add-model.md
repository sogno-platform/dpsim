---
title: "Add New Model"
linkTitle: "Add New Model"
weight: 6
date: 2020-03-17
description: >
  Extending the simulator with new component or control models.
---

# Add a Component Model

In this section we will show the implementation of a new component model by means a three-phase dynamic phasor inductor model.

## C++ OOP

DPsim implements component models in a sub project called CPowerSystems (CPS) that is located in the *models* folder.
This folder is added to the DPsim CMake project.
Every component in DPsim is represented by a C++ class.

DPsim supports different types of solvers (MNA, DAE, NRP).
Each solver requires certain member functions in the component class to be implemented.
These functions are specified by the solver interface classes: ``MNAInterface.h``, ``DAEInterface.h``, etc.

## Directory / Namespace Structure

For the implementation of the new component, we add two new files

- ``models/Source/DP/DP_Ph3_Inductor.cpp``
- ``models/Include/DP/DP_Ph3_Inductor.h``

In these files, we will implement a new C++ class with the name ``CPS::DP::Ph3::Inductor``.
The general structure looks as follows.

Directories:

```text
DPsim
 |
 |- Source
 |- Include
  \ models
      |- Source
          |- DP
          |- EMT
          |- Static
            \ Signal
      |- Include
          |- DP
          |- EMT
          |- Static
            \ Signal
```

Namespaces:

```cpp
CPS::{DP,EMT,Static,Signal}::{Ph1,Ph3}::{Name}
```

## Attributes

Each components has a list of attributes, which has to be specified when creating the components class.

TODO: explain attribute system

## Tasks for Pre/Post-step Functions

TODO: add example task dependency graph

## Adding the new Component to DPsim

After finishing the implementation of the new component, it needs to be added to the following files:

- `models/Include/cps/Components.h`
- `models/Source/CMakeLists.txt`
- `Sources/Python/Module.cpp`
