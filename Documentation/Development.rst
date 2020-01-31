Development
===========


Environment
-----------

At the Institute for Automation of Complex Power Systems (ACS) we recommend the following development tools:

- Editor: [Visual Studio Code](https://code.visualstudio.com)
  - Extensions:
    - [Install](vscode:extension/ms-vscode-remote.remote-containers)
    - [C/C++](https://marketplace.visualstudio.com/items?itemName=ms-vscode.cpptools)
    - [Python](https://marketplace.visualstudio.com/items?itemName=ms-python.python)
    - [CMake Tools](https://marketplace.visualstudio.com/items?itemName=vector-of-bool.cmake-tools)
    - [Docker](https://marketplace.visualstudio.com/items?itemName=ms-azuretools.vscode-docker)
    - [EditorConfig for VS Code](https://marketplace.visualstudio.com/items?itemName=EditorConfig.EditorConfig)
    - [Remote - SSH](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-ssh)
    - [Remote - Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)
- [Docker](https://www.docker.com)
- [CMake](https://cmake.org)
- [Git](https://git-scm.com)
  - Issues: http://git.rwth-aachen.de/acs/public/simulation/dpsim/issues
  - Merge Requests: http://git.rwth-aachen.de/acs/public/simulation/dpsim/merge_requests
  - Continous Integration and Testing: http://git.rwth-aachen.de/acs/public/simulation/dpsim/pipelines

Please follow the [Build](./Build.rst) guide to checkout your code and install the basic dependencies and tools.

Add a new Component Model
-------------------------

In this section we will show the implementation of a new component model at the example of a three-phase inductor.

C++ OOP for Component Models
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

DPsim implements component models in a sub-component called CPowerSystems or short CPS.
CPS is embedded as a submodule to the DPsim repository.

TODO: add class diagram

Every component in CPS is represented by a C++ class.

DPsim supports different types of solvers (MNA, DAE, NRP).
Each solver requires certain member functions in the component class to be implemented.
These functions are specified by the solver interface classes: ``MNAInterface.h``, ``DAEInterface.h``, ...

Directory / Namespace Structure
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

For the implementation of the new component, we add two new files

- ``Dependencies/libcps/Source/DP/DP_Ph3_Inductor.cpp``
- ``Dependencies/libcps/Include/DP/DP_Ph3_Inductor.h``

In these files, we will implement a new C++ class with the name ``CPS::DP::Ph3::Inductor``.

Component Types
"""""""""""""""

Directories:

::

    DPsim
     |
     |- Source
     |- Include
      \ Dependencies
         \ libcps
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


Namespaces:

::

    CPS::{DP,EMT,Signal,Static}::{Ph1,Ph3}::{Name}

Attributes
~~~~~~~~~~

Tasks for Pre/Post-step Functions
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

TODO: add example task dependency graph


MNA Matrix Stamping
~~~~~~~~~~~~~~~~~~~

https://en.wikipedia.org/wiki/Modified_nodal_analysis

Adding the new Component to DPsim
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

After finishing the implementation of the new component, it needs to be added to the following files:

- ``Dependencies/libcps/Include/cps/Components.h``
- ``Dependencies/libcps/Source/CMakeLists.txt``
- ``Sources/Python/Module.cpp``