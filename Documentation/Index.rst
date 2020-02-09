.. dpsim documentation master file, created by
   sphinx-quickstart on Fri Sep 22 16:22:46 2017.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

DPsim
======

DPsim is a solver library for power system simulation.

- It supports both the electro-magnetic transient (EMT) and dynamic phasor (DP) domain.
- It provides a Python module which can be embedded in any Python 3 application / scripts.
- It is implemented in highly-efficient C++ code.
- It supports real-time execution with time-steps up to 50 uS.
- It can load models in the `Common Information Model (CIM) <https://en.wikipedia.org/wiki/Common_Information_Model_%28electricity%29>`__ XML format.
- It can be interfaced to a variety of protocols and interfaces via `VILLASnode <http://www.fein-aachen.org/projects/villas-framework/>`__.


Installation
-------------

If you just want to use DPsim for simulations, take a look at the `installation instructions <https://dpsim.fein-aachen.org/doc/master/sphinx/Install.html>`__.

If you would like to modify / extend DPsim or build it for a specific platform, take a look at the `build instructions <https://dpsim.fein-aachen.org/doc/master/sphinx/Build.html>`__.

Usage
-------------
DPsim is a Python extension module which is implemented in optimized native C++ code. 
Users can control and script the simulation with Python scripts. Checkout the `Examples <https://dpsim.fein-aachen.org/doc/master/sphinx/Examples.html>`__.

Documentation
-------------
The `user documentation <http://dpsim.fein-aachen.org/doc/master/sphinx/>`__ has examples, build / installation instructions and covers the Python API.

The C++ `developer documentation <http://dpsim.fein-aachen.org/doc/master/doxygen/>`__ only includes automatically generated content using Doxygen.
It is helpful to understand the general structure of the C++ DPsim core components.


Contact
-------------

- `Markus Mirz <mmirz@eonerc.rwth-aachen.de>`__
- `Steffen Vogel <stvogel@eonerc.rwth-aachen.de>`__

.. toctree::
   :maxdepth: 2
   :hidden:

   Examples
   Install
   Build
   Real-time
   Reference
