---
title: "Overview"
linkTitle: "Overview"
weight: 1
---

DPsim is a real-time capable power system simulator that supports dynamic phasor and electromagnetic transient simulation as well as continuous powerflow. It primarily targets large-scale scenarios on commercial off-the-sheld hardware that require deterministic time steps in the range of micro- to milliseconds.

DPsim supports the CIM format as native input for the description of electrical network topologies, component parameters and load flow data, which is used for initialization. For this purpose, CIM++ is integrated in DPsim.
Users interact with the C++ simulation kernel via Python bindings, which can be used to script the execution, schedule events, change parameters and retrieve results. Supported by the availability of existing Python frameworks like Numpy, Pandas and Matplotlib, Python scripts have been proven as an easy and flexible way to codify the complete workflow of a simulation from modelling to analysis and plotting, for example in Jupyter notebooks.

The DPsim simulation kernel is implemented in C++ and uses the Eigen linear algebra library. By using a system programming language like C++ and a highly optimized math library, optimal performance and real-time execution can be guaranteed.
The integration into the [VILLASframework](https://github.com/VILLASframework/node) allows DPsim to be used in large-scale co-simulations.

## Licensing

The project is released under the terms of the [MPL 2.0](https://mozilla.org/MPL/2.0/).

## Where should I go next?

* [Getting Started]({{< ref "/docs/Getting Started/" >}} "Getting Started"): Get started with DPsim
* [Examples]({{< ref "/docs/Examples/" >}} "Examples"): Check out some example code!

