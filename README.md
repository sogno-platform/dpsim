# <img src="docs/images/dpsim.png" width=40 /> DPsim

[![pipeline status](https://git.rwth-aachen.de/acs/public/simulation/dpsim/dpsim/badges/master/pipeline.svg)](https://git.rwth-aachen.de/acs/public/simulation/dpsim/dpsim/commits/master)
[![coverage report](https://git.rwth-aachen.de/acs/public/simulation/dpsim/dpsim/badges/master/coverage.svg)](https://git.rwth-aachen.de/acs/public/simulation/dpsim/dpsim/commits/master)

DPsim is a solver library for dynamic power system simulation.

- It supports both the electromagnetic transient (EMT) and dynamic phasor (DP) domain for dynamic simulation.
- A powerflow solver is included standalone usage or initialization of dynamic simulations.
- It provides a Python module which can be embedded in any Python 3 application / scripts.
- The simulation core is implemented in highly-efficient C++ code.
- It supports real-time execution with time-steps down to 50 uS.
- It can load models in the IEC61970 Common Information Model (CIM) XML format.
- It can be interfaced to a variety of protocols and interfaces via [VILLASnode](https://fein-aachen.org/projects/villas-node/).

## Documentation

The [documentation](https://dpsim.fein-aachen.org/) has build / installation instructions, links to examples and explains the concepts implemented in DPsim as well as its architecture.
The DPsim [documentation source](https://github.com/dpsim-simulator/docs) repository is maintained on github.

## License

Most of the DPsim source files are dual-licensed under the [MPL2](https://mozilla.org/MPL/2.0/) and [GPL3](http://www.gnu.org/licenses/).
The project is released under the terms of the GPL3 due to its dependency on VILLASnode.

For other licensing options, please consult [Prof. Antonello Monti](mailto:amonti@eonerc.rwth-aachen.de).

## Contact

- Markus Mirz <mmirz@eonerc.rwth-aachen.de>
- Steffen Vogel <stvogel@eonerc.rwth-aachen.de>
- Jan Dinkelbach <jdinkelbach@eonerc.rwth-aachen.de>

[Institute for Automation of Complex Power Systems (ACS)](http://www.acs.eonerc.rwth-aachen.de) \
[EON Energy Research Center (EONERC)](http://www.eonerc.rwth-aachen.de) \
[RWTH University Aachen, Germany](http://www.rwth-aachen.de)

