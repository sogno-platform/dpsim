# <img src="docs/images/dpsim.png" width=40 /> DPsim

[![Build & Test CentOS](https://github.com/sogno-platform/dpsim/actions/workflows/build_test_linux_centos.yaml/badge.svg)](https://github.com/sogno-platform/dpsim/actions/workflows/build_test_linux_centos.yaml)

[![Build & Test Fedora](https://github.com/sogno-platform/dpsim/actions/workflows/build_test_linux_fedora.yaml/badge.svg)](https://github.com/sogno-platform/dpsim/actions/workflows/build_test_linux_fedora.yaml)

[![Build & Test Fedora Minimal](https://github.com/sogno-platform/dpsim/actions/workflows/build_test_linux_fedora_minimal.yaml/badge.svg)](https://github.com/sogno-platform/dpsim/actions/workflows/build_test_linux_fedora_minimal.yaml)

[![Build & Test Windows](https://github.com/sogno-platform/dpsim/actions/workflows/build_test_windows.yaml/badge.svg)](https://github.com/sogno-platform/dpsim/actions/workflows/build_test_windows.yaml)

[![License: MPL 2.0](https://img.shields.io/badge/License-MPL%202.0-brightgreen.svg)](https://opensource.org/licenses/MPL-2.0)

DPsim is a solver library for dynamic power system simulation.

- It supports both the electromagnetic transient (EMT) and dynamic phasor (DP) domain for dynamic simulation.
- A powerflow solver is included for standalone usage or initialization of dynamic simulations.
- It provides a Python module which can be embedded in any Python 3 application / scripts.
- The simulation core is implemented in highly-efficient C++ code.
- It supports real-time execution with time-steps down to 50 uS.
- It can load models in the IEC61970 CIM / CGMES XML format.
- It can be interfaced to a variety of protocols and interfaces via [VILLASnode](https://fein-aachen.org/projects/villas-node/).

## Documentation

The [documentation](https://dpsim.fein-aachen.org/) has build / installation instructions, links to examples and explains the concepts implemented in DPsim as well as its architecture.

## License

The project is released under the terms of the [MPL 2.0](https://mozilla.org/MPL/2.0/).

## Contact

[LF Energy Slack](https://slack.lfenergy.org/) - Chat with other users and developers and get help in the **#sogno** or **#sogno-dpsim** channel.

You can also send a direct message to
- Markus Mirz
- Jan Dinkelbach
- Steffen Vogel
