---
title: "Documentation"
linkTitle: "Documentation"
weight: 20
menu:
  main:
    weight: 20
---

DPsim is a solver library for dynamic power system simulation.

- It supports both the electromagnetic transient (EMT) and dynamic phasor (DP) domain for dynamic simulation.
- A powerflow solver is included for standalone usage or to initialize dynamic simulations.
- It provides a Python module which can be embedded in any Python 3 application / scripts.
- The simulation core is implemented in highly-efficient C++ code.
- It supports real-time execution with time-steps down to 50 uS.
- It can load models in the IEC61970 Common Information Model (CIM) / Common Grid Model Exchange Standard (CGMES) XML format.
- It can be interfaced to a variety of protocols and interfaces via [VILLASnode](https://fein-aachen.org/projects/villas-node/).

# Connect

Using or want to use DPsim? Find out more here:

[GitHub Discussions](https://github.com/sogno-platform/dpsim/discussions) - Ask questions, share ideas, and get community support. This is our main point of contact.

[LF Energy Slack](https://slack.lfenergy.org/) - Chat with other users and developers and get help in the **#sogno** or **#sogno-dpsim** channel.

For email inquiries, please contact the Institute for Automation of Complex Power Systems (ACS), which coordinates DPsim development: [post_acs@eonerc.rwth-aachen.de](mailto:post_acs@eonerc.rwth-aachen.de).

# How to contribute

If you want to get more involved with DPsim, we welcome contributions of all kinds, including code, documentation, examples, models, bug reports, feature requests, and reviews.

Please open a Pull Request or issue on [GitHub](https://github.com/sogno-platform/dpsim), or start a discussion there to propose ideas and get feedback from the community.

## Contributors

See [CONTRIBUTORS.md](https://github.com/sogno-platform/dpsim/blob/master/CONTRIBUTORS.md) for a list of contributors.

# Publications

If you are using DPsim for your research, please cite one of the following papers in your publications:

- M. Mirz, S. Vogel, G. Reinke, A. Monti, "[**DPsim — A dynamic phasor real-time simulator for power systems**](https://www.sciencedirect.com/science/article/pii/S2352711018302760)," _SoftwareX_, Volume 10, July–December 2019, 100253.
- J. Dinkelbach, M. Moraga, and A. Monti, “[Reduced-Order Synchronous Generator Modelling for Real-Time Simulation using Shifted Frequency Analysis](https://ieeexplore.ieee.org/document/10089718),” in _2023 Open Source Modelling and Simulation of Energy Systems (OSMSES)_, 2023.
- J. Dinkelbach, L. Razik, M. Mirz, A. Benigni, and A. Monti, “[Template-based generation of programming language specific code for smart grid modelling compliant with CIM and CGMES](https://onlinelibrary.wiley.com/doi/abs/10.1049/tje2.12208),” _The Journal of Engineering_, 2022.
- G. Nakti, J. Dinkelbach, M. Mirz, and A. Monti, “[Comparative Assessment of Shifted Frequency Modeling in Transient Stability Analysis using the Open Source Simulator DPsim](https://ieeexplore.ieee.org/document/9769135),” in _2022 Open Source Modelling and Simulation of Energy Systems (OSMSES)_, 2022.
- J. Dinkelbach, L. Schumacher, L. Razik, A. Benigni, and A. Monti, “[Factorisation Path Based Refactorisation for High-Performance LU Decomposition in Real-Time Power System Simulation](https://www.mdpi.com/1996-1073/14/23/7989),” _Energies_, 2021.
- J. Dinkelbach, G. Nakti, M. Mirz, A. Monti, "[Simulation of Low Inertia Power Systems Based on Shifted Frequency Analysis](https://www.mdpi.com/1996-1073/14/7/1860)," _Energies_, 2021.
- M. Mirz, A. Monti, and A. Benigni, "[A Dynamic Phasor Real-Time Simulation Based Digital Twin for Power Systems](https://publications.rwth-aachen.de/record/804608/files/804608.pdf)," E.ON Energy Research Center, RWTH Aachen University, 2020.
- M. Mirz, J. Dinkelbach, A. Monti, "[DPsim — Advancements in Power Electronics Modelling Using Shifted Frequency Analysis and in Real-Time Simulation Capability by Parallelization](https://www.mdpi.com/1996-1073/13/15/3879)," _Energies_, 2020.
- M. Mirz, A. Estebsari, F. Arrigo, E. Bompard and A. Monti, "[Dynamic phasors to enable distributed real-time simulation](http://ieeexplore.ieee.org/document/8004805/)," _2017 6th International Conference on Clean Electrical Power (ICCEP)_, Santa Margherita Ligure, 2017.
- S. Vogel, M. Mirz, L. Razik, A. Monti, "[An Open Solution for Next-generation Real-time Power System Simulation](https://ieeexplore.ieee.org/document/8245739)," _1st IEEE Conference on Energy Internet and Energy System Integration (IEEE-EI^2)_, Beijing, 2017.
- M. Mirz, A. Monti, A. Estebsari, F. Arrigo, E. Bompard, "[Functionality of the releases of the real time solver V1](http://re-serve.eu/files/reserve/Content/Deliverables/D4.2.pdf)," _RESERVE Library_, 2017.
- M. Mirz, S. Vogel, A. Monti, "[First Interconnection test of the nodes in pan-European simulation platform](http://re-serve.eu/files/reserve/Content/Deliverables/D4.4.pdf)," _RESERVE Library_, 2017.
