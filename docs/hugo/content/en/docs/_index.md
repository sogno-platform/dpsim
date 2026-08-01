---
title: "Documentation"
linkTitle: "Documentation"
weight: 20
menu:
  main:
    weight: 20
description: >
  Documentation for DPsim, a solver library for dynamic power system simulation.
---

DPsim is a solver library for dynamic power system simulation.

- It supports both the electromagnetic transient (EMT) and dynamic phasor (DP) domain for dynamic simulation.
- A powerflow solver is included for standalone usage or to initialize dynamic simulations.
- It provides a Python module which can be embedded in any Python 3 application / scripts.
- The simulation core is implemented in highly-efficient C++ code.
- It supports real-time execution with time-steps down to 50 uS.
- It can load models in the IEC61970 Common Information Model (CIM) / Common Grid Model Exchange Standard (CGMES) XML format.
- It can be interfaced to a variety of protocols and interfaces via [VILLASnode](https://fein-aachen.org/projects/villas-node/).

## Connect

Using or want to use DPsim? Find out more here:

[GitHub Discussions](https://github.com/sogno-platform/dpsim/discussions) - Ask questions, share ideas, and get community support. This is our main point of contact.

[LF Energy Slack](https://slack.lfenergy.org/) - Chat with other users and developers and get help in the **#sogno** or **#sogno-dpsim** channel.

For email inquiries, please contact the Institute for Automation of Complex Power Systems (ACS), which coordinates DPsim development: [post_acs@eonerc.rwth-aachen.de](mailto:post_acs@eonerc.rwth-aachen.de).

## How to contribute

If you want to get more involved with DPsim, we welcome contributions of all kinds, including code, documentation, examples, models, bug reports, feature requests, and reviews.

Please open a Pull Request or issue on [GitHub](https://github.com/sogno-platform/dpsim), or start a discussion there to propose ideas and get feedback from the community.

### Contributors

See [CONTRIBUTORS.md](https://github.com/sogno-platform/dpsim/blob/master/CONTRIBUTORS.md) for a list of contributors.

## Publications

If you use DPsim in your research, please cite it. See
[how to cite]({{< ref "/docs/Citation.md" >}}) for the software paper, a ready-made BibTeX
entry, and the papers covering specific methods.
