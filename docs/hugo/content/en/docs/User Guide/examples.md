---
title: "Examples"
linkTitle: "Examples"
weight: 3
description: >
  Runnable notebooks and C++ examples.
aliases: ["/docs/examples/"]
---

If you are starting out, start with the [tutorials]({{< ref "/docs/Tutorials" >}}) instead. They
work through one idea at a time in order, each as a complete script. This page is the inventory of
what else the repository carries, which is the right thing once you know what you are looking for.

The examples come in both languages. The Python notebooks run a scenario and plot the result in one
place, so they are the better way to see a complete study. The C++ examples are the better reference
for using DPsim as a library, and are what the real time and co-simulation scenarios are written in.

## Where each kind fits

| If you want to | Go to |
| --- | --- |
| Learn how a simulation is put together | [Tutorials]({{< ref "/docs/Tutorials" >}}) |
| See a complete study with plots | Notebooks, below |
| Use DPsim from an application | C++ examples, below |
| Look up what a model does | [Concepts]({{< ref "/docs/Concepts" >}}) |

## Notebooks

Run them in the browser with no local installation:

[![Binder](https://2i2c.mybinder.org/badge_logo.svg)](https://2i2c.mybinder.org/v2/gh/sogno-platform/dpsim/HEAD?urlpath=%2Fdoc%2Ftree%2Fexamples%2FIndex.ipynb)

Locally, they live under
[examples/Notebooks](https://github.com/sogno-platform/dpsim/tree/master/examples/Notebooks) and
need the Python package on the path, as described in the
[build]({{< ref "/docs/Developer Guide/Architecture and Conventions/build.md" >}}) section.

| Category | Contents |
| --- | --- |
| Quickstart Guide | A single notebook covering a first simulation end to end |
| Circuits | Small networks exercising one modelling aspect at a time |
| Components | One component at a time, often comparing domains against each other |
| Grids | Published test systems such as the WSCC 9 bus and CIGRE networks |
| Features | Cross-cutting capabilities rather than a specific network |
| Performance | Timing and scaling comparisons |
| StateSpace | State-space extraction from a running simulation |

`Understanding_DP.ipynb` is worth reading early if dynamic phasors are new to you, since it
builds the intuition the [concepts]({{< ref "/docs/Concepts" >}}) section then formalises.

### Continuing from a tutorial

Each tutorial ends at the point where the notebooks take over, so a category is usually the natural
next step from the rung that introduced the idea.

| After this tutorial | These notebooks go further |
| --- | --- |
| [Adding dynamics]({{< ref "/docs/Tutorials/Python/adding-dynamics" >}}) | Circuits, for larger passive networks |
| [Two-bus network]({{< ref "/docs/Tutorials/Python/two-bus-network" >}}) | Grids, for published test systems solved the same way |
| [Comparing domains]({{< ref "/docs/Tutorials/Python/comparing-domains" >}}) | Components, which compare one model across domains |
| [Adding a machine]({{< ref "/docs/Tutorials/Python/a-machine" >}}) | Circuits, for the SMIB and multi-machine fault studies |
| [Exchanging data]({{< ref "/docs/Tutorials/Python/exchanging-data" >}}) | The co-simulation examples below |

## C++ examples

Under [dpsim/examples/cxx](https://github.com/sogno-platform/dpsim/tree/master/dpsim/examples/cxx),
built as part of a normal build and produced as executables in the build directory.

| Directory | Contents |
| --- | --- |
| Circuits | Networks assembled directly in C++ |
| Components | Single component scenarios |
| CIM | Reading network data from CIM and CGMES files |
| StateSpace | State-space extraction |
| RealTime | Scenarios run against the wall clock |
| DAE, signals, timer | Smaller scenarios for the DAE solver, signal models and timing |

The target name is the source file name without its extension, regardless of which directory the
source sits in, and the executables are written flat into the build tree. So to build and run a
single example from your build directory:

```shell
cmake --build . --target DP_VS_RL1
./dpsim/examples/cxx/DP_VS_RL1
```

## Co-simulation

The [dpsim-villas](https://github.com/sogno-platform/dpsim/tree/master/dpsim-villas/examples/cxx)
examples exchange data with other simulators or with hardware through VILLASnode. These need
`WITH_VILLAS` enabled and are therefore not available on Windows. See
[interfaces]({{< ref "/docs/User Guide/co-simulation.md" >}}) for how the coupling works and
[real time]({{< ref "/docs/User Guide/real-time.md" >}}) for running them against the wall
clock.
