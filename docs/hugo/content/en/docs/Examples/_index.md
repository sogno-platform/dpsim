---
title: "Examples"
linkTitle: "Examples"
weight: 8
date: 2026-07-31
description: >
  Runnable examples in Python and C++.
---

The repository carries examples in both languages. The Python notebooks are the better starting
point, since they run a scenario and plot the result in one place. The C++ examples are the
better reference for using DPsim as a library, and are what the real time and co-simulation
scenarios are written in.

## Notebooks

Run them in the browser with no local installation:

[![Binder](https://2i2c.mybinder.org/badge_logo.svg)](https://2i2c.mybinder.org/v2/gh/sogno-platform/dpsim/HEAD?urlpath=%2Fdoc%2Ftree%2Fexamples%2FIndex.ipynb)

Locally, they live under
[examples/Notebooks](https://github.com/sogno-platform/dpsim/tree/master/examples/Notebooks) and
need the Python package on the path, as described in the
[build]({{< ref "/docs/Getting started/build.md" >}}) section.

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
[interfaces]({{< ref "/docs/Overview/interfaces.md" >}}) for how the coupling works and
[real time]({{< ref "/docs/Getting started/real-time.md" >}}) for running them against the wall
clock.
