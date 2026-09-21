---
title: "Install"
aliases: ["/docs/getting-started/install/"]
linkTitle: "Install"
date: 2026-07-31
description: >
  Installing DPsim and its Python package.
weight: 1
---

DPsim is a Python module and C++ library for dynamic power system simulation.

The quickest route to a result is the Python module: install it, then work through the
[tutorials]({{< ref "/docs/Tutorials" >}}), which build up one idea at a time from a source and a
resistor. If you would rather read a finished study than build one, the
[example notebooks]({{< ref "examples.md" >}}) run complete scenarios and plot them.

Building from source is only needed for a platform without a published wheel, or to work on DPsim
itself; see [build]({{< ref "/docs/Developer Guide/Architecture and Conventions/build.md" >}}).

## Try it without installing

The example notebooks run in the browser with no local installation:

[![Binder](https://2i2c.mybinder.org/badge_logo.svg)](https://2i2c.mybinder.org/v2/gh/sogno-platform/dpsim/HEAD?urlpath=%2Fdoc%2Ftree%2Fexamples%2FIndex.ipynb)

## Python package

DPsim is published on PyPI and installs like any other Python package:

```shell
python3 -m venv venv
source venv/bin/activate
pip install dpsim
```

On Windows the same three steps are:

```powershell
py -m venv venv
venv\Scripts\activate
pip install dpsim
```

{{% alert title="Published wheels cover Linux and Windows" color="info" %}}
Three limitations are worth knowing before you start.
Wheels are published for Linux and Windows on x86-64 only, so on macOS you have to build from
source for now.
The Windows wheel is a reduced build and does not carry every feature; see
[supported versions](#supported-versions) below.
The package also contains only the simulation core; the example notebooks additionally need
plotting and data handling packages, which are listed in the import section of each notebook.
{{% /alert %}}

## macOS

DPsim currently does not provide a pre-built macOS wheel. On macOS, install DPsim from source using the native setup script included in the repository.

The setup has been tested on Apple Silicon and uses the standard CMake build together with Homebrew-provided dependencies.

First make sure the Xcode Command Line Tools are installed:

```shell
xcode-select --install
```

Then clone DPsim and run the macOS setup script:

```shell
git clone https://github.com/sogno-platform/dpsim.git
cd dpsim

chmod +x scripts/install-macos.sh
./scripts/install-macos.sh
```

The setup script prepares the complete native development environment. It installs the required Homebrew dependencies, creates a local Python virtual environment called `dpsim-python`, configures and builds DPsim with CMake, installs the Python package, and registers a `DPsim Python` Jupyter kernel.

The native build includes support for:

* Apple Silicon (`arm64`) and Intel (`x86_64`) macOS
* CMake and Ninja
* Eigen 3
* SuiteSparse / KLU
* Graphviz
* OpenMP through Homebrew `libomp`
* Python bindings through pybind11
* C++ examples
* JupyterLab and the DPsim Python environment

After installation, activate the Python environment with:

```shell
source dpsim-python/bin/activate
```

The Python package can then be used normally:

```python
import dpsim
import dpsimpy
```

For C++ development, the build directory created by the setup script is already configured with the required macOS-specific CMake settings. Normal rebuilds therefore only require:

```shell
cmake --build build --parallel "$(sysctl -n hw.ncpu)"
```

The macOS-specific dependency paths and compiler settings are stored in the CMake build directory and do not have to be specified again for subsequent builds.

To recreate the complete local environment and build tree from scratch, run:

```shell
CLEAN=1 ./scripts/install-macos.sh
```

This removes the local DPsim build directory, Python environment and registered DPsim Jupyter kernel before recreating them.

The default macOS setup builds the DPsim simulation core, Python bindings, OpenMP support,
Graphviz support and C++ examples.

Note: CIM/CGMES support and VILLASnode integration are not enabled by the default macOS setup script
and require their respective native dependencies to be configured separately.

## Supported versions

DPsim needs CPython 3.10 or newer, both for the wheels and for a source build. One wheel is
published per platform and CPython version:

| Platform | Architecture | CPython | Modules in the wheel |
| --- | --- | --- | --- |
| Linux, `manylinux_2_28` | x86-64 | 3.10, 3.11, 3.12, 3.13, 3.14 | `dpsim`, `dpsimpy`, `dpsimpyvillas` |
| Windows | x86-64 | 3.10, 3.11, 3.12, 3.13, 3.14 | `dpsim`, `dpsimpy` |
| macOS | any | none | build from source |

Free-threaded interpreters (`cp3XXt`) and PyPy are not built, and neither are 32-bit or Arm
wheels. Older DPsim releases additionally ship CPython 3.9 wheels.

The two wheels are not equivalent. The Windows one is built without VILLASnode, real-time
support and Sundials, because those do not build there, so co-simulation, real-time execution
and the ODE-based generator are missing from it:

| Feature | Linux wheel | Windows wheel |
| --- | --- | --- |
| MNA, power flow and SSN solvers | yes | yes |
| CIM/CGMES reader | yes | yes |
| Component models | all | all but `SynchronGeneratorDQODE` in `dpsimpy.dp.ph3` and `dpsimpy.emt.ph3` |
| `dpsimpyvillas`, VILLASnode co-simulation | yes | no |
| `RealTimeSimulation`, `RealTimeDataLogger` | yes | no |
| MNA solver plugins | yes | no |

Do not build from source to close that gap; on Windows there are two easier routes to the full
package. Either run the Linux wheel inside
[WSL2](https://learn.microsoft.com/windows/wsl/install), where a plain `pip install dpsim` gives
you everything, or use the `sogno/dpsim` container described below, which ships the same
complete build.

The wheel pulls in NumPy 2.0 or newer, pandas 2.0 or newer and SciPy 1.10 or newer.

If you prefer conda, the equivalent is:

```shell
conda create -n dpsim python=3.13
conda activate dpsim
pip install dpsim
```

## Docker

You need [Docker](https://docs.docker.com/install/) installed first. The prepared image on
Docker Hub bundles the module together with a JupyterLab session:

```shell
docker run -p 8888:8888 sogno/dpsim
```

Then open <http://localhost:8888/lab?token=3adaa57df44cea75e60c0169e1b2a98ae8f7de130481b5bc>.

Note that the image pins that access token in its startup command, so it is the same for
everyone who runs the image. Publish the port on localhost only, as above, and do not expose
it to an untrusted network.

To build the image yourself rather than pulling it, see
[build]({{< ref "/docs/Developer Guide/Architecture and Conventions/build.md" >}}).
