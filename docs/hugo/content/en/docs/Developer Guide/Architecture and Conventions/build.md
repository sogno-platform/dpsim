---
title: "Build"
aliases: ["/docs/getting-started/build/"]
linkTitle: "Build"
date: 2026-07-31
description: >
  Building DPsim from source, with and without the optional features.
weight: 2
---

All builds start from a checkout of the repository. To build and read the code, cloning over
HTTPS needs no account:

```shell
git clone https://github.com/sogno-platform/dpsim.git
cd dpsim
```

If you intend to contribute, clone your own fork over SSH instead, since contributions are
accepted from forks only and pushing needs an authenticated remote:

```shell
git clone git@github.com:<your-user>/dpsim.git
cd dpsim
git remote add upstream https://github.com/sogno-platform/dpsim.git
```

The container route below is the most reproducible, because the image already carries every
dependency at the version CI uses. The native Linux and Windows routes need their dependencies
installed separately. On macOS, the repository provides a setup script that installs the required
dependencies and configures the native development environment.

## Container based

The commands below use `docker`, but the images are ordinary OCI images, so `podman` works as a
drop-in replacement throughout. On Fedora and Rocky, `podman` is usually the one already
installed. Substitute `podman` for `docker` in every command if you prefer it.

The repository ships a development image with all required dependencies:

```shell
docker build -t sogno/dpsim:dev -f packaging/Docker/Dockerfile.dev .
```

Alternatively, pull the prebuilt image instead of building it:

```shell
docker pull sogno/dpsim:dev
```

Then start an interactive session with the working copy mounted into the container:

```shell
docker run -it -p 8888:8888 -v $(pwd):/dpsim --privileged sogno/dpsim:dev bash
```

The `-p` option maps port 8888 so a JupyterLab instance inside the container is reachable from
the host. The `--privileged` option is required for debug builds. On Windows, the current
directory is spelled differently:

```shell
docker run -it -p 8888:8888 -v ${pwd}:/dpsim --privileged sogno/dpsim:dev bash
```

Inside the container, the C++ and Python libraries build as follows:

```shell
cd /dpsim
mkdir build && cd build
cmake ..
cmake --build . --target dpsimpy
```

Targets that are not built by default have to be named explicitly, for example:

```shell
cmake --build . --target dpsimpy dpsimpyvillas
```

To build everything:

```shell
cmake --build .
```

Optional features are enabled through the CMake options defined in the CMakeLists.txt files,
for example:

```shell
cmake .. -DWITH_GSL=ON
```

To use the freshly built Python package without installing it, put both the compiled extension
and the pure Python package on the path:

```shell
cd /dpsim/build
export PYTHONPATH=$(pwd):$(pwd)/../python/src
```

This is the setup most contributors work with, since it picks up a rebuild immediately without
any reinstall step.

Do not use `pip install -e .` for this. An editable install only links the pure Python sources;
`dpsimpy` is a compiled extension, so edits to the C++ are not picked up and you keep running
whatever binary was built at install time. The failure is silent, since the import still
succeeds and simply gives you stale behaviour. Either rebuild and rely on `PYTHONPATH` as above,
or reinstall the package after every C++ change.

To summarise the three ways to get DPsim, in increasing order of involvement: `pip install dpsim`
for a released Linux or Windows wheel, a native build plus `PYTHONPATH` for development, and
`make install` to place a build system wide.

If you develop inside a conda environment, the equivalent is to register the same two
directories from within the active environment. This needs `conda-build` installed:

```shell
cd /dpsim/build
conda develop $(pwd) && conda develop $(pwd)/../python/src
```

Note that this writes into the environment, so it becomes specific to your setup.

To run JupyterLab against it:

```shell
cd /dpsim
jupyter lab --ip="0.0.0.0" --allow-root --no-browser
```

To install DPsim system wide instead:

```shell
cd /dpsim/build
sudo make install
```

## CMake for Linux

The authoritative dependency list is whatever the Dockerfiles install, since that is what CI
builds against. See `packaging/Docker/Dockerfile.dev` for the Fedora set, and
[install-fedora-deps.sh](https://github.com/sogno-platform/dpsim/blob/master/packaging/Shell/install-fedora-deps.sh)
or [install-ubuntu-deps.sh](https://github.com/sogno-platform/dpsim/blob/master/packaging/Shell/install-ubuntu-deps.sh)
for scripts that install them.

Both `libcimpp` and `villas-node` are optional. Neither needs to be built from source, though
the images do not yet take the same route for both.

libcimpp publishes prebuilt `.deb` and `.rpm` packages per CIM version as release assets. The
Fedora and Debian images install those directly, while the Rocky image still builds it from
source:

```shell
# Pick the package matching your distribution and the CIM version you need.
wget https://github.com/sogno-platform/libcimpp/releases/download/release%2Fv2.2.0/libcimpp_CGMES_2.4.15_16FEB2016-2.2.0-Linux.deb
sudo apt-get install -y ./libcimpp_CGMES_2.4.15_16FEB2016-2.2.0-Linux.deb
sudo ldconfig
```

VILLASnode is served from the package repositories at <https://packages.fein-aachen.org>, which
carry both `debian/` and `redhat/`. Note that the images currently still build it from source,
pinned to a specific commit, so the packaged version is the more convenient route for a local
build but is not what CI exercises.

Building either from source remains supported, and the deps scripts above do that, which is what
you want when you need a specific commit rather than a release.

Sundials is only needed for the DAE solver. If your distribution does not package it, the
version CI uses is:

```shell
git clone --branch v3.2.1 --recurse-submodules --depth 1 https://github.com/LLNL/sundials.git
mkdir -p sundials/build && cd sundials/build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc) install
```

Cloning, building and installing then work exactly as in the container section above.

## CMake for Windows

Windows is built in CI on `windows-latest`, so the recipe below mirrors what
the `Build Windows` jobs of `.github/workflows/ci.yaml` run. You need Visual Studio with the C++ desktop
development workload, [CMake](https://cmake.org/) and
[Git for Windows](https://git-scm.com/download/win). For Python support, install Python 3 and
add it to your PATH. Let CMake pick the default generator rather than naming a Visual Studio
version, so the build follows whichever Visual Studio you have.

For the C++ libraries only:

```shell
mkdir build
cd build
cmake -DWITH_PYBIND=OFF ..
cmake --build . --target dpsim --target dpsim-models --parallel
```

For the Python bindings, install pybind11 first:

```shell
pip install pybind11[global]
mkdir build
cd build
cmake -DWITH_PYBIND=ON ..
cmake --build . --target dpsimpy --parallel
```

If CMake rejects the spdlog dependency because of its minimum policy version, add
`-DCMAKE_POLICY_VERSION_MINIMUM=3.5`, which is what CI currently does as a workaround.

Visual Studio is a multi-configuration generator, so the configuration is chosen at build time
with `cmake --build . --config Release`, not with `CMAKE_BUILD_TYPE`. DPsim adds its optimization
flags per configuration and per compiler: MSVC gets `/fp:fast` on top of the `/O2` that CMake
already supplies for `Release` and `RelWithDebInfo`, while the GCC and Clang flags `-Ofast`,
`-march=native` and `-flto` are never passed to `cl.exe`. `WITH_LTO` maps to `/GL` and `/LTCG`.
`WITH_MARCH_NATIVE` has no MSVC equivalent and is ignored with a warning.

The `dpsim-villas` library is not available on Windows, since it requires VILLASnode, which does
not build there. `WITH_VILLAS` therefore stays off and the `dpsimpyvillas` target does not exist,
so co-simulation examples cannot be built on Windows. The CIM reader is likewise not part of the
CI Windows build, as libcimpp is not installed there.

## CMake for macOS

DPsim can be built natively on macOS using the setup script provided in the repository. The setup
has been tested on Apple Silicon (`arm64`). macOS is currently not part of the DPsim CI matrix.

The Xcode Command Line Tools provide AppleClang and the macOS SDK. Install them first if they are
not already available:

```shell
xcode-select --install
```

Then run the macOS setup script from the root of the DPsim repository:

```shell
chmod +x packaging/Shell/install-macos.sh
./packaging/Shell/install-macos.sh
```

If Homebrew is not already installed, the setup script installs it first. It then installs the
required native development dependencies, including CMake, Ninja, Eigen 3, Graphviz, OpenMP
through Homebrew `libomp`, Python and the dependencies required by the repository development
tooling.

SuiteSparse and spdlog are fetched by the DPsim CMake configuration rather than installed
globally.

The setup creates a repository-local Python virtual environment named `dpsim-python` and
configures the regular `build` directory with the required macOS-specific dependency paths and
compiler settings. It then builds DPsim, the Python bindings and the C++ examples.

The DPsim Python package and its notebook dependencies are installed into the same environment,
and a `DPsim Python` Jupyter kernel is registered.

After the setup has completed, activate the environment with:

```shell
source dpsim-python/bin/activate
```

The build directory is already configured, so normal C++ development only requires:

```shell
cmake --build build --parallel "$(sysctl -n hw.ncpu)"
```

The required dependency paths and compiler settings are stored in the CMake build directory and
do not need to be supplied again for subsequent builds.

If changes affect the Python bindings or Python package, reinstall the package into the active
environment:

```shell
python -m pip install .
```

The installation can be verified with:

```shell
python -c "import dpsim, dpsimpy; print(dpsimpy.__file__)"
```

For a completely clean rebuild of the local macOS development environment, run:

```shell
CLEAN=1 ./packaging/Shell/install-macos.sh
```

This removes the local DPsim build directory, the `dpsim-python` virtual environment and the
registered DPsim Jupyter kernel before recreating them.

{{% alert title="Optional DPsim features" color="info" %}}
The default macOS setup builds the DPsim simulation core, Python bindings, OpenMP support,
Graphviz support and C++ examples.

CIM/CGMES support and VILLASnode integration are not enabled by the default macOS setup and
require their respective native dependencies to be configured separately.
{{% /alert %}}

## Python package

Wheels are produced by cibuildwheel in the `packaging-python` workflow, currently for
manylinux x86_64 and Windows x86-64, CPython 3.10 through 3.14. The Windows wheel is the
reduced one described in [install]({{< ref "/docs/User Guide/install.md" >}}), which also
carries the full matrix. The set of versions lives in the `tool.cibuildwheel` sections of
`pyproject.toml` and in the `python` matrix of `.github/workflows/packaging-python.yaml`, and
both have to be changed together.
To build a source distribution locally:

```shell
python3 -m build --sdist
```

### Nix

DPsim can be built using [Nix](https://nixos.org/), a declarative package manager for
reproducible builds. The following steps require a working single-user or multi-user
installation of Nix, but not necessarily NixOS.

DPsim uses the Flakes feature, which has to be enabled:

```shell
echo "experimental-features=nix-command flakes" > ~/.config/nix/nix.conf
```

Building DPsim, including all its dependencies:

```shell
nix build github:sogno-platform/dpsim
```

The result is placed in the `result` folder of the current directory. For development, a local
environment can be set up with:

```shell
nix develop github:sogno-platform/dpsim
```

The Flake reference above can be replaced by a local path such as `.` when the repository is
already checked out.

## Documentation

The Python and C++ references are generated by separate CMake targets. Both are also built and
published by the `documentation` workflow on every push to master.

### Python

Install [Sphinx](https://www.sphinx-doc.org/en/master/) or use the Docker image, then:

```shell
mkdir -p build && cd build
cmake ..
make docs
```

The result is generated in `build/docs/sphinx/html/`. Note that this target requires the Python
bindings, so it is only available when configured with `-DWITH_PYBIND=ON`.

### C++

Install [Doxygen](https://www.doxygen.nl/) or use the Docker image, then:

```shell
mkdir -p build && cd build
cmake ..
make docs_cxx
```

The result is generated in `build/docs/doxygen/html/`.

### Website

The surrounding website is a Hugo site under `docs/hugo`. It needs the Hugo version pinned in
the documentation workflow, since the theme does not build with arbitrary versions:

```shell
cd docs/hugo
npm ci
hugo --minify
```
