---
title: "Build"
linkTitle: "Build"
date: 2023-05-03
---

# Docker based

Clone the repository

```shell
git clone git@github.com:sogno-platform/dpsim.git
```

or using https if you do not have an account

```shell
git clone https://github.com/sogno-platform/dpsim.git
```

In the repository, there is a Docker file with all required dependencies

```shell
cd dpsim
docker build -t sogno/dpsim:dev -f Packaging/Docker/Dockerfile.dev .
```

Alternatively, the image can be pulled from DockerHub like so

```shell
docker pull sogno/dpsim:dev
```

For OS specific instructions on how to install requirements, see the sections below.

Next, run a Docker container

```shell
cd dpsim
docker run -it -p 8888:8888 -v $(pwd):/dpsim --privileged sogno/dpsim:dev bash
```

The option `-p` maps the port 8888 of the container to the docker host. This is required to access the jupyter lab instance inside the container. The option `--privileged` is required for debug builds.

For Windows, you might need to specify the current directory with curly brackets

```shell
docker run -it -p 8888:8888 -v ${pwd}:/dpsim --privileged sogno/dpsim:dev bash
```

Now, you should be in an interactive session inside the docker container.

The DPsim C++ and Python library without C++ examples or documentation can be built as follows

```shell
cd /dpsim
mkdir build && cd build
cmake ..
cmake --build . --target dpsimpy
```

If you need other libraries that are not built by default, you need to target them specifically, for example if you need `dpsimpy´ and ´dpsimpyvillas´:

```shell
cmake --build . --target dpsimpy dpsimpyvillas
```

To build everything run

```shell
cmake --build .
```

To use other libraries that are installed, use the relevant option defined in the CMakeList.txt files, for example for GSL below, and then build as usual:

```shell
cmake .. -DWITH_GSL=ON
```

If you would like to use the Python package, it has to be added to the path.
The following command adds the dpsimpy C++/Python package as well as the dpsim pure Python package.

```shell
cd /dpsim/build
export PYTHONPATH=$(pwd):$(pwd)/../python/src/
```

If you are using `conda` or other ways to develop with environments, please keep in mind that this will become specific for your setup. For this case, from within the environment already active:

```shell
cd /dpsim/build
conda develop $(pwd) && conda develop $(pwd)/Source/Python && conda develop $(pwd)/../Source/Python
```

To run JupyterLab

```shell
cd /dpsim
jupyter lab --ip="0.0.0.0" --allow-root --no-browser
```

To install DPsim run

```shell
cd /dpsim/build
sudo make install
```

# CMake for Linux

The most recent list of requirements can be found in the Dockerfiles.

Make sure that the required dependencies are installed.
The [fedora installation script](https://github.com/sogno-platform/dpsim/blob/c40e283338574e0ba7cd9861c70f1e41aa3399ba/packaging/Shell/install-fedora-deps.sh) in the DPsim repository is a good place to start from.

**Note:** There are currently no Debian packages for `villas-node` and `libcimpp16v29a`.
If you want to use these optional feature, you have to build them manually.

Install Sundials

```shell
git clone --branch v3.1.1 https://github.com/LLNL/sundials.git
mkdir sundials/build
pushd sundials/build
cmake .. \
    -DBUILD_SHARED_LIBS=ON \
    -DBUILD_STATIC_LIBS=OFF \
    -DEXAMPLES_ENABLE_C=OFF
make -j$(nproc) install
popd
```

The following steps to clone, build and install are the same as for the Docker setup.

# CMake for Windows

Make sure that the required dependecies are installed:

- Visual Studio 2017 with C++ Desktop development package
- [CMake](https://cmake.org/) for Windows
- [Git for Windows](https://git-scm.com/download/win)
- For Python support, install Python3, for example, Anaconda, and add Python to your PATH.

Clone the project as explained for Docker.

Open a windows command prompt and navigate into the new DPsim folder.
Generate a Visual Studio project with CMake and use it to build the project

```shell
mkdir build
cd build
cmake -G "Visual Studio 15 2017 Win64" ..
```

Open Visual Studio and load the Visual Studio project from the build directory within the DPsim folder.

You can either build the project from within Visual Studio or from the command line by running the following command in the windows command prompt

```shell
cmake --build .
```

To install the Python package use Visual Studio and the Release configuration to build the DPsim Python module and then build the INSTALL project.

# CMake for macOS

Make sure that the required dependencies are installed

```shell
/usr/bin/ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"
brew install gcc9 git cmake graphviz python3 gsl eigen spdlog
sudo pip3 install numpy
```

Clone the source as explained for the Docker setup.

Compile

```shell
mkdir build
cmake ..
make -j$(sysctl -n hw.ncpu)
```

To install the generated Python module to your system

```shell
sudo make install
```

# Python Package for pypi

Follow the previous steps to set up the Docker container.

To build the Python package run

```shell
python3 setup.py bdist_wheel
```

# Documentation

## Python

Install [Sphinx](https://www.sphinx-doc.org/en/master/) or use the Docker image.

Generate the Python documentation by running Sphinx via CMake:

```shell
mkdir -p build && cd build
cmake ..
make docs
```

The resulting documentation will be generated in `Documentation/html/`.

## C++

Install [Doxygen](http://www.doxygen.nl/) or use the Docker image.

Generate the C++ documentation by running Doxygen via CMake:

```shell
mkdir -p build && cd build
cmake ..
make docs_cxx
```

The resulting documentation will be generated in `Documentation/html/Cxx`.
