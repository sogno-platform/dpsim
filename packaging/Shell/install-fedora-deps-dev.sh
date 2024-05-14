#!/bin/bash

CIM_VERSION=${CIM_VERSION:-CGMES_2.4.15_16FEB2016}
VILLAS_VERSION=${VILLAS_VERSION:-c976cd62d8c6667a078be0785ca3e623a05f7456}

MAKE_PROCS=${MAKE_PROCS:-$(nproc)}
MAKE_OPTS+="-j${MAKE_PROCS}"

dnf -y update

# Toolchain
dnf -y install \
    gcc gcc-c++ clang \
    git \
    rpmdevtools rpm-build \
    make cmake pkgconfig \
    python3-pip \
    cppcheck

# Tools needed for developement
dnf -y install \
    doxygen graphviz \
    gdb

# Dependencies
dnf -y install \
    python3-devel \
    eigen3-devel \
    libxml2-devel \
    graphviz-devel \
    gsl-devel \
    spdlog-devel \
    fmt-devel

# Sundials
cd /tmp && \
git clone --branch v3.2.1 --recurse-submodules --depth 1 https://github.com/LLNL/sundials.git && \
mkdir -p sundials/build && cd sundials/build && \
cmake ${CMAKE_OPTS} .. \
    -DCMAKE_BUILD_TYPE=Release  && \
make ${MAKE_OPTS} install

# Install some debuginfos
dnf -y debuginfo-install \
    python3

# CIMpp and VILLASnode are installed here
LD_LIBRARY_PATH="/usr/local/lib64:${LD_LIBRARY_PATH}"

# Minimal VILLASnode dependencies
dnf -y install \
    openssl-devel \
    libuuid-devel \
    libcurl-devel \
    jansson-devel \
    libwebsockets-devel

# Optional VILLASnode dependencies
dnf -y install \
    mosquitto-devel \
    libconfig-devel \
    libnl3-devel

pip3 install --upgrade setuptools
pip3 install --upgrade wheel
pip3 install .[test,docs]

cd /tmp && \
    git clone --recurse-submodules --depth 1 https://github.com/cim-iec/libcimpp.git && \
    mkdir -p libcimpp/build && cd libcimpp/build && \
    cmake ${CMAKE_OPTS} .. \
        -DBUILD_SHARED_LIBS=ON \
        -DCMAKE_INSTALL_LIBDIR=/usr/local/lib64 \
        -DUSE_CIM_VERSION=${CIM_VERSION} \
        -DBUILD_ARABICA_EXAMPLES=OFF && \
    make ${MAKE_OPTS} install && \
    rm -rf /tmp/libcimpp

cd /tmp && \
    git clone --recurse-submodules https://github.com/VILLASframework/node.git villas-node && \
    mkdir -p villas-node/build && cd villas-node/build && \
    git checkout ${VILLAS_VERSION} && \
    cmake ${CMAKE_OPTS} .. \
        -DCMAKE_INSTALL_LIBDIR=/usr/local/lib64 && \
    make ${MAKE_OPTS} install && \
    rm -rf /tmp/villas-node

# Activate Jupyter extensions
dnf -y --refresh install npm
pip3 install jupyterlab jupyter_contrib_nbextensions nbconvert nbformat
