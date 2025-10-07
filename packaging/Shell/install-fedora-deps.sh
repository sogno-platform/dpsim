#!/bin/bash

CIM_VERSION=${CIM_VERSION:-CGMES_2.4.15_16FEB2016}
VILLAS_VERSION=${VILLAS_VERSION:-5606793aaee2d1eeb663180d3e52718fa8bd5931}

MAKE_PROCS=${MAKE_PROCS:-$(nproc)}
MAKE_OPTS+="-j${MAKE_PROCS}"

dnf -y update

# Toolchain
dnf -y install \
    git clang gdb ccache \
    redhat-rpm-config \
    rpmdevtools \
    make cmake ninja-build \
    doxygen \
    graphviz \
    pandoc \
    python3-pip \
    pkg-config

# Dependencies
dnf --refresh -y install \
	python3-devel \
	eigen3-devel \
    libxml2-devel \
	spdlog-devel \
	graphviz-devel \
	sundials-devel \
	gsl-devel

# Python
pip3 install -U setuptools
pip3 install -U wheel
pip3 install -r requirements.txt

# Activate Jupyter extensions
dnf -y --refresh install npm
pip3 install jupyterlab jupyter_contrib_nbextensions nbconvert nbformat

# VILLASnode dependencies
sudo dnf install \
    openssl-devel \
    libuuid-devel \
    libconfig-devel \
    libnl3-devel \
    libcurl-devel \
    jansson-devel \
    libwebsockets-devel \
    mosquitto-devel \

# add lib64 to library search path
grep -qxF '/usr/local/lib64' /etc/ld.so.conf || echo '/usr/local/lib64' | sudo tee -a /etc/ld.so.conf
sudo ldconfig

# Install CIMpp from source
cd /tmp && \
	git clone --recurse-submodules --depth 1 https://github.com/cim-iec/libcimpp.git && \
	mkdir -p libcimpp/build && cd libcimpp/build && \
	cmake ${CMAKE_OPTS} .. \
        -DCMAKE_INSTALL_LIBDIR=/usr/local/lib64 \
        -DUSE_CIM_VERSION=${CIM_VERSION} \
        -DBUILD_SHARED_LIBS=ON \
        -DBUILD_ARABICA_EXAMPLES=OFF && \
    make ${MAKE_OPTS} install && \
	rm -rf /tmp/libcimpp

# Install VILLASnode from source
cd /tmp && \
	git clone --recurse-submodules https://github.com/VILLASframework/node.git villas-node && \
	mkdir -p villas-node/build && cd villas-node/build && \
    git checkout ${VILLAS_VERSION} && \
	cmake ${CMAKE_OPTS} .. \
		-DCMAKE_INSTALL_LIBDIR=/usr/local/lib64 && \
    make ${MAKE_OPTS} install && \
	rm -rf /tmp/villas-node
