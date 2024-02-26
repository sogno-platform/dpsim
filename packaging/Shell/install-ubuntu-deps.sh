#!/bin/bash

CIM_VERSION=${CIM_VERSION:-CGMES_2.4.15_16FEB2016}
VILLAS_VERSION=${VILLAS_VERSION:-c976cd62d8c6667a078be0785ca3e623a05f7456}

MAKE_PROCS=${MAKE_PROCS:-$(nproc)}
MAKE_OPTS+="-j${MAKE_PROCS}"

# run with sudo
apt-get -y update

apt-get -y install \
    build-essential git clang gdb make cmake \
    doxygen graphviz \
    python3-distutils python3-dev python3-pip \
    pkg-config

# Dependencies
apt-get -y install \
	libeigen3-dev \
    libxml2-dev \
	libfmt-dev \
    libspdlog-dev \
	libgraphviz-dev \
	libsundials-dev

pip3 install --user .
apt-get -y install npm
jupyter nbextension enable --py widgetsnbextension
jupyter labextension install @jupyter-widgets/jupyterlab-manager

apt-get -y install \
    libssl-dev \
    uuid-dev \
    libconfig-dev \
    libnl-3-dev \
    libcurl4-openssl-dev \
    libjansson-dev \
    libwebsockets-dev \
    mosquitto-dev

# Install CIMpp from source
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

# Install VILLASnode from source
cd /tmp && \
	git clone --recurse-submodules https://github.com/VILLASframework/node.git villas-node && \
	mkdir -p villas-node/build && cd villas-node/build && \
    git checkout ${VILLAS_VERSION} && \
	cmake ${CMAKE_OPTS} .. \
        -DCMAKE_INSTALL_LIBDIR=/usr/local/lib64 && \
    make ${MAKE_OPTS} install && \
	rm -rf /tmp/villas-node
