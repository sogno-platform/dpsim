FROM fedora:36 AS base

ARG CIM_VERSION=CGMES_2.4.15_16FEB2016
ARG VILLAS_VERSION=411b0ad49e2629ad41c6918d2a6c51e9a72220b4

ARG CMAKE_OPTS
ARG MAKE_OPTS=-j4

ARG NotebookApp.default_url
ARG ip
ARG port

LABEL \
	org.opencontainers.image.title="DPsim" \
	org.opencontainers.image.licenses="MPL 2.0" \
	org.opencontainers.image.url="http://dpsim.fein-aachen.org/" \
	org.opencontainers.image.source="https://github.com/sogno-platform/dpsim"

RUN dnf -y update

# Toolchain
RUN dnf -y install \
	gcc gcc-c++ clang \
	git \
	rpmdevtools rpm-build \
	make cmake pkgconfig \
	python3-pip \
	cppcheck

# Tools needed for developement
RUN dnf -y install \
	doxygen graphviz \
	gdb \
	procps-ng

# Dependencies
RUN dnf --refresh -y install \
	python3-devel \
	eigen3-devel \
	libxml2-devel \
	graphviz-devel \
	spdlog-devel \
	fmt-devel

# Install some debuginfos
RUN dnf -y debuginfo-install \
	python3

# Build & Install sundials
RUN cd /tmp && \
	git clone --branch v3.2.1 --recurse-submodules --depth 1 https://github.com/LLNL/sundials.git && \
	mkdir -p sundials/build && cd sundials/build && \
	cmake ${CMAKE_OPTS} .. \
		-DCMAKE_BUILD_TYPE=Release && \
	make ${MAKE_OPTS} install

# CIMpp and VILLASnode are installed here
ENV LD_LIBRARY_PATH="/usr/local/lib64:${LD_LIBRARY_PATH}"

# Install minimal VILLASnode dependencies
RUN dnf -y install \
	openssl-devel \
	libuuid-devel \
	libcurl-devel \
	jansson-devel \
	libwebsockets-devel

# Install optional VILLASnode dependencies
RUN dnf -y install \
	mosquitto-devel \
	libconfig-devel \
	libnl3-devel

# Python dependencies
ADD requirements.txt .
RUN pip3 install --upgrade wheel build
RUN pip3 install -r requirements.txt

# Install CIMpp from source
RUN cd /tmp && \
	git clone --recurse-submodules --depth 1 https://github.com/cim-iec/libcimpp.git && \
	mkdir -p libcimpp/build && cd libcimpp/build && \
	cmake ${CMAKE_OPTS} ..\
		-DBUILD_SHARED_LIBS=ON \
		-DCMAKE_INSTALL_LIBDIR=/usr/local/lib64 \
		-DUSE_CIM_VERSION=${CIM_VERSION} \
		-DBUILD_ARABICA_EXAMPLES=OFF && \
	make ${MAKE_OPTS} install && \
	rm -rf /tmp/libcimpp

# Install VILLASnode from source
RUN cd /tmp && \
	git clone --recurse-submodules https://github.com/VILLASframework/node.git villas-node && \
	mkdir -p villas-node/build && cd villas-node/build && \
	git checkout ${VILLAS_VERSION} && \
	cmake ${CMAKE_OPTS} .. \
		-DCMAKE_INSTALL_LIBDIR=/usr/local/lib64 \
		-DDOWNLOAD_GO=OFF && \
	make ${MAKE_OPTS} install && \
	rm -rf /tmp/villas-node

RUN pip3 install --no-cache \
	jupyter \
	jupyterlab \
	jupyter_contrib_nbextensions \
	nbconvert \
	nbformat

ARG NB_USER=jovyan
ARG NB_UID=1000
ENV USER ${NB_USER}
ENV NB_UID ${NB_UID}
ENV HOME /home/${NB_USER}

RUN adduser \
    --comment "Default user" \
    --uid ${NB_UID} \
    ${NB_USER}

COPY . ${HOME}/dpsim
USER root
RUN chown -R ${NB_UID} ${HOME}
USER ${NB_USER}
RUN rm -rf ${HOME}/dpsim/build && mkdir ${HOME}/dpsim/build
WORKDIR ${HOME}/dpsim

RUN python3 -m build --wheel
RUN python3 -m pip install ./dist/dpsim*
