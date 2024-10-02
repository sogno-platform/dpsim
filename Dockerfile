FROM fedora:34 AS base

LABEL \
	org.label-schema.schema-version = "1.0.0" \
	org.label-schema.name = "DPsim" \
	org.label-schema.license = "MPL 2.0" \
	org.label-schema.url = "http://dpsim.fein-aachen.org/" \
	org.label-schema.vcs-url = "https://github.com/sogno-platform/dpsim"

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
	git clone --recursive https://github.com/LLNL/sundials.git && \
	mkdir -p sundials/build && cd sundials/build && \
	git checkout v3.2.1 && \
	cmake -DCMAKE_BUILD_TYPE=Release ..  && \
	make -j$(nproc) install

# CIMpp and VILLAS are installed here
ENV LD_LIBRARY_PATH="/usr/local/lib64:${LD_LIBRARY_PATH}"

# minimal VILLAS dependencies
RUN dnf -y install \
    openssl-devel \
    libuuid-devel \
    libcurl-devel \
    jansson-devel \
    libwebsockets-devel

# optional VILLAS dependencies
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
	git clone --recursive https://github.com/cim-iec/libcimpp.git && \
	mkdir -p libcimpp/build && cd libcimpp/build && \
	cmake -DCMAKE_INSTALL_LIBDIR=/usr/local/lib64 -DUSE_CIM_VERSION=CGMES_2.4.15_16FEB2016 -DBUILD_SHARED_LIBS=ON -DBUILD_ARABICA_EXAMPLES=OFF .. && make -j$(nproc) install && \
	rm -rf /tmp/libcimpp

# Install VILLAS from source
ENV LC_ALL C.UTF-8
ENV LANG C.UTF-8

RUN cd /tmp && \
	git -c submodule.fpga.update=none clone --recursive https://git.rwth-aachen.de/acs/public/villas/node.git villasnode && \	
	mkdir -p villasnode/build && cd villasnode/build && \
	git -c submodule.fpga.update=none checkout b94746effb015aa98791c0e319ef11223d18e8b0 && git -c submodule.fpga.update=none submodule update --recursive && \
	cmake -DCMAKE_INSTALL_LIBDIR=/usr/local/lib64 .. && make -j$(nproc) install && \
	rm -rf /tmp/villasnode

# Remove this part in the future and use dedicated jupyter Dockerfile
# Activate Jupyter extensions
RUN dnf -y --refresh install npm
RUN pip3 install jupyter jupyterlab jupyter_contrib_nbextensions nbconvert nbformat

EXPOSE 8888

# target for vscode dev container
FROM base AS dev-vscode

# create a non-root user for vscode to use
ARG USERNAME=dpsim
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
	&& useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
	&& echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
	&& chmod 0440 /etc/sudoers.d/$USERNAME

FROM base