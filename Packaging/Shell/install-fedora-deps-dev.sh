VILLAS_VERSION=0.8.0
CIM_VERSION=CGMES_2.4.15_16FEB2016

dnf -y update

# Toolchain
dnf -y install \
	gcc gcc-c++ clang \
	git \
	rpmdevtools rpm-build \
	make cmake pkgconfig \
	python3-pip

# Tools needed for developement
dnf -y install \
doxygen graphviz \
gdb

# Dependencies
dnf --refresh -y install \
	python3-devel \
	eigen3-devel \
	libxml2-devel \
	graphviz-devel \
	gsl-devel \
	spdlog-devel \
	fmt-devel

# Sundials
cd /tmp && \
git clone --recursive https://github.com/LLNL/sundials.git && \
mkdir -p sundials/build && cd sundials/build && \
git checkout v3.2.1 && \
cmake -DCMAKE_BUILD_TYPE=Release ..  && \
make -j$(nproc) install

# Install some debuginfos
dnf -y debuginfo-install \
    python3

# CIMpp and VILLAS are installed here
LD_LIBRARY_PATH="/usr/local/lib64:${LD_LIBRARY_PATH}"

# minimal VILLAS dependencies
dnf -y install \
    openssl-devel \
    libuuid-devel \
    libcurl-devel \
    jansson-devel \
    libwebsockets-devel

# optional VILLAS dependencies
dnf -y install \
    mosquitto-devel \
	libconfig-devel \
    libnl3-devel

pip3 install -U setuptools
pip3 install -U wheel
pip3 install -r requirements.txt

cd /tmp && \
	git clone --recursive https://github.com/cim-iec/libcimpp.git && \
	mkdir -p libcimpp/build && cd libcimpp/build && \
	cmake -DCMAKE_INSTALL_LIBDIR=/usr/local/lib64 -DUSE_CIM_VERSION=${CIM_VERSION} -DBUILD_SHARED_LIBS=ON -DBUILD_ARABICA_EXAMPLES=OFF .. && make -j$(nproc) install && \
	rm -rf /tmp/libcimpp

cd /tmp && \
	git -c submodule.fpga.update=none clone --recursive https://git.rwth-aachen.de/acs/public/villas/node.git villasnode && \	
	mkdir -p villasnode/build && cd villasnode/build && \
	git -c submodule.fpga.update=none checkout dpsim-villas && git -c submodule.fpga.update=none submodule update --recursive && \
	cmake -DCMAKE_INSTALL_LIBDIR=/usr/local/lib64 .. && make -j$(nproc) install && \
	rm -rf /tmp/villasnode

# Activate Jupyter extensions
dnf -y --refresh install npm
pip3 install jupyterlab jupyter_contrib_nbextensions nbconvert nbformat
