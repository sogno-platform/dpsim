CIM_VERSION=CGMES_2.4.15_16FEB2016

# run with sudo
apt update

apt install \
    build-essential git clang gdb make cmake \
    doxygen graphviz \
    python3-distutils python3-dev python3-pip \
    pkg-config

# Dependencies
apt install \
	libeigen3-dev \
    libxml2-dev \
	libfmt-dev \
    libspdlog-dev \
	libgraphviz-dev \
	libsundials-dev

pip3 install --user -r requirements.txt
pip3 install --user -r requirements-jupyter.txt
apt install npm
jupyter nbextension enable --py widgetsnbextension
jupyter labextension install @jupyter-widgets/jupyterlab-manager

sudo apt install \
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
	git clone --recursive https://github.com/cim-iec/libcimpp.git && \
	mkdir -p libcimpp/build && cd libcimpp/build && \
	cmake -DCMAKE_INSTALL_LIBDIR=/usr/local/lib64 -DUSE_CIM_VERSION=${CIM_VERSION} -DBUILD_SHARED_LIBS=ON -DBUILD_ARABICA_EXAMPLES=OFF .. && make -j$(nproc) install && \
	rm -rf /tmp/libcimpp

# Install VILLAS from source
cd /tmp && \
	git -c submodule.fpga.update=none clone --recursive https://git.rwth-aachen.de/acs/public/villas/node.git villasnode && \	
	mkdir -p villasnode/build && cd villasnode/build && \
	git -c submodule.fpga.update=none checkout dpsim-villas && git -c submodule.fpga.update=none submodule update --recursive && \
	cmake -DCMAKE_INSTALL_LIBDIR=/usr/local/lib64 .. && make -j$(nproc) install && \
	rm -rf /tmp/villasnode