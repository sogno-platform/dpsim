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
git clone --recurse-submodules https://github.com/cim-iec/libcimpp.git && \
mkdir -p libcimpp/build && cd libcimpp/build && \
cmake -DCMAKE_INSTALL_LIBDIR=/usr/local/lib -DUSE_CIM_VERSION=IEC61970_16v29a -DBUILD_SHARED_LIBS=ON -DBUILD_ARABICA_EXAMPLES=OFF .. && sudo make -j$(nproc) install && \
rm -rf /tmp/libcimpp

# Install VILLAS from source
cd /tmp && \
git -c submodule.fpga.update=none clone --recursive https://git.rwth-aachen.de/acs/public/villas/node.git villasnode && \
mkdir -p villasnode/build && cd villasnode/build && \
cmake -DCMAKE_INSTALL_LIBDIR=/usr/local/lib .. && make -j$(nproc) install && \
rm -rf /tmp/villasnode