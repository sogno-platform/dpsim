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
pip3 install --user -r requirements.txt
pip3 install --user -r requirements-jupyter.txt
dnf -y --refresh install npm
jupyter nbextension enable --py widgetsnbextension
jupyter labextension install @jupyter-widgets/jupyterlab-manager

# VILLAS dependencies
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
git clone --recurse-submodules https://github.com/CIM-IEC/libcimpp.git && \
mkdir -p libcimpp/build && cd libcimpp/build && \
cmake -DCMAKE_INSTALL_LIBDIR=/usr/local/lib64 -DUSE_CIM_VERSION=IEC61970_16v29a -DBUILD_SHARED_LIBS=ON -DBUILD_ARABICA_EXAMPLES=OFF .. && sudo make -j$(nproc) install && \
rm -rf /tmp/libcimpp

# Install VILLAS from source
cd /tmp && \
git -c submodule.fpga.update=none clone --recursive https://git.rwth-aachen.de/acs/public/villas/node.git villasnode && \
mkdir -p villasnode/build && cd villasnode/build && \
cmake -DCMAKE_INSTALL_LIBDIR=/usr/local/lib64 .. && make -j$(nproc) install && \
rm -rf /tmp/villasnode