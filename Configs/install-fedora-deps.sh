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
    
pip3 install --user -r requirements.txt
pip3 install --user -r requirements-jupyter.txt
dnf -y --refresh install npm
jupyter nbextension enable --py widgetsnbextension
jupyter labextension install @jupyter-widgets/jupyterlab-manager