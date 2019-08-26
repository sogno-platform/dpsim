#!/bin/bash
set -e -x

VILLAS_VERSION=0.8.0

# Set up DPsim dependencies
curl https://packages.fein-aachen.org/redhat/fein.repo > /etc/yum.repos.d/fein.repo

# Enable Extra Packages for Enterprise Linux (EPEL) repo
yum -y install epel-release

# Toolchain
yum -y install \
	devtoolset-7-toolchain \
	pkgconfig make cmake3 \
	git tar \
	expat-devel \
	graphviz-devel \
	sundials-devel \
	gsl-devel \
    libxml2-devel \
	# libvillas-devel-${VILLAS_VERSION} \
	# villas-node-${VILLAS_VERSION}

# rm -rf spdlog
# git clone --recursive https://github.com/gabime/spdlog.git
# mkdir -p spdlog/build
# pushd spdlog/build
# cmake ..
# make -j$(nproc) install
# popd

# Make cmake3 the default
update-alternatives --install /usr/bin/cmake cmake /usr/bin/cmake3 1

# Compile wheels
for PYDIR in /opt/python/{cp35,cp36,cp37}-*; do
	LIB=(${PYDIR}/lib/python*)
	INC=(${PYDIR}/include/python*)
	EXC=${PYDIR}/bin/python3
    
	export CMAKE_OPTS="-DWITH_EIGEN_SUBMODULE=ON \
					   -DWITH_CIM_SUBMODULE=ON \
					   -DPYTHON_EXECUTABLE=${EXC} \
					   -DPYTHON_INCLUDE_DIR=${INC} \
					   -DPYTHON_LIBRARY=${LIB}"

    "${PYDIR}/bin/pip" install -r /dpsim/requirements.txt
    "${PYDIR}/bin/pip" wheel /dpsim/ -w wheelhouse/
done

# Bundle external shared libraries into the wheels
for whl in wheelhouse/*.whl; do
    auditwheel repair "$whl" --plat $PLAT -w /dpsim/
done

# Install packages and test
for PYDIR in /opt/python/{cp35,cp36,cp37}-*; do
    "${PYDIR}/bin/pip" install python-manylinux-demo --no-index -f /io/wheelhouse
    (cd "$HOME"; "${PYDIR}/bin/nosetests" pymanylinuxdemo)
done