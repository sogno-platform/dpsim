FROM fedora:27

LABEL \
	org.label-schema.schema-version = "1.0" \
	org.label-schema.name = "DPsim" \
	org.label-schema.license = "GPL-3.0" \
	org.label-schema.vendor = "Institute for Automation of Complex Power Systems, RWTH Aachen University" \
	org.label-schema.author.name = "Steffen Vogel" \
	org.label-schema.author.email = "stvogel@eonerc.rwth-aachen.de" \
	org.label-schema.url = "http://fein-aachen.org/projects/dpsim/" \
	org.label-schema.vcs-url = "https://git.rwth-aachen.de/acs/core/simulation/DPsim"

ADD https://villas.fein-aachen.org/packages/villas.repo /etc/yum.repos.d/

RUN dnf -y update

# Toolchain
RUN dnf -y install \
	git \
	gcc-c++ gdb \
	redhat-rpm-config \
	rpmdevtools \
	make cmake \
	doxygen \
	graphviz \
	pandoc \
	python3-pip

# Install some debuginfos
RUN dnf -y debuginfo-install \
	python3 \
	villas-node

# Dependencies
RUN dnf -y install \
	python3-devel \
	eigen3-devel \
	villas-node-devel \
	libcimpp16v29a

RUN dnf -y install \
	pkg-config \
	expat \
	expat-devel

# Python Packages
ADD requirements.txt .
RUN pip3 install -r requirements.txt

ENV LD_LIBRARY_PATH /usr/local/lib64

RUN useradd -m dpsim
COPY Source/ /home/dpsim/Source/
COPY CMake/ /home/dpsim/CMake/
COPY Documentation/ /home/dpsim/Documentation/
COPY Examples/ /home/dpsim/Examples/
COPY Dependencies/ /home/dpsim/Dependencies/
COPY CMakeLists.txt COPYING.md README.md setup.py /home/dpsim/
WORKDIR /home/dpsim
RUN python3 setup.py install
