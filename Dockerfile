FROM fedora:latest
MAINTAINER Steffen Vogel <stvogel@eonerc.rwth-aachen.de>

ADD https://villas.fein-aachen.org/packages/villas.repo /etc/yum.repos.d/

RUN dnf -y update

RUN dnf -y install \
	git \
	gcc-c++ \
	make \
	cmake \
	eigen3-devel \
	doxygen \
	expat \
	python3-pandas \
    python3-numpy

RUN dnf -y install villas-node-devel
