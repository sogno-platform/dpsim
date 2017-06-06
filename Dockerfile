FROM villas/node
MAINTAINER Steffen Vogel <stvogel@eonerc.rwth-aachen.de>

RUN dnf -y install \
	gcc-c++ \
	make \
	eigen3-devel \
	doxygen \
	numpy \
	python-pandas
