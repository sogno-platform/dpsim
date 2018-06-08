# rwthacs/dpsim-dev is built by dpsim-git/Packaging/Docker/Dockerfile.dev
FROM rwthacs/dpsim-dev AS builder

COPY . /dpsim/

RUN rm -rf /dpsim/build && mkdir /dpsim/build 
WORKDIR /dpsim/build

RUN cmake -DBUILD_EXAMPLES=OFF -DCPACK_GENERATOR=RPM ..
RUN make -j2 package

FROM fedora:28

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

COPY --from=builder /dpsim/build/*.rpm /tmp
RUN dnf -y install /tmp/*.rpm

ADD requirements.txt .
RUN pip3 install -r requirements.txt

CMD python3