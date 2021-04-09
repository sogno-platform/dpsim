# rwthacs/dpsim-dev is built by dpsim-git/Packaging/Docker/Dockerfile.dev
ARG BUILDER_IMAGE=rwthacs/dpsim-dev
ARG DOCKER_TAG=latest
ARG CI
ARG CI_COMMIT_SHA
ARG CI_COMMIT_REF_NAME
ARG CI_COMMIT_TAG
ARG CIM_VERSION

FROM ${BUILDER_IMAGE}:${DOCKER_TAG}

COPY . /dpsim/

RUN rm -rf /dpsim/build && mkdir /dpsim/build
WORKDIR /dpsim/build

RUN cmake -DBUILD_EXAMPLES=OFF ..
ENV PYTHONPATH=/dpsim/build:/dpsim/build/Source/Python:/dpsim/Source/Python:
# PYBIND package cannot be installed yet
#RUN make -j$(nproc) install && ldconfig
RUN make -j$(nproc)

WORKDIR /dpsim
EXPOSE 8888
CMD [ "jupyter", "lab", "--ip=0.0.0.0", "--allow-root", "--no-browser", "--LabApp.token=3adaa57df44cea75e60c0169e1b2a98ae8f7de130481b5bc" ]

LABEL \
	org.label-schema.schema-version = "1.0" \
	org.label-schema.name = "DPsim" \
	org.label-schema.license = "MPL 2.0" \
	org.label-schema.vendor = "Institute for Automation of Complex Power Systems, RWTH Aachen University" \
	org.label-schema.author.name = "Markus Mirz" \
	org.label-schema.author.email = "mmirz@eonerc.rwth-aachen.de" \
	org.label-schema.url = "http://fein-aachen.org/projects/dpsim/" \
	org.label-schema.vcs-url = "https://git.rwth-aachen.de/acs/public/simulation/dpsim"
