# sogno/dpsim-dev is built by dpsim-git/Packaging/Docker/Dockerfile.dev
ARG BUILDER_IMAGE=sogno/dpsim-dev
ARG DOCKER_TAG=latest

FROM ${BUILDER_IMAGE}:${DOCKER_TAG}

COPY . /dpsim/
RUN rm -rf /dpsim/build && mkdir /dpsim/build
WORKDIR /dpsim

RUN python3 -m build --wheel
RUN python3 -m pip install ./dist/dpsim*

EXPOSE 8888
CMD [ "jupyter", "lab", "--ip=0.0.0.0", "--allow-root", "--no-browser", "--LabApp.token=3adaa57df44cea75e60c0169e1b2a98ae8f7de130481b5bc" ]

LABEL \
	org.label-schema.schema-version = "1.0.0" \
	org.label-schema.name = "DPsim" \
	org.label-schema.license = "MPL 2.0" \
	org.label-schema.url = "http://dpsim.fein-aachen.org/" \
	org.label-schema.vcs-url = "https://github.com/sogno-platform/dpsim"
