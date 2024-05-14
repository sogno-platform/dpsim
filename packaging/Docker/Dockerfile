# sogno/dpsim:dev is built by dpsim-git/packaging/Docker/Dockerfile.dev
ARG BASE_IMAGE=sogno/dpsim:dev

FROM ${BASE_IMAGE}

LABEL \
	org.opencontainers.image.title = "DPsim" \
	org.opencontainers.image.licenses = "MPL 2.0" \
	org.opencontainers.image.url = "http://dpsim.fein-aachen.org/" \
	org.opencontainers.image.source = "https://github.com/sogno-platform/dpsim"

COPY . /dpsim/
RUN rm -rf /dpsim/build && mkdir /dpsim/build
WORKDIR /dpsim

RUN python3 -m build --wheel
RUN python3 -m pip install ./dist/dpsim*

CMD [ "jupyter", "lab", "--ip=0.0.0.0", "--allow-root", "--no-browser", "--LabApp.token=3adaa57df44cea75e60c0169e1b2a98ae8f7de130481b5bc" ]
