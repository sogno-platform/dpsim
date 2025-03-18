# sogno/dpsim:dev is built by dpsim-git/packaging/Docker/Dockerfile.dev
ARG BASE_IMAGE=sogno/dpsim:dev

FROM ${BASE_IMAGE}

ARG NotebookApp.default_url
ARG ip
ARG port
ARG NB_USER=jovyan
ARG NB_UID=1000
ENV USER ${NB_USER}
ENV NB_UID ${NB_UID}
ENV HOME /home/${NB_USER}

RUN adduser \
    --comment "Default user" \
    --uid ${NB_UID} \
    ${NB_USER}

COPY . ${HOME}/dpsim
USER root
RUN chown -R ${NB_UID} ${HOME}
USER ${NB_USER}
RUN rm -rf ${HOME}/dpsim/build && mkdir ${HOME}/dpsim/build
WORKDIR ${HOME}/dpsim

RUN python3 -m build --wheel --no-isolation --verbose
RUN python3 -m pip install ./dist/dpsim*