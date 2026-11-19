# SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
# SPDX-License-Identifier: MPL-2.0

ARG BASE_IMAGE=sogno/dpsim:dev
FROM ${BASE_IMAGE}

ARG NotebookApp.default_url
ARG ip
ARG port
ARG NB_USER=jovyan
ARG NB_UID=1000

ENV USER=${NB_USER}
ENV NB_UID=${NB_UID}
ENV HOME=/home/${NB_USER}
ENV PYTHONPATH=${HOME}/dpsim/build
ENV DPSIM_CXX_EXAMPLES_DIR=${HOME}/dpsim/build/dpsim/examples/cxx

RUN adduser --comment "Default user" --uid ${NB_UID} ${NB_USER}

COPY . ${HOME}/dpsim
WORKDIR ${HOME}/dpsim

RUN python3 -m pip install notebook jupyterlab jupyterhub

# Build the same source tree in place so notebooks that resolve the repository
# root with git and expect build/dpsim/examples/cxx continue to work.
RUN cmake -S . -B build \
	-DCMAKE_BUILD_TYPE=Release \
	-DCIM_VERSION=CGMES_2.4.15_16FEB2016 \
	-DDPSIM_BUILD_DOC=OFF \
	-DDPSIM_BUILD_EXAMPLES=ON \
	-DWITH_PYBIND=ON \
	&& cmake --build build --parallel "$(nproc)" \
	&& cp -r python/src/dpsim build/ \
	&& python3 .binder/smoke_test.py \
	&& chown -R ${NB_UID}:${NB_UID} ${HOME}

USER ${NB_USER}
