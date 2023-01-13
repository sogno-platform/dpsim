#!/bin/bash

# Jupyter Extension Matplotlib Widget
pip3 install \
    nodejs \
    ipympl
pip3 install --upgrade \
    jupyterlab

jupyter labextension install \
    @jupyter-widgets/jupyterlab-manager \
    jupyter-matplotlib

jupyter nbextension install --py --symlink ipympl
jupyter nbextension enable --py ipympl
