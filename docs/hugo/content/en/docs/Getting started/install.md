---
title: "Install"
linkTitle: "Install"
date: 2017-01-05
---

DPsim is a Python module / C++ library for complex power system
simulation. As a Python module, the easiest way to get to know DPsim is
via [Jupyter Notebooks](https://jupyter-notebook-beginner-guide.readthedocs.io/en/latest/).

## Docker

First, you need to install [Docker](https://docs.docker.com/install/).
Then, you could either build a docker image by yourself as described in the build instructions or download a prepared image from Docker Hub as described in the following.

To start a Jupyter session, run a DPsim Docker container

		$ docker run -p 8888:8888 sogno/dpsim

And access the session by opening the following link: http://localhost:8888/lab?token=3adaa57df44cea75e60c0169e1b2a98ae8f7de130481b5bc

## Python

**Currently, the pypi packages are not maintained. Until we have updated the packages, please use the docker installation.**

### Prerequisites

First, you need to make sure that Python is installed and your version is compatible.
An easy way to install Python and all required packages is the [Anaconda distribution](https://www.anaconda.com/).
To get started, install the latest installer for Python 3.x from the [downloads section](https://www.anaconda.com/download/).
Then, run the Anaconda Prompt and create a new conda environment:

	$ conda create -n dpsim python=3.6

After creating the environment you need to make sure that it is activated.
The current environment is displayed at the beginning of the command line in brackets.
It should read *"(dpsim)..."*.
In case it is not activated, run:

	$ activate dpsim


### Pip Package Installation

Then, DPsim can be installed as a Python module:

	$ pip install dpsim


## From Source

To build and install DPsim from the source files, please refer to the build section.
