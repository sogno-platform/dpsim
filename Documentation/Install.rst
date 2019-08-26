Installation
============

DPsim is a Python module / C++ library for complex power system
simulation. As a Python module, the easiest way to get to know DPsim is
via `Jupyter Notebooks <https://jupyter-notebook-beginner-guide.readthedocs.io/en/latest/>`__.

Docker
------

1. First, you need to install `Docker <https://docs.docker.com/install/>`_.

2. To start a Jupyter session, run a DPsim Docker container

::

   $ docker run -p 8888:8888 rwthacs/dpsim

And access the session by opening the following link: http://localhost:8888/lab/tree/Index.ipynb?token=3adaa57df44cea75e60c0169e1b2a98ae8f7de130481b5bc

Python
------

**Currently, the DPsim Python packages are broken. Until we have updated the packages, please use the docker installation.** 

Prerequisites
^^^^^^^^^^^^^

First, you need to make sure that Python is installed and your version is compatible.
DPsim is built for **Python 3.5, 3.6 and 3.7**. 
An easy way to install Python and all required packages is the `Anaconda distribution <https://www.anaconda.com/>`_.

To get started, install the latest installer for Python 3.x from the `downloads section <https://www.anaconda.com/download/>`_.
Then, run the Anaconda Prompt and create a new conda environment:

.. code-block:: bash

	conda create -n dpsim python=3.6

After creating the environment you need to make sure that it is activated. 
The current environment is displayed at the beginning of the command line in brackets.
It should read *"(dpsim)..."*.
In case it is not activated, run:

.. code-block:: bash

	activate dpsim
	

Pip Package Installation
^^^^^^^^^^^^^^^^^^^^^^^^

Then, DPsim can be easily installed as a Python module by running for the latest stable release:

.. code-block:: bash

	pip install dpsim

or for the latest nightly version:

.. code-block:: bash

	pip install --extra-index-url https://packages.fein-aachen.org/python/simple dpsim

From Source
-----------

To build and install DPsim from the source files, please refer to the section :doc:`Build </Build>`.