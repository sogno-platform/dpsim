Installation
============

Prerequisites
-------------

First, you need to make sure that Python is installed and your version is compatible.
Currently, DPsim is built for **Python 3.6**. 
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
	

PyPI Package Installation
-------------------------

Then, DPsim can be easily installed as a Python module by running:

.. code-block:: bash

	pip install dpsim
