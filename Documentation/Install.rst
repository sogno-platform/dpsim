Installation
============

Prerequisites
-------------

First, you need to make sure that Python is installed and your version is compatible.
Currently, DPsim is built for **Python 3.6**. 
An easy way to install Python and all required packages is the `Anaconda distribution <https://www.anaconda.com/>`_.

To get started, install the latest installer for Python 3.x from the `downloads section <https://www.anaconda.com/download/>`_.
Then, run the Anaconda Prompt and create a new conda environment:

::

	conda create -n dpsim python=3.6
	
DPsim Python Packages Installation
----------------------------------

After creating the environment you need to make sure that it is activated. 
The current environment is displayed at the beginning of the command line in brackets.
It should read *"(dpsim)..."*.
In case it is not activated, run:

::

	activate dpsim

Then, DPsim can be easily installed as a Python module by running:

::

	pip install dpsim

Usage with PyCharm
------------------

To be able to use DPsim in a new PyCharm project, you need to select the Python interpreter of your new dpsim environment.

- First, go to *File->Settings->Project:Python->Project Interpreter*.
- Then, click on the gearwheel next to the Project Interpreter field and select *Add Local*.
- If you chose to install Anaconda for your user, the python interpreter of the dpsim environment should be located at *C:\\Users\\[user name]\\AppData\\Local\\Continuum\\anaconda3\\envs\\dpsim\\python.exe*.
- Click *Ok* and check if the correct interpreter is shown in the *External Libraries* tree view in the left pane.

