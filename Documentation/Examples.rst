Examples
========

The following examples have been provided as Jupyter Notebooks.
You can read them online or execute and modify them interactively on your local machine.

Use the notebooks on your local machine
---------------------------------------

*Please note that the DPsim source code is not yet publicy available.*

Jupyter
~~~~~~~

Please install jupyter by following `this guide`_.

.. _`this guide`: http://jupyter.readthedocs.io/en/latest/install.html

DPsim module + notebooks
~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

     $ git clone git@git.rwth-aachen.de:acs/core/simulation/dpsim.git
     $ cd dpsim
     $ sudo python3 setup.py install
     $ jupyter notebook --notebook-dir Documentation/Notebooks/

List of Notebooks
-----------------

.. toctree::

   Quickstart Guide <Notebooks/Quickstart Guide.ipynb>
   Rendering Network Topologies with Graphviz <Notebooks/Graphviz.ipynb>

List of old Notebooks
---------------------

Please note that many of these Jupyter Notebooks are outdated and not working.

.. toctree::
   :glob:

   Notebooks/*