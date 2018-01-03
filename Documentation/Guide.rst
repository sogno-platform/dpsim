Quickstart Guide
================

This guides demonstrates the basic usage of DPsim as a Python module.

(Please note that DPsim also provides a C++ API.)


The first simulation
--------------------

As a first simple, test we will simulate the following static network:

.. image:: ../Figures/Example1.svg
	:align: center

The network consists of 4 nodes and 4 elements:

=========  ===============  =============================== ====== ====== ========
Component  Type             Python Class                    Node A Node B Paramter
=========  ===============  =============================== ====== ====== ========
v_s        Voltage Source   :py:class:`dpsim.VoltSourceRes` 1      0      10 kV
r_line     Line Resistance  :py:class:`dpsim.Resistor`      1      2      1 Ohm
l_line     Line Inductance  :py:class:`dpsim.Inductor`      2      3      1 Henry
r_load     Load             :py:class:`dpsim.Resistor`      3      0      1 kOhm
=========  ===============  =============================== ====== ====== ========

Before we can start, we must import the DPsim Python module:

.. code-block:: python

	import dpsim

Next, we can define the model by creating a couple of components.
Each component is identified by a name which is passed as the first argument.
Following arguments are used to define the topology by assigning the component to a specific node / bus or to pass parameters.

.. code-block:: python

	components = [
		dpsim.VoltSourceRes("v_s", 1, 0, 10000+0j, 1),
		dpsim.Resistor("r_line", 1, 2, 1),
		dpsim.Inductor("l_line", 2, 3, 1),
		dpsim.Resistor("r_load", 3, 0, 1000)
	]


Next, we have to create a simulation object:

.. code-block:: python

	sim = dpsim.Simulation(
		components,
		timestep = 1e-3,
		duration = 0.03,
		 log = "Log_Example1.log",
		llog = "LeftVector_Example1.csv",
		rlog = "RightVector_Example1.csv"
	)

Finally, we can start the simulation and wait for its completion:

.. code-block:: python

	sim.start()
	sim.wait()

Results are written directly to the log files specified in the simulation object.

We can use matplotlib to plot the results:

.. code-block:: python

	import pandas
	import matplotlib.pyplot as plt
	
	results = pandas.read_csv('RightVector_Example1.csv')
	
	plt.plot(results)
	plot.show()


.. image:: ../Figures/Example1_prelim_results.svg
	:align: center