
# coding: utf-8

# # Dynamic WSCC 9-bus System with Switch Event

# **Authors**:
#  - Markus Mirz <mmirz@eonerc.rwth-aachen.de>
#  - Steffen Vogel <stvogel@eoner.rwth-aachen.de>
#
# This Jupyter Notebook shows a simple dynamic phasor simulation of the WSCC-9bus benchmark model.
# The grid data is loaded from CIM-XML files, while simulation results are stored in CSV files and plotted via Matplotlib

# In[1]:

import dpsim
from dpsim.Event import Event
import glob
import asyncio
import matplotlib.pyplot as plt


# ### Loading Grid Topology from CIM-XML Model

# In[2]:


name = 'WSCC-9bus_dyn_switch'
files = glob.glob('dpsim/Examples/CIM/WSCC-09_RX_Dyn/*.xml')
print(files)
system = dpsim.load_cim(name, files, frequency=60)

## Switch
sw = dpsim.dp.ph1.Switch("Switch")

sw.R_open = 1e9
sw.R_closed = 0.1
sw.is_closed = False

## Load
load = dpsim.dp.ph1.PQLoadCS("Switched Load")

load.V_nom = 230950
load.P = 30000000
load.Q = 0

## Short
res = dpsim.dp.ph1.Resistor("Short")
res.R = 0.1

## Topology
bus9 = system.nodes["BUS6"]
#bus10 = dpsim.dp.Node("BUS10")
gnd = dpsim.dp.Node.GND()

#system.add_node(bus10)

#system.add_component(res)
#res.connect([bus10, gnd])

#system.add_component(load)
#load.connect([ bus10 ])

system.add_component(sw)
sw.connect([ bus9, gnd ])


# ### Running Simulation
#
# The actual simulation is done by the C++ DPsim solver. Python is just used for configuration, scripting and analysis

# In[ ]:


sim = dpsim.Simulation(name, system, timestep=0.0001, duration=2, init_steady_state=True, pbar=False)

system.components['GEN3'].inertia *= 2
sw.is_closed = False
sim.add_event(0.2, sw, 'is_closed', True)

print(system.components)
print(system.nodes)

logger = dpsim.Logger(name)
sim.add_logger(logger)
for i in range(1,4):
	logger.log_attribute(system.components['GEN%d' % i], 'w_r')
for node in system.nodes:
	logger.log_attribute(system.nodes[node], 'voltage')

sim.start()

import time
while (sim.state != 0):
    time.sleep(0.1)

print('Simulation done')

# ## Analysis
#
# ### Read log files and list all column names

# In[7]:


from villas.dataprocessing.dpsim import *
from villas.dataprocessing.plottools import *

ts = read_timeseries_dpsim('Logs/WSCC-9bus_dyn_switch.csv')


# ### Phasors at first time step

# In[6]:


phasors = get_node_voltage_phasors(ts)
for node, phasor in phasors.items():
    if 'voltage' in node:
        print(node + ': ' + str(phasor['abs'].values[0]) + '<' + str(phasor['phase'].values[0]))


# ### Phasors at last time step

# In[7]:


for node, phasor in phasors.items():
    if 'voltage' in node:
        print(node + ': ' + str(phasor['abs'].values[-1]) + '<' + str(phasor['phase'].values[-1]))


# ### Phasors at last time step in per unit

# In[8]:


nominal_voltages = {
    'BUS1.voltage': 16.5e3,
    'BUS2.voltage': 18e3,
    'BUS3.voltage': 13.8e3,
    'BUS4.voltage': 230e3,
    'BUS5.voltage': 230e3,
    'BUS6.voltage': 230e3,
    'BUS7.voltage': 230e3,
    'BUS8.voltage': 230e3,
    'BUS9.voltage': 230e3
}

plt.figure(1)
for node, nom_voltage in nominal_voltages.items():
    mag = phasors[node]['abs'].values[0] / nom_voltage
    pha = phasors[node]['phase'].values[0]
    print(node + ': ' + str(mag) + '<' + str(pha))
    plt.polar([0, pha / 180 * np.pi], [0, mag], marker='o', label=node)
plt.show()


# ### Plot node phases

# In[9]:


for i in range(1,9):
    plot_timeseries(20, phasors['BUS%d.voltage' % i]['phase'])


# ### Plot node voltages

# In[10]:


for i in range(4,9):
    plot_timeseries(10, phasors['BUS%d.voltage' % i]['abs'])
for i in range(1,4):
    plot_timeseries(11, phasors['BUS%d.voltage' % i]['abs'])
plt.xlim(0.0, 0.06)


# In[13]:


plot_timeseries(8, ts['GEN1.w_r'])
plot_timeseries(8, ts['GEN2.w_r'])
plot_timeseries(8, ts['GEN3.w_r'])


# In[11]:


plot_timeseries(8, ts['GEN1.w_r'])
plot_timeseries(8, ts['GEN2.w_r'])
plot_timeseries(8, ts['GEN3.w_r'])

