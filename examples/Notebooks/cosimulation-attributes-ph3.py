#!/usr/bin/env python
# coding: utf-8

# Weakly coupled Co-simulation
# ================
# 
# This guides shows how to implement a simplified weakly-coupled co-simulation in emtsim
# 
# Test circuit
# --------------------
# 
# As a first simple, test we will simulate a small static network. The network consists of 4 nodes and 4 elements:
# 
# | Component | Type              | Python Class                              | Node A | Node B | Paramter |
# | :---------| :----             | :---------------------------------------- | :----- | :----- | :------- |
# | r_1       | Source resistance | `dpsimpy.emt.ph1.Resistor`                 | 0      | GND    | 0.1 Ohm  |
# | c_1       | Line capacitance  | `dpsimpy.emt.ph1.Capacitor`                | 0      | GND    | 1 Farad  |
# | c_2       | Line capacitance  | `dpsimpy.emt.ph1.Capacitor`                | 1      | GND    | 1 Farad  |
# | r_line    | Line resistance   | `dpsimpy.emt.ph1.Resistor`                 | 1      | 0      | 0.1 Ohm  |
# | r_load    | Load              | `dpsimpy.emt.ph1.Resistor`                 | 1      | GND    | 1 Ohm    |

# Before we can start, we must import the DPsim Python module.
# We also add `emt` as an alias for the dynamic phasor components.

# In[ ]:


import dpsimpy
import numpy as np


# Next, we can define the model by creating a couple of components.
# Each component is identified by a name which is passed as the first argument.
# Following arguments are used to define the topology by assigning the component to a specific node / bus or to pass parameters.

# In[ ]:


# Nodes
gnd = dpsimpy.emt.SimNode.gnd

n0 = dpsimpy.emt.SimNode("n0")
n1  = dpsimpy.emt.SimNode("n1")
n2  = dpsimpy.emt.SimNode("n2")
n3 = dpsimpy.emt.SimNode("n3")

v_s_1 = dpsimpy.emt.ph3.VoltageSource("v_s_1")
v_s_1_c = complex(2, 0)
v_s_1.set_parameters(V_ref=dpsimpy.Math.single_phase_variable_to_three_phase(v_s_1_c), f_src=50)

v_s_2 = dpsimpy.emt.ph3.VoltageSource("v_s_2")
v_s_2_c = complex(np.sqrt(2)/2, np.sqrt(2)/2)
v_s_2.set_parameters(V_ref=dpsimpy.Math.single_phase_variable_to_three_phase(v_s_2_c), f_src=50)

r_1 = dpsimpy.emt.ph3.Resistor("r_1")
r_1.set_parameters([0.1])
r_line = dpsimpy.emt.ph3.Resistor("r_line")
r_line.set_parameters([0.1])
c_1 = dpsimpy.emt.ph3.Capacitor("c_1")
c_1.set_parameters([1])
c_2 = dpsimpy.emt.ph3.Capacitor("c_2")
c_2.set_parameters([1])
r_load = dpsimpy.emt.ph3.Resistor("r_load")
r_load.set_parameters([1])

# Initial conditions
# n1.set_initial_voltage(0 * dpsimpy.PEAK1PH_TO_RMS3PH)
# n2.set_initial_voltage(0 * dpsimpy.PEAK1PH_TO_RMS3PH)

# Connections
v_s_1.connect([gnd, n0])
r_1.connect([n0, n1])     

v_s_2.connect([gnd, n3])
r_load.connect([n2, n3])
    
r_line.connect([n1, n2])
c_1.connect([n1, gnd])
c_2.connect([n2, gnd])


# Next, we have to create a simulation object:

# In[ ]:


sys = dpsimpy.SystemTopology(50, [ gnd, n0, n1, n2, n3 ], [ v_s_1, r_1, r_line, c_1, c_2, r_load, v_s_2 ])


# We can also visualize the system topology:

# In[ ]:


sys


# Finally, we can start the simulation and wait for its completion:

# In[ ]:


time_step = 1e-5
final_time = 1e-4

sim = dpsimpy.Simulation("EMTCosim", loglevel=dpsimpy.LogLevel.debug)
sim.set_domain(dpsimpy.Domain.EMT)
sim.set_system(sys)
sim.set_time_step(time_step)
sim.set_final_time(final_time)

log = dpsimpy.Logger("EMTCosim")
for i in range(1, len(sys.nodes)):
    log.log_attribute("v" + str(i), "v", sys.nodes[i])

# log.log_attribute("r_line.I", "i_intf", sys.component("r_line"))

sim.add_logger(log)
    
sim.run()


# Next, we run the co-simulation

# In[ ]:


import subprocess
import numpy as np

H_v = np.array([1e-4, 2e-4, 4e-4, 0.001, 0.003, 0.01])
H_v_legends = ['1e-4', '2e-4', '4e-4', '1e-3', '3e-3', '1e-2']

i=0
for H in H_v:
    process1 = subprocess.Popen(["python3", "../Python/Attributes/emt-cosim-attributes_polynomial.py", "-t", str(time_step), "-e", "1", "-H", str(H), "-p", "EMTCosimAttributes_zoh_" + H_v_legends[i], "--num-vs", str(num_vs)], stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    output1, error1 = process1.communicate()
    process2 = subprocess.Popen(["python3", "../Python/Attributes/emt-cosim-attributes_polynomial.py", "-t", str(time_step), "-e", "1", "-H", str(H), "-i", "linear", "-p", "EMTCosimAttributes_linear_" + H_v_legends[i], "--num-vs", str(num_vs)], stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    output2, error2 = process2.communicate()

    # Print output and error for debugging
    print("Output 1:", output1)
    print("Error 1:", error1)
    print("Output 2:", output2)
    print("Error 2:", error2)

    # Check return codes
    # assert process1.returncode == 0, f"Subprocess 1 failed for H = {H}"
    # assert process2.returncode == 0, f"Subprocess 2 failed for H = {H}"

    i+=1

# TODO: Verify why the villas co-simulation does not log outputs correctly using Popen
# process = subprocess.Popen(["python3", "../villas/emt-cosim-villas.py"], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)


# Results can be analyzed and plotted by the `villas.dataprocessing` package:

# In[ ]:


get_ipython().run_line_magic('matplotlib', 'inline')
get_ipython().run_line_magic('config', "InlineBackend.figure_format = 'svg'")
get_ipython().run_line_magic('config', "InlineBackend.rc = {'font.size': 10, 'figure.figsize': (6.0, 4.0), 'figure.facecolor': 'white', 'savefig.dpi': 72, 'figure.subplot.bottom': 0.125, 'figure.edgecolor': 'white'}")

import matplotlib.pyplot as plt
import villas.dataprocessing.plottools as pt
import villas.dataprocessing.readtools as rt
import villas.dataprocessing.timeseries as ts
# %matplotlib widget

results = rt.read_timeseries_dpsim('logs/EMTCosim.csv')

results_emt = []
for series in results:
    results_emt.append(results[series])

results_attributes_zoh = {}
for H_v_leg in H_v_legends:
    results_attributes_zoh[H_v_leg] = rt.read_timeseries_dpsim('logs/EMTCosimAttributes_zoh_' + H_v_leg + '_S1/S1.csv')
    
results_attributes_linear_S1 = {}
for H_v_leg in H_v_legends:
    results_attributes_linear_S1[H_v_leg] = rt.read_timeseries_dpsim('logs/EMTCosimAttributes_linear_' + H_v_leg + '_S1/S1.csv')

# results_attributes_linear_S2 = {}
# for H_v_leg in H_v_legends:
#     results_attributes_linear_S2[H_v_leg] = rt.read_timeseries_dpsim('logs/EMTCosimAttributes_linear_' + H_v_leg + '_S2/S2.csv')

# Get logs, if the co-simulation was executed from this script
# results_villas1 = rt.read_timeseries_dpsim('logs/EMTCosimVILLAS1/EMTCosimVILLAS1.csv')
# results_villas2 = rt.read_timeseries_dpsim('logs/EMTCosimVILLAS2/EMTCosimVILLAS2.csv')

# If the villas co-simulation was not executed from this script, read the logs as follows
# This assumes that you executed the script examples/villas/emt-cosim-villas.py previously
# results_villas1 = rt.read_timeseries_dpsim('../../logs/EMTCosimVILLAS1/EMTCosimVILLAS1.csv')
# results_villas2 = rt.read_timeseries_dpsim('../../logs/EMTCosimVILLAS2/EMTCosimVILLAS2.csv')

# Get logs from the co-simulation with Matlab
# results_matlab = rt.read_timeseries_dpsim('logs/EMTCosimUDP1/EMTCosimUDP1.csv')

# results_emt_matlab = []
# for series in results_matlab:
#     results_emt_matlab.append(results_matlab[series])

results_emt_attributes_zoh = {}

for k, results_att_list in results_attributes_zoh.items():
    results_emt_attributes_zoh[k] = []
    for series in results_att_list:
        pt.set_timeseries_labels(results_att_list[series], series + ', H=' + k + ' - ZOH')
        results_emt_attributes_zoh[k].append(results_att_list[series])
        
results_emt_attributes_linear_S1 = {}

for k, results_att_list in results_attributes_linear_S1.items():
    results_emt_attributes_linear_S1[k] = []
    for series in results_att_list:
        pt.set_timeseries_labels(results_att_list[series], series + ', H=' + k + ' - Linear')
        results_emt_attributes_linear_S1[k].append(results_att_list[series])

results_emt_attributes_linear_S2 = {}

for k, results_att_list in results_attributes_linear_S1.items():
    results_emt_attributes_linear_S2[k] = []
    for series in results_att_list:
        pt.set_timeseries_labels(results_att_list[series], series + ', H=' + k + ' - Linear')
        results_emt_attributes_linear_S2[k].append(results_att_list[series])
    
# results_emt_villas_1 = []
# for series in results_villas1:
#     results_emt_villas_1.append(results_villas1[series])
    
# results_emt_villas_2 = []
# for series in results_villas2:
#     results_emt_villas_2.append(results_villas2[series])

if num_vs == 0:
    pt.set_timeseries_labels(results_emt[1], results_emt[1].label + ' - Monolithic')
    pt.plot_timeseries('Co-simulation results - V1', results_emt[1])

    pt.set_timeseries_labels(results_emt[2], results_emt[2].label + ' - Monolithic')
    pt.plot_timeseries('Co-simulation results - V2', results_emt[2])
else:
    pt.set_timeseries_labels(results_emt[2], results_emt[2].label + ' - Monolithic')
    pt.plot_timeseries('Co-simulation results - V1', results_emt[2])

    pt.set_timeseries_labels(results_emt[3], results_emt[3].label + ' - Monolithic')
    pt.plot_timeseries('Co-simulation results - V2', results_emt[3])

pt.set_timeseries_labels(results_emt[0], results_emt[0].label + ' - Monolithic')
pt.plot_timeseries('Co-simulation results - i_intf', results_emt[0])

# for k, results_att_list in results_emt_attributes_zoh.items():
#     pt.plot_timeseries('Co-simulation results - V1', results_att_list[2], '--')
#     pt.plot_timeseries('Co-simulation results - V2', results_att_list[3], '--')
#     pt.plot_timeseries('Co-simulation results - i_intf', results_att_list[0], '--')

for k, results_att_list in results_emt_attributes_linear_S1.items():
    pt.plot_timeseries('Co-simulation results - V1', results_att_list[2], '--')
    pt.plot_timeseries('Co-simulation results - V2', results_att_list[3], '--')
    pt.plot_timeseries('Co-simulation results - i_intf', results_att_list[1], '--')

# for k, results_att_list in results_emt_attributes_linear_S2.items():
#     pt.plot_timeseries('Co-simulation results - V2', results_att_list[3], '-.')
    
# pt.plot_timeseries('Co-simulation results', results_emt_villas_1[3], '--')
# pt.plot_timeseries('Co-simulation results', results_emt_villas_1[4], '--')
# pt.plot_timeseries('Co-simulation results', results_emt_villas_2[2], '--')

# pt.plot_timeseries('Co-simulation results', results_emt_matlab[3], '-.')
# pt.plot_timeseries('Co-simulation results', results_emt_matlab[4], '-.')

plt.xlabel('t [s]')
# plt.ylabel('Voltage [V]')
plt.grid()
plt.show()


# ## Obtain the co-simulation errors

# In[ ]:


global_error_zoh = []
global_error_linear = []

for i in range(0, len(H_v_legends)):    
    H = H_v[i]
    
    m = int(H/time_step)
    
    if num_vs == 0:
        ts_1_m = results_emt[1]
        ts_2_m = results_emt[2]

        ts_1_zoh = results_emt_attributes_zoh[H_v_legends[i]][1]
        ts_2_zoh = results_emt_attributes_zoh[H_v_legends[i]][2]
        
        ts_1_linear = results_emt_attributes_linear_S1[H_v_legends[i]][1]
        ts_2_linear = results_emt_attributes_linear_S1[H_v_legends[i]][2]
    else:
        ts_1_m = results_emt[2]
        ts_2_m = results_emt[3]
    
        ts_1_zoh = results_emt_attributes_zoh[H_v_legends[i]][2]
        ts_2_zoh = results_emt_attributes_zoh[H_v_legends[i]][3]
        
        ts_1_linear = results_emt_attributes_linear_S1[H_v_legends[i]][2]
        ts_2_linear = results_emt_attributes_linear_S1[H_v_legends[i]][3]
    
    v_a = np.array([ts_1_m.values[::m], ts_2_m.values[::m]])
    v_zoh = np.array([ts_1_zoh.values[::m], ts_2_zoh.values[::m]])
    v_linear = np.array([ts_1_linear.values[::m], ts_2_linear.values[::m]])
    
    gloal_error_zoh_t = np.max(np.linalg.norm(v_a - v_zoh, axis=0))
    
    global_error_zoh.append(gloal_error_zoh_t)
    global_error_linear.append(np.max(np.linalg.norm(v_a - v_linear, axis=0)))
    
plt.figure()
plt.plot(np.log10(H_v), np.log10(global_error_zoh), 'b', label='ZOH')
plt.plot(np.log10(H_v), np.log10(global_error_zoh), 'bo')
plt.plot(np.log10(H_v), np.log10(np.multiply(H_v, 10*0.012)), 'k', label=r'$\mathcal{O}(H)$')
plt.plot(np.log10(H_v), np.log10(global_error_linear), 'r', label='Linear')
plt.plot(np.log10(H_v), np.log10(global_error_linear), 'ro')
plt.plot(np.log10(H_v), np.log10(np.multiply(H_v**2, 10**1.5)), '--k', label=r'$\mathcal{O}(H^2)$')
plt.xlabel('')
# TODO: Fix log scale
# plt.xscale("log")
# plt.yscale("log")
plt.grid()
plt.legend()

plt.tight_layout()
plt.show()
    
    


# In[ ]:


get_ipython().system('cat logs/EMTCosim.log')


# 
