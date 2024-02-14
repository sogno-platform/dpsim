#!/usr/bin/python3

import asyncio
import dpsim
import glob
import time
import logging
from dpsim.Event import Event

import villas.dataprocessing.readtools as rt
import villas.dataprocessing.plottools as pt
from villas.dataprocessing.timeseries import TimeSeries as ts
import matplotlib.pyplot as plt

def multiply_decoupled(sys, num_copies, nodes, resistance, inductance, capacitance):
    sys.multiply(num_copies)
    counter = 0
    for orig_node in nodes:
        node_names = [orig_node]
        node_names += [orig_node + '_' + str(i) for i in range(2, num_copies+2)]
        node_names.append(orig_node)
        for i in range(0, num_copies+1):
            sys.add_decoupling_line('dline' + str(counter), sys.nodes[node_names[i]], sys.nodes[node_names[i+1]], resistance, inductance, capacitance)
            counter += 1

def multiply_coupled(sys, num_copies, nodes, resistance, inductance, capacitance):
    gnd = dpsim.dp.Node.GND()
    sys.multiply(num_copies)
    counter = 0
    for orig_node in nodes:
        node_names = [orig_node]
        node_names += [orig_node + '_' + str(i) for i in range(2, num_copies+2)]
        node_names.append(orig_node)
        for i in range(0, num_copies+1):
            # TODO lumped resistance?
            rl_node = dpsim.dp.Node('N_add_' + str(counter))
            res = dpsim.dp.ph1.Resistor('R_' + str(counter))
            res.R = resistance
            ind = dpsim.dp.ph1.Inductor('L_' + str(counter))
            ind.L = inductance
            cap1 = dpsim.dp.ph1.Capacitor('C1_' + str(counter))
            cap1.C = capacitance / 2
            cap2 = dpsim.dp.ph1.Capacitor('C2_' + str(counter))
            cap2.C = capacitance / 2

            sys.add_node(rl_node)
            res.connect([sys.nodes[node_names[i]], rl_node])
            ind.connect([rl_node, sys.nodes[node_names[i+1]]])
            cap1.connect([sys.nodes[node_names[i]], gnd])
            cap2.connect([sys.nodes[node_names[i+1]], gnd])
            counter += 1

            sys.add_component(res)
            sys.add_component(ind)
            sys.add_component(cap1)
            sys.add_component(cap2)

def do_sim(name, system):
    logger = dpsim.Logger(name)
    for i in range(0, 6):
        logger.log_attribute(system.nodes['BUS' + str(i+4)], 'v')

    sim = dpsim.Simulation(name, system, timestep=0.0001, duration=0.1, init_steady_state=False, pbar=False, split_subnets=True)
    sim.set_scheduler('thread_level', threads=4)
    sim.add_logger(logger)
    sim.run()

name = 'WSCC-9bus'
files = glob.glob('../dpsim/examples/CIM/WSCC-09_RX/*.xml')

system = dpsim.load_cim(name, files, frequency=60)
do_sim('normal', system)

system = dpsim.load_cim(name, files, frequency=60)
multiply_coupled(system, 1, ['BUS5', 'BUS6', 'BUS8'], 100, 0.16, 1e-6)
do_sim('coupled', system)

system = dpsim.load_cim(name, files, frequency=60)
multiply_decoupled(system, 1, ['BUS5', 'BUS6', 'BUS8'], 100, 0.16, 1e-6)
do_sim('decoupled', system)

normal_dp = rt.read_timeseries_dpsim("Logs/normal.csv")
normal_emt = ts.frequency_shift_list(normal_dp, 60)

coupled_dp = rt.read_timeseries_dpsim("Logs/coupled.csv")
coupled_emt = ts.frequency_shift_list(coupled_dp, 60)

decoupled_dp = rt.read_timeseries_dpsim("Logs/decoupled.csv")
decoupled_emt = ts.frequency_shift_list(decoupled_dp, 60)

for i in range(0, 6):
    varname = 'BUS' + str(i+4) + '.v'
    # varname = 'BUS4.v'
    i = 0
    pt.set_timeseries_labels(normal_emt[varname], varname + ' normal')
    pt.set_timeseries_labels(coupled_emt[varname], varname + ' coupled')
    pt.set_timeseries_labels(decoupled_emt[varname], varname + ' decoupled')
    pt.plot_timeseries(i+1, normal_emt[varname])
    pt.plot_timeseries(i+1, coupled_emt[varname])
    pt.plot_timeseries(i+1, decoupled_emt[varname])
    plt.title('WSCC-09_RX with copy connected by lines (100 Ohm, 0.16 H, 1e-6 F)')
    plt.xlabel('Time [s]')
    plt.ylabel('Voltage [V]')

plt.show()
