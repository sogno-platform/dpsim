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


def decouple_line(sys, line, node1, node2):
    orig_line = system.components[line]
    sys.remove_component(line)
    sys.add_decoupling_line(
        line,
        sys.nodes[node1],
        sys.nodes[node2],
        orig_line.R_series,
        orig_line.L_series,
        orig_line.C_parallel,
    )


def do_sim(name, system):
    logger = dpsim.Logger(name)
    logger.log_attribute(system.nodes["BUS5"], "v")
    logger.log_attribute(system.nodes["BUS6"], "v")
    logger.log_attribute(system.nodes["BUS8"], "v")

    sim = dpsim.Simulation(
        name,
        system,
        timestep=0.0001,
        duration=0.1,
        init_steady_state=False,
        pbar=False,
        split_subnets=True,
    )
    sim.add_logger(logger)
    sim.run()


name = "WSCC-9bus"

files = glob.glob("../dpsim/examples/CIM/WSCC-09/*.xml")
system = dpsim.load_cim(name, files, frequency=60)
do_sim("normal", system)

system = dpsim.load_cim(name, files, frequency=60)
decouple_line(system, "LINE75", "BUS5", "BUS7")
# decouple_line(system, 'LINE78', 'BUS7', 'BUS8')
decouple_line(system, "LINE64", "BUS6", "BUS4")
decouple_line(system, "LINE89", "BUS8", "BUS9")
do_sim("decoupled", system)

normal_dp = rt.read_timeseries_dpsim("Logs/normal.csv")
normal_emt = ts.frequency_shift_list(normal_dp, 60)

decoupled_dp = rt.read_timeseries_dpsim("Logs/decoupled.csv")
decoupled_emt = ts.frequency_shift_list(decoupled_dp, 60)

for i, v in enumerate([5, 6, 8]):
    varname = "BUS" + str(v) + ".v"
    pt.set_timeseries_labels(normal_emt[varname], varname + " normal")
    pt.set_timeseries_labels(decoupled_emt[varname], varname + " decoupled")
    pt.plot_timeseries(i + 1, normal_emt[varname])
    pt.plot_timeseries(i + 1, decoupled_emt[varname])
    plt.title("WSCC-09 with 3 lines replaced by decoupling equivalents")
    plt.xlabel("Time [s]")
    plt.ylabel("Voltage [V]")

plt.show()
