#!/usr/bin/python3

import subprocess
import asyncio
import dpsimpy
import glob
import time
import logging

import villas.dataprocessing.readtools as rt
import villas.dataprocessing.plottools as pt
from villas.dataprocessing.timeseries import TimeSeries as ts
import matplotlib.pyplot as plt


def decouple_line(sys, line, node1, node2):
    orig_line = system.component(line)
    sys.remove_component(line)
    R_line = orig_line.attr("R_series").get()
    L_line = orig_line.attr("L_series").get()
    C_line = orig_line.attr("C_parallel").get()

    line = dpsimpy.signal.DecouplingLine(
        "dline_" + node1 + "_" + node2, dpsimpy.LogLevel.info
    )
    line.set_parameters(sys.node(node1), sys.node(node2), R_line, L_line, C_line)

    # TODO: Probably interestingn to add the line this way
    # sys.add_decoupling_line(line, node1, node2, R_line, L_line, C_line)

    sys.add_component(line)
    sys.add_components(line.get_line_components())


def do_sim(name, system):
    logger = dpsimpy.Logger(name)

    logger.log_attribute("BUS5.V", "v", system.node("BUS5"))
    logger.log_attribute("BUS6.V", "v", system.node("BUS6"))
    logger.log_attribute("BUS8.V", "v", system.node("BUS8"))

    sim = dpsimpy.Simulation(name, dpsimpy.LogLevel.debug)
    sim.set_system(system)
    sim.set_time_step(0.0001)
    sim.set_final_time(0.1)
    # sim.do_steady_state_init(False)
    sim.do_split_subnets(True)
    sim.do_init_from_nodes_and_terminals(True)
    sim.add_logger(logger)
    sim.run()


name = "WSCC-9bus"

dpsim_root_dir = (
    subprocess.Popen(["git", "rev-parse", "--show-toplevel"], stdout=subprocess.PIPE)
    .communicate()[0]
    .rstrip()
    .decode("utf-8")
)

files = glob.glob(dpsim_root_dir + "/build/_deps/cim-data-src/WSCC-09/WSCC-09/*.xml")
reader_normal = dpsimpy.CIMReader(name)
system = reader_normal.loadCIM(
    60,
    files,
    dpsimpy.Domain.DP,
    dpsimpy.PhaseType.Single,
    dpsimpy.GeneratorType.IdealVoltageSource,
)
do_sim("normal", system)

reader_decoupled = dpsimpy.CIMReader(name)
system = reader_decoupled.loadCIM(
    60,
    files,
    dpsimpy.Domain.DP,
    dpsimpy.PhaseType.Single,
    dpsimpy.GeneratorType.IdealVoltageSource,
)
decouple_line(system, "LINE75", "BUS5", "BUS7")
# decouple_line(system, 'LINE78', 'BUS7', 'BUS8')
decouple_line(system, "LINE64", "BUS6", "BUS4")
decouple_line(system, "LINE89", "BUS8", "BUS9")
do_sim("decoupled", system)

normal_dp = rt.read_timeseries_dpsim("logs/normal.csv")
normal_emt = ts.frequency_shift_list(normal_dp, 60)

decoupled_dp = rt.read_timeseries_dpsim("logs/decoupled.csv")
decoupled_emt = ts.frequency_shift_list(decoupled_dp, 60)

for i, v in enumerate([5, 6, 8]):
    varname = "BUS" + str(v) + ".V_shift"
    pt.set_timeseries_labels(normal_emt[varname], varname + " normal")
    pt.set_timeseries_labels(decoupled_emt[varname], varname + " decoupled")
    pt.plot_timeseries(i + 1, normal_emt[varname])
    pt.plot_timeseries(i + 1, decoupled_emt[varname])
    plt.title("WSCC-09 with 3 lines replaced by decoupling equivalents")
    plt.xlabel("Time [s]")
    plt.ylabel("Voltage [V]")

plt.show()
