#!/usr/bin/python3

import asyncio
import dpsim
import glob
import time
import logging
import socket
import os.path
from dpsim.Event import Event

import matplotlib

matplotlib.use("SVG")
import matplotlib.pyplot as plt

name = "WSCC-9bus_RX_dyn"
files = glob.glob("../dpsim/examples/CIM/WSCC-09_RX_Dyn/*.xml")

threads = [1, 2, 4, 8, 16]
sizes = range(1, 32)
scheduler = "thread_level"
reps = 5

for size in sizes:
    filename = "measurements_" + str(size) + ".txt"
    if not os.path.isfile(filename):
        system = dpsim.load_cim(name, files, frequency=60)
        if size > 1:
            system.multiply(size - 1)
        sim = dpsim.Simulation(
            name,
            system,
            timestep=0.001,
            duration=0.5,
            init_steady_state=True,
            pbar=False,
            split_subnets=True,
            log_level=0,
        )
        sim.set_scheduler("sequential", out_measurement_file=filename)
        sim.run()

step_times = {}
for n in threads:
    step_times[n] = []
    for size in sizes:
        rep_times = []
        for i in range(0, reps):
            system = dpsim.load_cim(name, files, frequency=60)
            if size > 1:
                system.multiply(size - 1)
            sim = dpsim.Simulation(
                name,
                system,
                timestep=0.001,
                duration=0.5,
                init_steady_state=True,
                pbar=False,
                split_subnets=True,
                log_level=0,
            )
            filename = "measurements_" + str(size) + ".txt"
            sim.set_scheduler(scheduler, threads=n, in_measurement_file=filename)
            sim.run()
            rep_times.append(sim.avg_step_time)
        step_times[n].append(sum(rep_times) / len(rep_times))

for n in threads:
    plt.plot(sizes, step_times[n], label=(str(n) + " threads"))

plt.xlabel("System copies (with splitting)")
plt.ylabel("Wall time per step [s]")

hostname = socket.gethostname()
plt.title("{} (using meas), on {}, {} reps average".format(name, hostname, reps))
plt.legend()
plt.savefig("mult_split_{}.svg".format(hostname))
