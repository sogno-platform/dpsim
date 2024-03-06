#!/usr/bin/python3

import glob
import sys

from meas_utils import *


def sched_nthreads(instance, threads, schedulers, size):
    times = {}
    sigmas = {}
    for scheduler in schedulers:
        times[scheduler] = []
        sigmas[scheduler] = []
        for n in threads:
            sched_name, args = map_scheduler(scheduler, size)
            if "in_measurement_file" in list(args.keys()):
                do_meas(instance, size)
            args["threads"] = n
            avg, sigma = instance.do_sim(
                scheduler=sched_name, scheduler_args=args, size=size
            )
            times[scheduler].append(avg)
            sigmas[scheduler].append(sigma)
    return Measurement(instance, threads, "threads", times, sigma=sigmas)


if __name__ == "__main__":
    threads = range(1, 11)
    schedulers = [
        #    'sequential',
        "omp_level",
        "thread_level",
        "thread_level meas",
        "thread_list",
        "thread_list meas",
    ]
    size = 1
    # size = 20

    name = "sched_nthreads"
    if len(sys.argv) > 1:
        name = sys.argv[1]

    instance = SimInstance(
        name, glob.glob("../dpsim/examples/CIM/WSCC-09_RX_Dyn/*.xml"), 60.0, 50
    )
    instance.sim_args = {
        "timestep": 0.0001,
        "duration": 0.1,
        "init_steady_state": False,
        "pbar": False,
        "split_subnets": True,
        "log_level": 0,
    }
    instance.copy_settings = {
        "decouple": False,
        "nodes": ["BUS5", "BUS6", "BUS8"],
        "resistance": 100,
        "inductance": 0.16,
        "capacitance": 1e-6,
    }

    meas = sched_nthreads(instance, threads, schedulers, size)
    meas.save()
    check_numa()
