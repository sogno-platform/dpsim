#!/usr/bin/python3

import glob
import sys

from meas_utils import *

def diakoptics_splitsize(instance, size, splits, scheduler):
    times = {}
    sigmas = {}
    for scheduler in schedulers:
        times[scheduler] = []
        sigmas[scheduler] = []
        for split in splits:
            sched_name, args = map_scheduler(scheduler, size)
            if 'in_measurement_file' in list(args.keys()):
                do_meas(instance, size)
            instance.copy_settings['splits'] = split
            time, sigma = instance.do_sim(scheduler=sched_name, scheduler_args=args, size=size)
            times[scheduler].append(time)
            sigmas[scheduler].append(sigma)
    return Measurement(instance, splits, 'splits', times, sigma=sigmas)

if __name__ == '__main__':
    name = 'diakoptics_splitsize'
    if len(sys.argv) > 1:
        name = sys.argv[1]

    instance = SimInstance(name, glob.glob('../dpsim/examples/CIM/WSCC-09_RX_Dyn/*.xml'), 60.0, 10)
    instance.sim_args = {
        'timestep': 0.0001,
        'duration': 0.1,
        'init_steady_state': False,
        'pbar': False,
        'split_subnets': True,
        'log_level': 0,
    }
    instance.copy_settings = {
        # 'decouple': True,
        'decouple': 'diakoptics',
        'nodes': ['BUS5', 'BUS6', 'BUS8'],
        'resistance': 100,
        'inductance': 0.16,
        'capacitance': 1e-6,
    }

    schedulers = {
        'sequential',
        'omp_level 8',
    }

    size = 60
    splits = [1, 2, 3, 4, 5, 6, 10, 12, 15, 30, 60]

    meas = diakoptics_splitsize(instance, size, splits, schedulers)
    meas.save()
    check_numa()
