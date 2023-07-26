import os
import urllib.request
import glob

import dpsimpy
import dpsimpyvillas

from multiprocessing import Process, Queue

import villas.dataprocessing.readtools as rt
import villas.dataprocessing.plottools as pt

if __name__ == '__main__':
    sim_name = "ShmemDistributed"
    time_step = 0.001

    n1 = dpsimpy.dp.SimNode('n1')
    n2 = dpsimpy.dp.SimNode('n2')

    vs1 = dpsimpy.dp.ph1.VoltageSource('vs_1', dpsimpy.LogLevel.debug)
    vs1.set_parameters(complex(10, 0))
    r12 = dpsimpy.dp.ph1.Resistor('r_1', dpsimpy.LogLevel.debug)
    r12.set_parameters(1)
    r02 = dpsimpy.dp.ph1.Resistor('r_02', dpsimpy.LogLevel.debug)
    r02.set_parameters(1)

    vs1.connect([dpsimpy.dp.SimNode.gnd, n1])
    r12.connect([n1, n2])
    r02.connect([dpsimpy.dp.SimNode.gnd, n2])

    sys = dpsimpy.SystemTopology(50, [n1, n2], [vs1, r12, r02])

    dpsimpy.Logger.set_log_dir('logs/' + sim_name)

    logger = dpsimpy.Logger(sim_name)
    logger.log_attribute('v1', n1.attr('v'))
    logger.log_attribute('v2', n2.attr('v'))
    logger.log_attribute('r12', r12.attr('i_intf'))
    logger.log_attribute('r02', r02.attr('i_intf'))

    sim = dpsimpy.RealTimeSimulation(sim_name)
    sim.set_system(sys)
    sim.set_time_step(time_step)
    sim.set_final_time(10)

    sim.add_logger(logger)
    sim.run(1)
