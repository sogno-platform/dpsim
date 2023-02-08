import sys
import os.path
import logging

import dpsimpy

sim_name = "DistributedVILLAS_base"
time_step = 0.01
final_time = 10

n1 = dpsimpy.dp.SimNode('n1', dpsimpy.PhaseType.Single, [10])
n2 = dpsimpy.dp.SimNode('n2', dpsimpy.PhaseType.Single, [5])

vs1 = dpsimpy.dp.ph1.VoltageSource('vs_1', dpsimpy.LogLevel.debug)
vs1.set_parameters(complex(10, 0))

r12 = dpsimpy.dp.ph1.Resistor('r_12', dpsimpy.LogLevel.debug)
r12.set_parameters(1)

r02 = dpsimpy.dp.ph1.Resistor('r_02', dpsimpy.LogLevel.debug)
r02.set_parameters(1)

vs1.connect([dpsimpy.dp.SimNode.gnd, n1])
r12.connect([n1, n2])
r02.connect([dpsimpy.dp.SimNode.gnd, n2])

sys = dpsimpy.SystemTopology(50, [n1, n2], [vs1, r12, r02])

sim = dpsimpy.RealTimeSimulation(sim_name)
sim.set_system(sys)
sim.set_time_step(time_step)
sim.set_final_time(final_time)

dpsimpy.Logger.set_log_dir('logs/' + sim_name)

logger = dpsimpy.Logger(sim_name)
logger.log_attribute('v1', 'v', n1)
logger.log_attribute('v2', 'v', n2)
logger.log_attribute('r12', 'i_intf', r12)
logger.log_attribute('r02', 'v_intf', r02)

sim.add_logger(logger)

sim.run(1)
