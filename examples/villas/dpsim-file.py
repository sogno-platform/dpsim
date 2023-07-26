# This example demonstrates the export of values calculated by dpsim to a file using the VILLASnode interface

import dpsimpy
import dpsimpyvillas

sim_name = "DPsim_File"
time_step = 0.01
final_time = 10

n1 = dpsimpy.dp.SimNode('n1', dpsimpy.PhaseType.Single, [10])
n2 = dpsimpy.dp.SimNode('n2', dpsimpy.PhaseType.Single, [5])

evs = dpsimpy.dp.ph1.VoltageSource('v_intf', dpsimpy.LogLevel.debug)
evs.set_parameters(complex(5, 0))

vs1 = dpsimpy.dp.ph1.VoltageSource('vs_1', dpsimpy.LogLevel.debug)
vs1.set_parameters(complex(10, 0))

r12 = dpsimpy.dp.ph1.Resistor('r_12', dpsimpy.LogLevel.debug)
r12.set_parameters(1)

evs.connect([dpsimpy.dp.SimNode.gnd, n2])
vs1.connect([dpsimpy.dp.SimNode.gnd, n1])
r12.connect([n1, n2])

sys = dpsimpy.SystemTopology(50, [n1, n2], [evs, vs1, r12])

dpsimpy.Logger.set_log_dir('logs/' + sim_name)

logger = dpsimpy.Logger(sim_name)
logger.log_attribute('v1', n1.attr('v'))
logger.log_attribute('v2', n2.attr('v'))
logger.log_attribute('r12', r12.attr('i_intf'))
logger.log_attribute('ievs', evs.attr('i_intf'))
logger.log_attribute('vevs', evs.attr('v_intf'))

sim = dpsimpy.RealTimeSimulation(sim_name)
sim.set_system(sys)
sim.set_time_step(time_step)
sim.set_final_time(final_time)

file_config = '''{
        "type": "file",
        "format": "csv",
        "uri": "logs/output.csv",
        "out": {
            "flush": true
        }
}'''

intf = dpsimpyvillas.InterfaceVillas(file_config, name='dpsim-file')
intf.export_attribute(evs.attr('i_intf').derive_coeff(0, 0), 0)

sim.add_interface(intf)
#sim.import_attribute('v_intf', 'V_ref', 0)

sim.add_logger(logger)

evs.set_intf_current([[complex(5, 0)]])

sim.run(1)
