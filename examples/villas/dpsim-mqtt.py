# This example demonstrates the export of values calculated by dpsim to a MQTT broker using the VILLASnode interface
# Note, that dpsim also expects to read a (complex) reference voltage from MQTT, so the simulation will block on every timestep until this value is provided 

import json

import dpsimpy
import dpsimpyvillas

sim_name = "DPsim_MQTT"
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
logger.log_attribute('v1', 'v', n1)
logger.log_attribute('v2', 'v', n2)
logger.log_attribute('r12', 'i_intf', r12)
logger.log_attribute('ievs', 'i_intf', evs)
logger.log_attribute('vevs', 'v_intf', evs)

sim = dpsimpy.RealTimeSimulation(sim_name)
sim.set_system(sys)
sim.set_time_step(time_step)
sim.set_final_time(final_time)

intf_config = {
    'type': 'mqtt',
    'format': 'json',
    'host': 'mqtt',
    'in': {
        'subscribe': '/mqtt-dpsim'
    },
    'out': {
        'publish': '/dpsim-mqtt'
    }
}

intf = dpsimpyvillas.InterfaceVillas(name='dpsim-mqtt', config=json.dumps(intf_config))
intf.import_attribute(evs.attr('V_ref'), 0, True)
intf.export_attribute(r12.attr('i_intf').derive_coeff(0, 0), 0)

sim.add_interface(intf)

sim.add_logger(logger)

sim.run(1)