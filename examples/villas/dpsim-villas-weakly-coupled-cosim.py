import sys
import os.path
import logging
import json
import time

from datetime import datetime
from villas.node.node import Node as VILLASnode

from multiprocessing import Process

import dpsimpy
import dpsimpyvillas

base = os.path.splitext(os.path.basename(sys.argv[0]))[0]
log = logging.getLogger(base)

def dpsim0():
    sim_name = "WeaklyCoupledCosim0"
    time_step = 1e-3
    final_time = 1.0

    gnd = dpsimpy.dp.SimNode.gnd
    n1 = dpsimpy.dp.SimNode('n1', dpsimpy.PhaseType.Single, [5])
    n2 = dpsimpy.dp.SimNode('n2', dpsimpy.PhaseType.Single, [2])

    evs = dpsimpy.dp.ph1.VoltageSource('v_intf', dpsimpy.LogLevel.debug)
    evs.set_parameters(complex(2, 0))

    r_1 = dpsimpy.dp.ph1.Resistor("r_1")
    r_1.set_parameters(0.1)
    c_1 = dpsimpy.dp.ph1.Capacitor("c_1")
    c_1.set_parameters(1)
    r_line = dpsimpy.dp.ph1.Resistor('r_line', dpsimpy.LogLevel.debug)
    r_line.set_parameters(0.1)

    r_1.connect([n1, gnd])
    r_line.connect([n1, n2])
    c_1.connect([n1, gnd])
    evs.connect([gnd, n2])

    sys = dpsimpy.SystemTopology(50, [n1, n2], [evs, r_1, c_1, r_line])

    sim = dpsimpy.Simulation(sim_name)
    sim.set_system(sys)
    sim.set_time_step(time_step)
    sim.set_final_time(final_time)

    intf_config = {
        "type": "socket",
        "layer": "udp",
        "format": "json",
        "hooks": [
            {"type": "print"}
        ],
        "in": {
            "address": "127.0.0.1:12008",
            "signals": [
                {
                    "name": "v_intf",
                    "type": "complex"
                }
            ]
        },
        "out": {
            "address": "127.0.0.1:12009",
            "signals": [
                {
                    "name": "i_intf",
                    "type": "complex"
                }
            ]
        }
    }
    
    dpsimpy.Logger.set_log_dir('logs/' + sim_name)

    logger = dpsimpy.Logger(sim_name)
    logger.log_attribute('1_v_1', 'v', n1)
    logger.log_attribute('2_v_2', 'v', n2)
    logger.log_attribute('3_i_rline', 'i_intf', r_line)
    logger.log_attribute('4_i_evs', 'i_intf', evs)
    logger.log_attribute('5_v_evs', 'v_intf', evs)
    
    intf = dpsimpyvillas.InterfaceVillas(name="dpsim0-dpsim1", config=intf_config)

    sim.add_interface(intf)
    sim.add_logger(logger)

    evs.set_intf_current([[complex(3, 0)]])

    intf.import_attribute(evs.attr('V_ref'), 0, block_on_read=False, sync_on_start=False)
    intf.export_attribute(evs.attr('i_intf').derive_coeff(0,0), 0, wait_for_on_write=True)
  
    sim.run()

def dpsim1():
    sim_name = "WeaklyCoupledCosim1"
    time_step = 1e-3
    final_time = 1.0

    gnd = dpsimpy.dp.SimNode.gnd
    n2 = dpsimpy.dp.SimNode('n2', dpsimpy.PhaseType.Single, [2])

    ecs = dpsimpy.dp.ph1.CurrentSource('i_intf', dpsimpy.LogLevel.debug)
    ecs.set_parameters(complex(3, 0))
    c_2 = dpsimpy.dp.ph1.Capacitor("c_2")
    c_2.set_parameters(1)
    r_load = dpsimpy.dp.ph1.Resistor('r_load', dpsimpy.LogLevel.debug)
    r_load.set_parameters(1)

    ecs.connect([gnd, n2])
    c_2.connect([gnd, n2])
    r_load.connect([gnd, n2])

    sys = dpsimpy.SystemTopology(50, [n2], [ecs, c_2, r_load])

    sim = dpsimpy.Simulation(sim_name)
    sim.set_system(sys)
    sim.set_time_step(time_step)
    sim.set_final_time(final_time)

    intf_config = {
        "type": "socket",
        "layer": "udp",
        "format": "json",
        "hooks": [
            {"type": "print"}
        ],
        "in": {
            "address": "127.0.0.1:12009",
            "signals": [
                {
                    "name": "i_intf",
                    "type": "complex"
                }
            ]
        },
        "out": {
            "address": "127.0.0.1:12008",
            "signals": [
                {
                    "name": "v_intf",
                    "type": "complex"
                }
            ]
        }
    }

    dpsimpy.Logger.set_log_dir('logs/' + sim_name)

    logger = dpsimpy.Logger(sim_name)
    logger.log_attribute('1_v_2', 'v', n2)
    logger.log_attribute('2_i_rload', 'i_intf', r_load)
    logger.log_attribute('3_v_ecs', 'v_intf', ecs)
    logger.log_attribute('4_i_ecs', 'i_intf', ecs)

    intf = dpsimpyvillas.InterfaceVillas(name="dpsim1-dpsim0", config=intf_config)

    sim.add_interface(intf)
    sim.add_logger(logger)
    intf.import_attribute(ecs.attr('I_ref'), 0, block_on_read=False, sync_on_start=False)
    intf.export_attribute(ecs.attr('v_intf').derive_coeff(0,0).derive_scaled(complex(-1,0)), 0, wait_for_on_write=True)
  
    sim.run()

if __name__ == '__main__':
    logging.basicConfig(format='[%(asctime)s %(name)s %(levelname)s] %(message)s', datefmt='%H:%M:%S', level=logging.DEBUG)

    p_sim0 = Process(target=dpsim0)
    p_sim1 = Process(target=dpsim1)
    
    p_sim0.start()
    p_sim1.start()

    p_sim0.join()
    p_sim1.join()

    print('Both simulations have ended!')
