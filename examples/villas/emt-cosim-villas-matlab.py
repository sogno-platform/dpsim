import sys
import os.path
import logging

import dpsimpy
import dpsimpyvillas
import numpy as np

from multiprocessing import Process

base = os.path.splitext(os.path.basename(sys.argv[0]))[0]
log = logging.getLogger(base)

if __name__ == '__main__':
    logging.basicConfig(format='[%(asctime)s %(name)s %(levelname)s] %(message)s', datefmt='%H:%M:%S', level=logging.DEBUG)
    t_s = 0.001
    t_f = 1
    
    r_1_r = 0.1
    c_1_c = 1
    r_line_r = 0.1
    
    sim_name = "EMTCosimVILLAS1"

    gnd = dpsimpy.emt.SimNode.gnd
    n1 = dpsimpy.emt.SimNode("n1")
    n2 = dpsimpy.emt.SimNode("n2")

    evs = dpsimpy.emt.ph1.VoltageSource('v_intf', dpsimpy.LogLevel.info)
    evs.set_parameters(2)

    r_1 = dpsimpy.emt.ph1.Resistor("r_1", dpsimpy.LogLevel.info)
    r_1.set_parameters(r_1_r)
    c_1 = dpsimpy.emt.ph1.Capacitor("c_1", dpsimpy.LogLevel.info)
    c_1.set_parameters(c_1_c)
    r_line = dpsimpy.emt.ph1.Resistor('r_line', dpsimpy.LogLevel.info)
    r_line.set_parameters(r_line_r)
    
    # Initial conditions
    n1.set_initial_voltage(5)
    n2.set_initial_voltage(2)

    # Connections
    r_1.connect([gnd, n1])
    r_line.connect([n2, n1])
    c_1.connect([gnd, n1])
    evs.connect([gnd, n2])

    sys = dpsimpy.SystemTopology(50, [gnd, n1, n2], [evs, r_1, c_1, r_line])

    sim = dpsimpy.Simulation(sim_name, loglevel=dpsimpy.LogLevel.debug)
    sim.set_domain(dpsimpy.Domain.EMT)
    sim.set_system(sys)
    sim.set_time_step(t_s)
    sim.set_final_time(t_f)

    intf_config = {
        "type": "socket",
        "layer": "udp",
        "format": "raw",
        "in": {
            "address": "*:12008",
            "signals": [
                {
                    "name": "v_intf",
                    "type": "complex"
                }
            ],
            "hooks": [
                {"type": "print", "enabled": True}
            ]
        },
        "out": {
            "address": "137.226.133.227:12008",
            "signals": [
                {
                    "name": "i_intf",
                    "type": "complex"
                }
            ],
            "hooks": [
                {"type": "print", "enabled": False}
            ]
        }
    }
    
    dpsimpy.Logger.set_log_dir('logs/' + sim_name)

    logger = dpsimpy.Logger(sim_name)
    logger.log_attribute('v1', 'v', n1)
    logger.log_attribute('v2_S1', 'v', n2)
    logger.log_attribute('i', 'i_intf', evs)
    logger.log_attribute('ir', 'i_intf', r_line)
    logger.log_attribute('V_ref', 'V_ref', evs)
    
    intf = dpsimpyvillas.InterfaceVillas(name="dpsim0-dpsim1", config=intf_config)

    sim.add_interface(intf)
    sim.add_logger(logger)

    n1_v0 = np.array([5.0])
    n2_v0 = np.array([2.0])
    
    ir_1_0 = n1_v0 / r_1_r
    i_r_line_0 = (n1_v0 - n2_v0) / r_line_r
    
    r_1.set_intf_voltage(n1_v0)
    r_1.set_intf_current(ir_1_0)
    c_1.set_intf_voltage(n1_v0)
    c_1.set_intf_current(ir_1_0 - i_r_line_0)
    r_line.set_intf_voltage(n1_v0 - n2_v0)
    r_line.set_intf_current(i_r_line_0)
    
    evs.set_intf_voltage(n2_v0)
    evs.set_intf_current(i_r_line_0)

    # This expects to import a complex value
    intf.import_attribute(evs.attr('V_ref'), 0, block_on_read=False, sync_on_start=False, name='from_matlab', unit='V')
    
    intf.export_attribute(evs.attr('i_intf').derive_coeff(0,0).derive_complex(), 0, wait_for_on_write=True, name='to_matlab', unit='A')
  
    sim.run()
