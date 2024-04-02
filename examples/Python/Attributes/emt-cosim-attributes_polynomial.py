import sys
import os.path
import logging
import dpsimpy

import numpy as np
import argparse

base = os.path.splitext(os.path.basename(sys.argv[0]))[0]
log = logging.getLogger(base)

def set_dpsim1(t_s, t_f, u_1_0, logger_prefix):
    r_1_r = 0.1
    c_1_c = 1
    r_line_r = 0.1
    
    # sim_name = "EMTCosimAttributes1"
    sim_name = "S1"

    gnd = dpsimpy.emt.SimNode.gnd
    n1 = dpsimpy.emt.SimNode("n1")
    n2 = dpsimpy.emt.SimNode("n2")

    evs = dpsimpy.emt.ph1.VoltageSource("v_intf", dpsimpy.LogLevel.info)
    evs.set_parameters(u_1_0)

    r_1 = dpsimpy.emt.ph1.Resistor("r_1", dpsimpy.LogLevel.info)
    r_1.set_parameters(r_1_r)
    c_1 = dpsimpy.emt.ph1.Capacitor("c_1", dpsimpy.LogLevel.info)
    c_1.set_parameters(c_1_c)
    r_line = dpsimpy.emt.ph1.Resistor("r_line", dpsimpy.LogLevel.info)
    r_line.set_parameters(r_line_r)
    
    # Initial conditions
    n1.set_initial_voltage(5 * dpsimpy.PEAK1PH_TO_RMS3PH)
    n2.set_initial_voltage(u_1_0 * dpsimpy.PEAK1PH_TO_RMS3PH)

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
    # sim.do_steady_state_init(True)
    
    dpsimpy.Logger.set_log_dir('logs/' + logger_prefix + '_' + sim_name)

    logger = dpsimpy.Logger(sim_name)
    logger.log_attribute('v1', 'v', n1)
    logger.log_attribute('v2', 'v', n2)
    logger.log_attribute('i', 'i_intf', evs)
    logger.log_attribute('ir', 'i_intf', r_line)
    
    sim.add_logger(logger)
    
    n1_v0 = np.array([5.0])
    n2_v0 = np.array([u_1_0])
    
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
  
    return sim


def set_dpsim2(t_s, t_f, logger_prefix):
    r_load_r = 1.0
    c_2_c = 1.0
    
    # sim_name = "EMTCosimAttributes2"
    sim_name = "S2"

    gnd = dpsimpy.emt.SimNode.gnd
    n2 = dpsimpy.emt.SimNode("n2")

    ecs = dpsimpy.emt.ph1.CurrentSource("i_intf", dpsimpy.LogLevel.info)
    c_2 = dpsimpy.emt.ph1.Capacitor("c_2", dpsimpy.LogLevel.info)
    c_2.set_parameters(c_2_c)
    r_load = dpsimpy.emt.ph1.Resistor("r_load", dpsimpy.LogLevel.info)
    r_load.set_parameters(r_load_r)
    
    # Initial conditions
    n2.set_initial_voltage(2.0 * dpsimpy.PEAK1PH_TO_RMS3PH)
    
    # Connections
    ecs.connect([gnd, n2])
    c_2.connect([gnd, n2])
    r_load.connect([gnd, n2])

    sys = dpsimpy.SystemTopology(50, [gnd, n2], [ecs, c_2, r_load])

    sim = dpsimpy.Simulation(sim_name, loglevel=dpsimpy.LogLevel.debug)
    sim.set_domain(dpsimpy.Domain.EMT)
    sim.set_system(sys)
    sim.set_time_step(t_s)
    sim.set_final_time(t_f)

    dpsimpy.Logger.set_log_dir('logs/' + logger_prefix + '_' + sim_name)

    logger = dpsimpy.Logger(sim_name)
    logger.log_attribute('v2', 'v', n2)
    logger.log_attribute('i', 'i_intf', ecs)
    logger.log_attribute('v', 'v_intf', ecs)

    sim.add_logger(logger)
    
    # Initialize currents and voltages
    n2_v0 = [2.0]
    i_r_load_0 = [n2_v0[0] / r_load_r]
    
    r_load.set_intf_voltage(n2_v0)
    r_load.set_intf_current(i_r_load_0)
    c_2.set_intf_voltage([n2_v0])
    
    ecs.set_intf_voltage(n2_v0)
      
    return sim

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-t', '--timestep', type=float, required=True)
    parser.add_argument('-e', '--end', type=float, required=True)
    parser.add_argument('-H', '--macro-step', type=float, required=True)
    parser.add_argument('-i', '--interp', default='zoh')
    parser.add_argument('-p', '--prefix', default='')
    parser.add_argument('-d', '--debug', type=bool, default=False)
    
    args = parser.parse_args()
    
    time_step = args.timestep
    t_f = args.end
    H = args.macro_step
    interp = args.interp 
    prefix = args.prefix   
    debug = args.debug
    
    logging.basicConfig(format='[%(asctime)s %(name)s %(levelname)s] %(message)s', datefmt='%H:%M:%S', level=logging.DEBUG)

    # time_step = 0.001
    t_0 = 0.0
    # t_f = 1
    t_k = 0.0
    
    n = int(round((t_f - t_0) / time_step))
    t = np.around(np.linspace(t_0, t_f, n + 1), 16)
    
    N = int(round((t_f - t_0) / H))
    t_m = np.around(np.linspace(t_0, t_f, N + 1), 16)
    
    m = int(H/time_step)
    print(m)
    
    # Initialization of S_2 and communication y_2_0 -> S_1
    sim2 = set_dpsim2(time_step, t_f, prefix)
    sim2.start()
    y_2_0 = sim2.get_idobj_attr("i_intf", "v_intf").derive_coeff(0,0).get()
    
    if debug:
        print("Output value from S2: {:f}".format(y_2_0))
   
    # Communication y_2_0 -> S_1 and initialization of S_1
    u_1_0 = y_2_0
    sim1 = set_dpsim1(time_step, t_f, u_1_0, prefix)
    sim1.start()
    y_1_0 = sim1.get_idobj_attr("v_intf", "i_intf").derive_coeff(0,0).get()
    
    if debug:
        print("Output value from S1: {:f}".format(y_1_0))
    
    y_1 = y_1_0
    
    # We have to assume the trajectory of y_2 extending its initial value, since we have no prior information
    y_1_m_prev = np.tile(y_1_0, m)
    
    for i in range(0, N):         
        y_1_prev = y_1_m_prev[-1]
        t_m_i = t[m*i : m*(i+1) + 1]
            
        # Extrapolation: Zero order hold
        if interp == 'zoh':
            u_2_m = np.tile(y_1_prev, m+1)
        elif interp == 'linear':
            f_u_2 = np.poly1d(np.polyfit([i*H-H, i*H], y_1_m_prev[-2:], 1))
            u_2_m = f_u_2(t_m_i)
        
        # j = 0
        y_1_m = np.zeros(m+1)
        
        for j in range(0, m+1):
            # Switch to S_2        
            if debug:
                u_2_prev = sim2.get_idobj_attr("i_intf", "i_intf").derive_coeff(0,0).get()
                print("Input value in S2 before set: {:f}".format(u_2_prev))
            
            u_2 = u_2_m[j]
            sim2.get_idobj_attr("i_intf", "I_ref").set(complex(u_2,0))
            
            if debug:
                u_2_test = sim2.get_idobj_attr("i_intf", "I_ref").get()
                print("Input value in S2 after set: {:f}".format(u_2_test))
            
            sim2.next()
            y_2 = sim2.get_idobj_attr("i_intf", "v_intf").derive_coeff(0,0).get()
            
            if debug:
                print("Output value from S2: {:f}".format(y_2))
            
            u_1 = y_2
            
            sim1.get_idobj_attr("v_intf", "V_ref").set(complex(u_1,0))
            t_k = sim1.next()
            y_1 = sim1.get_idobj_attr("v_intf", "i_intf").derive_coeff(0,0).get()
            
            y_1_m[j] = y_1
            # j += 1
            
            if debug:
                print("Output value from S1: {:f}".format(y_1))
                print("Time t={:f}".format(t_k))
                print("DPsim iteration: {}".format(j))

        # y_1_m_prev = y_1_m
        y_1_m_prev = np.append(y_1_m_prev, y_1_m[-1])
        
        if debug:
            print(y_1_m_prev)
    
    sim2.stop()
    sim1.stop()
        