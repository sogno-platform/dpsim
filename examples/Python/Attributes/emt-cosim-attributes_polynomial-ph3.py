import sys
import os.path
import logging
import dpsimpy

import numpy as np
import argparse

base = os.path.splitext(os.path.basename(sys.argv[0]))[0]
log = logging.getLogger(base)


def set_dpsim1(t_s, t_f, u_1_0, num_vs, logger_prefix):

    with_vs_1 = num_vs > 0
    with_vs_2 = num_vs > 1

    r_1_r = 0.1
    c_1_c = 1
    r_line_r = 0.1

    sim_name = "S1_ph3"

    gnd = dpsimpy.emt.SimNode.gnd

    if with_vs_1:
        n0 = dpsimpy.emt.SimNode("n0", dpsimpy.PhaseType.ABC)
        v_s_1 = dpsimpy.emt.ph3.VoltageSource("v_s_1")
        
    if with_vs_2:
        v_s_1_rms = 2.0 * dpsimpy.PEAK1PH_TO_RMS3PH
        v_s_1_c = complex(v_s_1_rms, 0)
        v_s_1.set_parameters(V_ref=dpsimpy.Math.single_phase_variable_to_three_phase(v_s_1_c), f_src=50)
    elif with_vs_1:
        v_s_1_rms = 1.0 * dpsimpy.PEAK1PH_TO_RMS3PH
        v_s_1_c = complex(v_s_1_rms, 0)
        v_s_1.set_parameters(V_ref=dpsimpy.Math.single_phase_variable_to_three_phase(v_s_1_c), f_src=50)
    
    n1 = dpsimpy.emt.SimNode("n1", dpsimpy.PhaseType.ABC)
    n2 = dpsimpy.emt.SimNode("n2", dpsimpy.PhaseType.ABC)

    evs = dpsimpy.emt.ph3.VoltageSource("v_intf", dpsimpy.LogLevel.info)
    evs.set_parameters(dpsimpy.Math.single_phase_variable_to_three_phase(u_1_0), 0)

    r_1 = dpsimpy.emt.ph3.Resistor("r_1", dpsimpy.LogLevel.info)
    r_1.set_parameters(dpsimpy.Math.single_phase_parameter_to_three_phase(r_1_r))
    c_1 = dpsimpy.emt.ph3.Capacitor("c_1", dpsimpy.LogLevel.info)
    c_1.set_parameters(dpsimpy.Math.single_phase_parameter_to_three_phase(c_1_c))
    r_line = dpsimpy.emt.ph3.Resistor("r_line", dpsimpy.LogLevel.info)
    r_line.set_parameters(dpsimpy.Math.single_phase_parameter_to_three_phase(r_line_r))

    # Initial conditions
    if with_vs_2:
        n0.set_initial_voltage(2 * dpsimpy.PEAK1PH_TO_RMS3PH)
    elif with_vs_1:
        n0.set_initial_voltage(1 * dpsimpy.PEAK1PH_TO_RMS3PH)

    if with_vs_1:
        # This adds a transient at t=0
        n1.set_initial_voltage(0 * dpsimpy.PEAK1PH_TO_RMS3PH)
    else:
        n1.set_initial_voltage(5 * dpsimpy.PEAK1PH_TO_RMS3PH)

    n2.set_initial_voltage(u_1_0 * dpsimpy.PEAK1PH_TO_RMS3PH)

    # Connections
    if with_vs_1:
        v_s_1.connect([gnd, n0])
        r_1.connect([n0, n1])
    else:
        r_1.connect([gnd, n1])

    r_line.connect([n1, n2])
    c_1.connect([n1, gnd])
    evs.connect([gnd, n2])

    if with_vs_1:
        sys = dpsimpy.SystemTopology(50, [gnd, n0, n1, n2], [v_s_1, evs, r_1, c_1, r_line])
    else:
        sys = dpsimpy.SystemTopology(50, [gnd, n1, n2], [evs, r_1, c_1, r_line])

    sim = dpsimpy.Simulation(sim_name, loglevel=dpsimpy.LogLevel.debug)
    sim.set_domain(dpsimpy.Domain.EMT)
    sim.set_system(sys)
    sim.set_time_step(t_s)
    sim.set_final_time(t_f)

    dpsimpy.Logger.set_log_dir('logs/' + logger_prefix + '_' + sim_name)

    logger = dpsimpy.Logger(sim_name)
    logger.log_attribute('v_1', 'v', n1)
    logger.log_attribute('v_2', 'v', n2)
    logger.log_attribute('v_intf', 'v_intf', evs)
    logger.log_attribute('v_ref', 'V_ref', evs)
    logger.log_attribute('i_intf', 'i_intf', evs)
    logger.log_attribute('i_r_line', 'i_intf', r_line)

    sim.add_logger(logger)

    return sim


def set_dpsim2(t_s, t_f, num_vs, logger_prefix):

    with_vs_1 = num_vs > 0
    with_vs_2 = num_vs > 1

    r_load_r = 1.0
    c_2_c = 1.0

    sim_name = "S2_ph3"

    gnd = dpsimpy.emt.SimNode.gnd
    n2 = dpsimpy.emt.SimNode("n2", dpsimpy.PhaseType.ABC)

    if with_vs_2:
        n3 = dpsimpy.emt.SimNode("n3", dpsimpy.PhaseType.ABC)
        v_s_2_peak = 1.0 * dpsimpy.PEAK1PH_TO_RMS3PH
        v_s_2_phase = np.pi / 4.0

        v_s_2_exp = v_s_2_peak*np.exp(1j*v_s_2_phase)

        v_s_2 = dpsimpy.emt.ph3.VoltageSource("v_s_2")
        v_s_2_c = complex(np.real(v_s_2_exp), np.imag(v_s_2_exp))
        v_s_2.set_parameters(V_ref=dpsimpy.Math.single_phase_variable_to_three_phase(v_s_2_c), f_src=50)

    ecs = dpsimpy.emt.ph3.CurrentSource("i_intf", dpsimpy.LogLevel.info)
    ecs.set_parameters(dpsimpy.Math.single_phase_variable_to_three_phase(complex(0,0)), 0)

    c_2 = dpsimpy.emt.ph3.Capacitor("c_2", dpsimpy.LogLevel.info)
    c_2.set_parameters(dpsimpy.Math.single_phase_parameter_to_three_phase(c_2_c))
    r_load = dpsimpy.emt.ph3.Resistor("r_load", dpsimpy.LogLevel.info)
    r_load.set_parameters(dpsimpy.Math.single_phase_parameter_to_three_phase(r_load_r))

    # Initial conditions
    if with_vs_1:
        n2.set_initial_voltage(0.0 * dpsimpy.PEAK1PH_TO_RMS3PH)
        if with_vs_2:
            n3.set_initial_voltage(np.cos(np.pi/4) * dpsimpy.PEAK1PH_TO_RMS3PH)
    else:
        n2.set_initial_voltage(2.0 * dpsimpy.PEAK1PH_TO_RMS3PH)

    # Connections
    ecs.connect([gnd, n2])
    c_2.connect([n2, gnd])

    if with_vs_2:
        v_s_2.connect([gnd, n3])
        r_load.connect([n2, n3])
    else:
        r_load.connect([gnd, n2])

    if with_vs_2:
        sys = dpsimpy.SystemTopology(50, [gnd, n2, n3], [ecs, c_2, r_load, v_s_2])
    else:
        sys = dpsimpy.SystemTopology(50, [gnd, n2], [ecs, c_2, r_load])

    sim = dpsimpy.Simulation(sim_name, loglevel=dpsimpy.LogLevel.debug)
    sim.set_domain(dpsimpy.Domain.EMT)
    sim.set_system(sys)
    sim.set_time_step(t_s)
    sim.set_final_time(t_f)

    dpsimpy.Logger.set_log_dir('logs/' + logger_prefix + '_' + sim_name)

    logger = dpsimpy.Logger(sim_name)
    logger.log_attribute('v_2', 'v', n2)
    logger.log_attribute('i_intf', 'i_intf', ecs)
    logger.log_attribute('i_ref', 'I_ref', ecs)
    logger.log_attribute('v_intf', 'v_intf', ecs)

    sim.add_logger(logger)

    return sim


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-t', '--timestep', type=float, required=True)
    parser.add_argument('-e', '--end', type=float, required=True)
    parser.add_argument('-H', '--macro-step', type=float, required=True)
    parser.add_argument('-i', '--interp', default='zoh')
    parser.add_argument('-p', '--prefix', default='')
    parser.add_argument('--num-vs', default=0)
    parser.add_argument('-d', '--debug', type=bool, default=False)

    args = parser.parse_args()

    time_step = args.timestep
    t_f = args.end
    H = args.macro_step
    interp = args.interp
    prefix = args.prefix
    num_vs = int(args.num_vs)
    debug = args.debug

    k_map = {'none': 0, 'zoh': 0, 'linear': 1}
    k = k_map[interp]

    if interp == 'none':
        NO_EXTRAP = True
    else:
        NO_EXTRAP = False

    logging.basicConfig(format='[%(asctime)s %(name)s %(levelname)s] %(message)s', datefmt='%H:%M:%S', level=logging.DEBUG)

    t_0 = 0.0
    t_k_v = []
    t_k = 0.0

    n = int(round((t_f - t_0) / time_step))
    t = np.around(np.linspace(t_0, t_f, n + 1), 16)

    N = int(round((t_f - t_0) / H))
    t_m = np.around(np.linspace(t_0, t_f, N + 1), 16)

    m = int(H/time_step)
    print(m)

    # Initialization of S_2 and communication y_2_0 -> S_1
    sim2 = set_dpsim2(time_step, t_f, num_vs, prefix)
    sim2.start()
    y_2_0 = sim2.get_idobj_attr("i_intf", "v_intf").derive_coeff(0,0).get()

    if debug:
        print("Output value from S2: {:f}".format(y_2_0))

    # Communication y_2_0 -> S_1 and initialization of S_1
    u_1_0 = y_2_0
    sim1 = set_dpsim1(time_step, t_f, u_1_0, num_vs, prefix)
    sim1.start()
    y_1_0 = sim1.get_idobj_attr("v_intf", "i_intf").get()

    if debug:
        print("Output value from S1: {:f}".format(y_1_0))

    y_1 = y_1_0

    # We have to assume the trajectory of y_2 extending its initial value, since we have no prior information
    # y_1_m_prev = np.tile(y_1_0, m)
    y_1_m_prev = np.tile(y_1_0, np.min([k+1, m]))

    if NO_EXTRAP:
        
        while t_k <= t_f:
            u_2 = y_1
            sim2.get_idobj_attr("i_intf", "I_ref").set([complex(elem, 0) for elem in u_2 * dpsimpy.PEAK1PH_TO_RMS3PH])
            sim2.next()
            y_2 = sim2.get_idobj_attr("i_intf", "v_intf").get()
            
            u_1 = y_2

            sim1.get_idobj_attr("v_intf", "V_ref").set([complex(elem, 0) for elem in u_1 * dpsimpy.PEAK1PH_TO_RMS3PH])
            t_k = sim1.next()
            y_1 = sim1.get_idobj_attr("v_intf", "i_intf").get()

    else:

        for i in range(0, N):
            y_1_prev = y_1_m_prev[:,-1]
            t_m_i = t[m*i+1: m*(i+1)+1]

            # Extrapolation: Zero order hold
            if interp == 'zoh':
                u_2_m = np.tile(y_1_prev, (m,1)).T
            elif interp == 'linear':
                t_poly = np.array([i*H-H, i*H])
                f_u_2 = np.polynomial.polynomial.polyfit(t_poly, y_1_m_prev[:,-2:].T, 1)
                u_2_m = np.polynomial.polynomial.polyval(t_m_i, f_u_2)

            # j = 0
            y_1_m = np.zeros((len(y_1_0), m))
            y_2_m = np.zeros((len(y_1_0), m))

            # Switch to S_2
            for j in range(0, m):
                if debug:
                    u_2_prev = sim2.get_idobj_attr("i_intf", "i_intf").derive_coeff(0,0).get()
                    print("Input value in S2 before set: {:f}".format(u_2_prev))

                u_2 = u_2_m[:,j]
                sim2.get_idobj_attr("i_intf", "I_ref").set([complex(elem, 0) for elem in u_2 * dpsimpy.PEAK1PH_TO_RMS3PH])

                if debug:
                    u_2_test = sim2.get_idobj_attr("i_intf", "I_ref").get()
                    print("Input value in S2 after set: {:f}".format(u_2_test))

                sim2.next()
                y_2 = sim2.get_idobj_attr("i_intf", "v_intf").get()

                if debug:
                    print("Output value from S2: {:f}".format(y_2))

                y_2_m[:,j] = y_2.flatten()

            # Switch to S_1
            u_1_m = y_2_m  
            for j in range(0, m):    
                u_1 = u_1_m[:,j]

                sim1.get_idobj_attr("v_intf", "V_ref").set([complex(elem, 0) for elem in u_1 * dpsimpy.PEAK1PH_TO_RMS3PH])
                t_k_1 = sim1.next()
                y_1 = sim1.get_idobj_attr("v_intf", "i_intf").get()

                y_1_m[:,j] = y_1.flatten()
                t_k_v.append(t_k_1)

                if debug:
                    print("Output value from S1: {:f}".format(y_1))
                    print("Time t={:f}".format(t_k))
                    print("DPsim iteration: {}".format(j))

            # y_1_m_prev = y_1_m
            y_1_m_prev = np.hstack((y_1_m_prev, y_1_m[:,-1][np.newaxis].T))

            if debug:
                print(y_1_m_prev)
        
            if t_k_1 > t_f:
                print(t_k_1)
                print("i: " + str(i) + ", expected: " + str(N-1))
                print("j: " + str(j) + ", expected: " + str(m-1))
                break

    sim2.stop()
    sim1.stop()
