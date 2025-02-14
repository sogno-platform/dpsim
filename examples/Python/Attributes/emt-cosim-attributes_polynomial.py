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

    sim_name = "S1"

    gnd = dpsimpy.emt.SimNode.gnd

    if with_vs_1:
        n0 = dpsimpy.emt.SimNode("n0")
        v_s_1 = dpsimpy.emt.ph1.VoltageSource("v_s_1")

    if with_vs_2:
        v_s_1.set_parameters(V_ref=complex(2, 0), f_src=50)
    elif with_vs_1:
        v_s_1.set_parameters(V_ref=complex(1, 0), f_src=50)

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
    # sim.do_steady_state_init(True)

    dpsimpy.Logger.set_log_dir('logs/' + logger_prefix + '_' + sim_name)

    logger = dpsimpy.Logger(sim_name)
    logger.log_attribute('v_1', 'v', n1)
    logger.log_attribute('v_2', 'v', n2)
    logger.log_attribute('v_intf', 'v_intf', evs)
    logger.log_attribute('v_ref', 'V_ref', evs)
    logger.log_attribute('i_intf', 'i_intf', evs)
    logger.log_attribute('i_r_line', 'i_intf', r_line)

    sim.add_logger(logger)

    if with_vs_2:
        n0_v0 = np.array([2.0])
    elif with_vs_1:
        n0_v0 = np.array([1.0])

    if with_vs_1:
        n1_v0 = np.array([0.0])
        ir_1_0 = (n0_v0 - n1_v0) / r_1_r
    else:
        n1_v0 = np.array([5.0])
        ir_1_0 = n1_v0 / r_1_r
    n2_v0 = np.array([u_1_0])

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


def set_dpsim2(t_s, t_f, num_vs, logger_prefix):

    with_vs_1 = num_vs > 0
    with_vs_2 = num_vs > 1

    r_load_r = 1.0
    c_2_c = 1.0

    sim_name = "S2"

    gnd = dpsimpy.emt.SimNode.gnd
    n2 = dpsimpy.emt.SimNode("n2")

    if with_vs_2:
        n3 = dpsimpy.emt.SimNode("n3")
        v_s_2 = dpsimpy.emt.ph1.VoltageSource("v_s_2")
        v_s_2.set_parameters(V_ref=complex(np.sqrt(2)/2, np.sqrt(2)/2), f_src=50)

    ecs = dpsimpy.emt.ph1.CurrentSource("i_intf", dpsimpy.LogLevel.info)
    # ecs.set_parameters(30.0)

    c_2 = dpsimpy.emt.ph1.Capacitor("c_2", dpsimpy.LogLevel.info)
    c_2.set_parameters(c_2_c)
    r_load = dpsimpy.emt.ph1.Resistor("r_load", dpsimpy.LogLevel.info)
    r_load.set_parameters(r_load_r)

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

    # Initialize currents and voltages
    if with_vs_1:
        n2_v0 = np.array([0.0])
    else:
        n2_v0 = np.array([2.0])

    if with_vs_2:
        n3_v0 = np.array([np.cos(np.pi/4)])
        i_r_load_0 = (n3_v0 - n2_v0) / r_load_r
        r_load.set_intf_voltage(n3_v0 - n2_v0)
    else:
        i_r_load_0 = [n2_v0 / r_load_r]
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
    parser.add_argument('-m', '--method', default='none')
    parser.add_argument('--y1-prev', type=float, default=0)
    parser.add_argument('-p', '--prefix', default='')
    parser.add_argument('--num-vs', default=0)
    parser.add_argument('-d', '--debug', type=bool, default=False)

    args = parser.parse_args()

    time_step = args.timestep
    t_f = args.end
    H = args.macro_step
    method = args.method
    y_1_prev = args.y1_prev
    prefix = args.prefix
    num_vs = int(args.num_vs)
    debug = args.debug

    if method == 'extrapolation-linear' and y_1_prev is None:
        print('Error: Linear extrapolation requires initial previous value!')
        exit(1)

    if method == 'none':
        NO_EXTRAP = True
    else:
        NO_EXTRAP = False

    logging.basicConfig(format='[%(asctime)s %(name)s %(levelname)s] %(message)s', datefmt='%H:%M:%S', level=logging.DEBUG)

    t_0 = 0.0
    t_k = 0.0

    n = int(round((t_f - t_0) / time_step))
    t = np.around(np.linspace(t_0, t_f, n + 1), 16)

    N = int(round((t_f - t_0) / H))
    t_m = np.around(np.linspace(t_0, t_f, N + 1), 16)

    m = int(round(H/time_step))
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
    y_1_0 = sim1.get_idobj_attr("v_intf", "i_intf").derive_coeff(0,0).get()

    if debug:
        print("Output value from S1: {:f}".format(y_1_0))

    y_1 = y_1_0

    # We have to assume the trajectory of y_2 extending its initial value, since we have no prior information
    if method == "extrapolation-linear":
        y_1_m_prev = np.array([y_1_prev, y_1_0])
    else:
        y_1_m_prev = np.tile(y_1_0, m)
    # y_1_m_prev = np.array([0.0, y_1_0])

    if NO_EXTRAP:

        while t_k <= t_f:
            u_2 = y_1
            sim2.get_idobj_attr("i_intf", "I_ref").set(complex(u_2,0))

            # if t_k == 0.0:
            #     sim2.start()
            # else:
            sim2.next()
            y_2 = sim2.get_idobj_attr("i_intf", "v_intf").derive_coeff(0,0).get()

            u_1 = y_2

            sim1.get_idobj_attr("v_intf", "V_ref").set(complex(u_1,0))
            t_k = sim1.next()
            y_1 = sim1.get_idobj_attr("v_intf", "i_intf").derive_coeff(0,0).get()

    else:

        for i in range(0, N):
            y_1_prev = y_1_m_prev[-1]
            t_m_i = t[m*(i)+1:m*(i+1)+1]

            # Extrapolation: Zero order hold
            if method == 'extrapolation-zoh':
                u_2_m = np.tile(y_1_prev, m)
            elif method == 'extrapolation-linear':
                f_u_2 = np.poly1d(np.polyfit([i*H-H, i*H], y_1_m_prev[-2:], 1))
                u_2_m = f_u_2(t_m_i)
            elif method == 'delay':
                u_2_m = y_1_m_prev

            # j = 0
            y_1_m = np.zeros(m)
            y_2_m = np.zeros(m)

            # Switch to S_2
            for j in range(0, m):
                if debug:
                    u_2_prev = sim2.get_idobj_attr("i_intf", "i_intf").derive_coeff(0,0).get()
                    print("Input value in S2 before set: {:f}".format(u_2_prev))

                u_2 = u_2_m[j]

                sim2.get_idobj_attr("i_intf", "I_ref").set(complex(u_2,0))

                if debug:
                    u_2_test = sim2.get_idobj_attr("i_intf", "I_ref").get()
                    print("Input value in S2 after set: {:f}".format(u_2_test))

                # if i == 0 and j == 0:
                #     t_k_2 = sim2.start()
                # else:
                t_k_2 = sim2.next()
                y_2 = sim2.get_idobj_attr("i_intf", "v_intf").derive_coeff(0,0).get()
                y_2_m[j] = y_2

                if debug:
                    print("Output value from S2: {:f}".format(y_2))

            # Switch to S_1
            u_1_m = y_2_m
            for j in range(0, m):
                u_1 = u_1_m[j]

                sim1.get_idobj_attr("v_intf", "V_ref").set(complex(u_1,0))
                t_k = sim1.next()
                y_1 = sim1.get_idobj_attr("v_intf", "i_intf").derive_coeff(0,0).get()

                y_1_m[j] = y_1

                if debug:
                    print("Output value from S1: {:f}".format(y_1))
                    print("Time t={:f}".format(t_k))
                    print("DPsim iteration: {}".format(j))

            if method == 'delay':
                y_1_m_prev = y_1_m
            else:
                y_1_m_prev = np.append(y_1_m_prev, y_1_m[-1])

            if debug:
                print(y_1_m_prev)

    sim2.stop()
    sim1.stop()
