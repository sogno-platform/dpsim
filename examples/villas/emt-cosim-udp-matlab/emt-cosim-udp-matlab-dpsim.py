import sys
import os.path
import logging

import dpsimpy
import dpsimpyvillas
import numpy as np
import time, socket, errno, signal, struct

from multiprocessing import Process

base = os.path.splitext(os.path.basename(sys.argv[0]))[0]
log = logging.getLogger(base)

if __name__ == '__main__':
    local = ("0.0.0.0", 12008)
    remote = (os.environ.get('MATLAB_HOST'), 12009)
    
    logging.basicConfig(format='[%(asctime)s %(name)s %(levelname)s] %(message)s', datefmt='%H:%M:%S', level=logging.DEBUG)
    t_0 = 0.0
    t_s = 0.001
    t_f = 1
    
    r_1_r = 0.1
    c_1_c = 1
    r_line_r = 0.1
    
    sim_name = "EMTCosimUDP1"

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
    
    dpsimpy.Logger.set_log_dir('logs/' + sim_name)

    logger = dpsimpy.Logger(sim_name)
    logger.log_attribute('v1', 'v', n1)
    logger.log_attribute('v2_S1', 'v', n2)
    logger.log_attribute('i', 'i_intf', evs)
    logger.log_attribute('ir', 'i_intf', r_line)
    logger.log_attribute('V_ref', 'V_ref', evs)

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
    
    sim.start()
  
    skt = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    # This client must be started first
    print("Start client...")

    skt.bind(local)

    # Try to connect in case Unix domain socket does not exist yet..
    connected = False
    while not connected:
        try:
            skt.connect(remote)
        except socket.error as serr:
            if serr.errno not in [errno.ECONNREFUSED, errno.ENOENT]:
                raise serr

            print("Not connected. Retrying in 1 sec")
            time.sleep(1)
        else:
            connected = True
            
    print("Ready. Ctrl-C to quit.")


    # Gracefully shutdown
    def sighandler(signum, frame):
        running = False


    signal.signal(signal.SIGINT, sighandler)
    signal.signal(signal.SIGTERM, sighandler)

    running = True
    t_k = 0.0
    
    while t_k < t_f:
        dgram, addr = skt.recvfrom(1024)
        if not dgram:
            break
        else:
            u_1 = struct.unpack('f', dgram)
            print(u_1)

            sim.get_idobj_attr("v_intf", "V_ref").set(complex(u_1[0],0))
            t_k = sim.next()
            y_1 = sim.get_idobj_attr("v_intf", "i_intf").derive_coeff(0,0).get()
            skt.send(struct.pack('f', y_1))

    skt.close()

    print("Bye.")