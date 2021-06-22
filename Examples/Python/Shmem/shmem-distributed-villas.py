import os
import urllib.request
import glob

import dpsimpy
import dpsimpyvillas

from multiprocessing import Process, Queue

import villas.dataprocessing.readtools as rt
import villas.dataprocessing.plottools as pt

def villas():

    villas_conf = """
        hugepages = 100

        nodes = {
            shmem0 = {
                type = "shmem",
                queuelen = 2048,

                in = {
                    name = "/dpsim0-in"

                    signals = "1c"
                },
                out = {
                    name = "/dpsim0-out"

                    signals = "1c"
                }
            },
            shmem1 = {
                type = "shmem",
                queuelen = 2048,

                in = {
                    name = "/dpsim1-in"

                    signals = "1c"
                },
                out = {
                    name = "/dpsim1-out"

                    signals = "1c"
                }
            }
        }

        paths = (
            {
                in = "shmem0",
                out = "shmem1",
                reverse = true
            }
        )
        """

    with open("villas-node.conf", "w") as text_file:
        text_file.write("%s" % villas_conf)

    os.system('villas-node villas-node.conf')


def dpsim0():
    sim_name = "ShmemDistributed0"
    time_step = 0.001

    n1 = dpsimpy.dp.SimNode('n1')
    n2 = dpsimpy.dp.SimNode('n2')
    n3 = dpsimpy.dp.SimNode('n3')

    evs = dpsimpy.dp.ph1.VoltageSource('v_t')
    vs = dpsimpy.dp.ph1.VoltageSourceNorton('v_s')
    l1 = dpsimpy.dp.ph1.Inductor('l_1')
    r1 = dpsimpy.dp.ph1.Resistor('r_1')

    evs.connect([dpsimpy.dp.SimNode.gnd, n3])
    vs.connect([dpsimpy.dp.SimNode.gnd, n1])
    l1.connect([n1, n2])
    r1.connect([n2, n3])

    evs.set_parameters(complex(0, 0))
    vs.set_parameters(complex(10000, 0), 1)
    l1.set_parameters(0.1)
    r1.set_parameters(1)

    sys = dpsimpy.SystemTopology(50, [n1, n2, n3], [evs, vs, l1, r1])

    dpsimpy.Logger.set_log_dir('logs/' + sim_name)

    logger = dpsimpy.Logger(sim_name)
    logger.log_attribute('v1', 'v', n1)
    logger.log_attribute('v2', 'v', n2)
    logger.log_attribute('v3', 'v', n3)

    sim = dpsimpy.RealTimeSimulation(sim_name)
    sim.set_system(sys)
    sim.set_time_step(time_step)
    sim.set_final_time(20)
    intf = dpsimpyvillas.InterfaceShmem('/dpsim0-in', '/dpsim0-out')

    sim.import_attr('v_t', 'V_ref', 0)
    sim.export_attr('v_t', 'i_intf', 0, 0, 0)

    sim.add_interface(intf)
    sim.add_logger(logger)
    sim.run(1)

def dpsim1():
    sim_name = "ShmemDistributed1"
    time_step = 0.001

    n4 = dpsimpy.dp.SimNode('n4')
    n5 = dpsimpy.dp.SimNode('n5')

    ecs = dpsimpy.dp.ph1.CurrentSource('v_s')
    r2a = dpsimpy.dp.ph1.Resistor('r_2')
    r2b = dpsimpy.dp.ph1.Resistor('r_2')
    sw = dpsimpy.dp.ph1.Switch('sw')

    ecs.connect([dpsimpy.dp.SimNode.gnd, n4])
    r2a.connect([dpsimpy.dp.SimNode.gnd, n4])
    sw.connect([n4, n5])
    r2b.connect([dpsimpy.dp.SimNode.gnd, n5])

    ecs.set_parameters(complex(0, 0))
    r2a.set_parameters(10)
    r2b.set_parameters(8)
    sw.set_parameters(open_resistance=1e9, closed_resistance=0.1, closed=False)

    sys = dpsimpy.SystemTopology(50, [n4, n5], [ecs, r2a, r2b, sw])

    dpsimpy.Logger.set_log_dir('logs/' + sim_name)

    logger = dpsimpy.Logger(sim_name)
    logger.log_attribute('v4', 'v', n4)
    logger.log_attribute('v5', 'v', n5)

    sim = dpsimpy.RealTimeSimulation(sim_name)
    sim.set_system(sys)
    sim.set_time_step(time_step)
    sim.set_final_time(20)
    intf = dpsimpyvillas.InterfaceShmem('/dpsim1-in', '/dpsim1-out')

    sim.import_attr('v_s', 'I_ref', 0)
    sim.export_attr('v_s', 'v_intf', 0, 0, 0)

    sim.add_interface(intf)
    evt = dpsimpy.event.SwitchEvent(10, sw, True)
    sim.add_event(evt)
    sim.add_logger(logger)

    sim.run(1)


if __name__ == '__main__':
    queue = Queue()
    p_villas = Process(target=villas)
    p_dpsim0 = Process(target=dpsim0)
    p_dpsim1 = Process(target=dpsim1)

    p_dpsim0.start()
    p_dpsim1.start()
    p_villas.start()
    p_dpsim0.join()
    p_dpsim1.join()

    print('Both simulations have ended!')

    work_dir0 = 'logs/ShmemDistributed0/'
    work_dir1 = 'logs/ShmemDistributed1/'
    log_name = 'ShmemDistributed'

    ts_dpsim_0 = rt.read_timeseries_dpsim(work_dir0 + log_name + '0.csv')
    ts_dpsim_1 = rt.read_timeseries_dpsim(work_dir1 + log_name + '1.csv')

    ts_dpsim_0['v1'].label = 'v1'
    ts_dpsim_0['v2'].label = 'v2'
    ts_dpsim_0['v3'].label = 'v3'
    ts_dpsim_1['v4'].label = 'v4'
    ts_dpsim_1['v5'].label = 'v5'
    pt.plot_timeseries(1, ts_dpsim_0['v1'])
    pt.plot_timeseries(1, ts_dpsim_0['v2'])
    pt.plot_timeseries(1, ts_dpsim_0['v3'])
    pt.plot_timeseries(2, ts_dpsim_1['v4'])
    pt.plot_timeseries(2, ts_dpsim_1['v5'])


    p_villas.join()
