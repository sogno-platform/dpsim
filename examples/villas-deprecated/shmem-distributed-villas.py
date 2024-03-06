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

    os.system("villas-node villas-node.conf")


def dpsim0():
    sim_name = "ShmemDistributed0"
    time_step = 0.001
    final_time = 10

    n1 = dpsimpy.dp.SimNode("n1", dpsimpy.PhaseType.Single, [10])
    n2 = dpsimpy.dp.SimNode("n2", dpsimpy.PhaseType.Single, [5])

    evs = dpsimpy.dp.ph1.VoltageSource("v_intf", dpsimpy.LogLevel.debug)
    evs.set_parameters(complex(5, 0))

    vs1 = dpsimpy.dp.ph1.VoltageSource("vs_1", dpsimpy.LogLevel.debug)
    vs1.set_parameters(complex(10, 0))

    r12 = dpsimpy.dp.ph1.Resistor("r_12", dpsimpy.LogLevel.debug)
    r12.set_parameters(1)

    evs.connect([dpsimpy.dp.SimNode.gnd, n2])
    vs1.connect([dpsimpy.dp.SimNode.gnd, n1])
    r12.connect([n1, n2])

    sys = dpsimpy.SystemTopology(50, [n1, n2], [evs, vs1, r12])

    dpsimpy.Logger.set_log_dir("logs/" + sim_name)

    logger = dpsimpy.Logger(sim_name)
    logger.log_attribute("v1", "v", n1)
    logger.log_attribute("v2", "v", n2)
    logger.log_attribute("r12", "i_intf", r12)
    logger.log_attribute("ievs", "i_intf", evs)
    logger.log_attribute("vevs", "v_intf", evs)

    sim = dpsimpy.RealTimeSimulation(sim_name)
    sim.set_system(sys)
    sim.set_time_step(time_step)
    sim.set_final_time(final_time)

    intf = dpsimpyvillas.InterfaceShmem("/dpsim0-out", "/dpsim0-in")

    sim.import_attribute(evs.attr("V_ref"), 0)
    sim.export_attribute(evs.attr("i_intf").derive_coeff(0, 0), 0)

    sim.add_interface(intf, True)
    sim.add_logger(logger)

    evs.set_intf_current([[complex(5, 0)]])

    sim.run(1)


def dpsim1():
    sim_name = "ShmemDistributed1"
    time_step = 0.001
    final_time = 10

    n2 = dpsimpy.dp.SimNode("n2", dpsimpy.PhaseType.Single, [5])

    ecs = dpsimpy.dp.ph1.CurrentSource("i_intf", dpsimpy.LogLevel.debug)
    ecs.set_parameters(complex(5, 0))
    r02 = dpsimpy.dp.ph1.Resistor("r_02", dpsimpy.LogLevel.debug)
    r02.set_parameters(1)

    ecs.connect([dpsimpy.dp.SimNode.gnd, n2])
    r02.connect([dpsimpy.dp.SimNode.gnd, n2])

    sys = dpsimpy.SystemTopology(50, [n2], [ecs, r02])

    dpsimpy.Logger.set_log_dir("logs/" + sim_name)

    logger = dpsimpy.Logger(sim_name)
    logger.log_attribute("v2", "v", n2)
    logger.log_attribute("r02", "i_intf", r02)
    logger.log_attribute("vecs", "v_intf", ecs)
    logger.log_attribute("iecs", "i_intf", ecs)

    sim = dpsimpy.RealTimeSimulation(sim_name)
    sim.set_system(sys)
    sim.set_time_step(time_step)
    sim.set_final_time(final_time)
    intf = dpsimpyvillas.InterfaceShmem("/dpsim1-out", "/dpsim1-in")

    sim.import_attribute(ecs.attr("I_ref"), 0)
    sim.export_attribute(
        ecs.attr("v_intf").derive_coeff(0, 0).derive_scaled(complex(-1, 0)), 0
    )

    sim.add_interface(intf)
    sim.add_logger(logger)

    sim.run(1)


if __name__ == "__main__":
    queue = Queue()
    p_villas = Process(target=villas)
    p_dpsim0 = Process(target=dpsim0)
    p_dpsim1 = Process(target=dpsim1)

    p_dpsim0.start()
    p_dpsim1.start()
    p_villas.start()

    p_dpsim0.join()
    p_dpsim1.join()

    print("Both simulations have ended!")

    p_villas.join()
