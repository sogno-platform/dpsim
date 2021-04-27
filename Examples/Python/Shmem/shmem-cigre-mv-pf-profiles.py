import os
import urllib.request
import glob

import dpsimpy
import dpsimpyvillas

from multiprocessing import Process, Queue

def villas():

    villas_conf = """
        nodes = {
            broker1 = {
                type = "mqtt"

                format = "json"
                host = "172.17.0.1"

                in = {
                    subscribe = "/powerflow-dpsim"
                }
                out = {
                    publish = "/dpsim-powerflow"
                }
            }

            dpsim1 = {
                type = "shmem"

                in = {
                    name = "/dpsim-villas",	# Name of shared memory segment for sending side

                    hooks = (
                        {
                            type = "stats"
                        }
                    ),
                    signals = {
                        count = 30
                        type = "float"
                    }
                },
                out = {
                    name = "/villas-dpsim"	# Name of shared memory segment for receiving side
                }
            }
        }

        paths = (
            {
                in = "dpsim1"
                out = "broker1"

                hooks = (
                    {
                        type = "limit_rate"
                        rate = 50
                    }
                )
            }
        )"""

    with open("villas-node.conf", "w") as text_file:
        text_file.write("%s" % villas_conf)

    os.system('villas-node villas-node.conf')


def dpsim():
    name = 'CIGRE-MV-Profiles'
    files = glob.glob('build/_deps/cim-data-src/CIGRE_MV/NEPLAN/CIGRE_MV_no_tapchanger_With_LoadFlow_Results/*.xml')
    print(files)

    reader = dpsimpy.CIMReader(name)
    system = reader.loadCIM(50, files, dpsimpy.Domain.SP, dpsimpy.PhaseType.Single, dpsimpy.GeneratorType.PVNode))

    csv_files = glob.glob('build/_deps/profile-data-src/CIGRE_MV_NoTap/load_profiles/')[0]
    print(csv_files)

    assignList = { }
    assignList['LOAD-H-1'] = 'Load_H_1'
    assignList['LOAD-H-3'] = 'Load_H_3'
    assignList['LOAD-H-4'] = 'Load_H_4'
    assignList['LOAD-H-5'] = 'Load_H_5'
    assignList['LOAD-H-6'] = 'Load_H_6'
    assignList['LOAD-H-8'] = 'Load_H_8'
    assignList['LOAD-H-10'] = 'Load_H_10'
    assignList['LOAD-H-11'] = 'Load_H_11'
    assignList['LOAD-H-12'] = 'Load_H_12'
    assignList['LOAD-H-14'] = 'Load_H_14'
    assignList['LOAD-I-1'] = 'Load_I_1'
    assignList['LOAD-I-3'] = 'Load_I_3'
    assignList['LOAD-I-7'] = 'Load_I_7'
    assignList['LOAD-I-9'] = 'Load_I_9'
    assignList['LOAD-I-10'] = 'Load_I_10'
    assignList['LOAD-I-12'] = 'Load_I_12'
    assignList['LOAD-I-13'] = 'Load_I_13'
    assignList['LOAD-I-14'] = 'Load_I_14'

    csvreader = dpsimpy.CSVReader(name, csv_files, assignList, dpsimpy.LogLevel.info)
    csvreader.assignLoadProfile(system, 0, 1, 300, dpsimpy.CSVReaderMode.MANUAL, dpsimpy.CSVReaderFormat.SECONDS)

    sim = dpsimpy.RealTimeSimulation(name)
    sim.set_system(system)
    sim.set_domain(dpsimpy.Domain.SP)
    sim.set_solver(dpsimpy.Solver.NRP)
    sim.set_time_step(1)
    sim.set_final_time(30)

    logger = dpsimpy.Logger(name)
    sim.add_logger(logger)

    intf = dpsimpyvillas.InterfaceShmem()
    sim.add_interface(intf)

    obj_list = system.list_idobjects()
    node_list = {k: v for k, v in obj_list.items() if v == 'SimNode<std::complex<double> >'}
    print(node_list)

    for i in range(15):
        objname = 'N'+str(i)
        sim.export_attr(objname, 'v', (i*2), dpsimpy.AttrModifier.mag)
        sim.export_attr(objname, 'v', (i*2)+1, dpsimpy.AttrModifier.phase)
        print(objname)

    for node in system.nodes:
        logger.log_attribute(node.name()+'.V', 'v', node)

    sim.run(1)


if __name__ == '__main__':
    queue = Queue()
    p_villas = Process(target=villas)
    p_dpsim = Process(target=dpsim)

    p_dpsim.start()
    p_villas.start()
    p_dpsim.join()
    p_villas.join()