# To run this example, an MQTT broker is required
# docker run -it -p 1883:1883 -p 9001:9001 eclipse-mosquitto
# test message: mosquitto_pub -t 'test/topic' -m "test"
# get messages: mosquitto_sub -v -t '#'

import glob
import sys
import os.path
import logging
import json

from datetime import datetime
from villas.node.node import Node as VILLASnode

import dpsimpy
import dpsimpyvillas

base = os.path.splitext(os.path.basename(sys.argv[0]))[0]
log = logging.getLogger(base)

def villas(intf, mqtt=False):

    log_filename=datetime.now().strftime(f'{base}-villas-%y-%m-%d_%H_%M_%S.log')

    nodes = {
        'dpsim1': intf.get_config(),
        'file1': {
            'type': 'file',

            'uri': f'{base}-results-%y-%m-%d_%H_%M_%S.csv'
        }
    }

    paths = [
        {
            'in': 'dpsim1',
            'out': 'file1'
        }
    ]

    config = {
        'nodes': nodes,
        'paths': paths
    }

    if mqtt:
        nodes['broker1'] = {
            'type': 'mqtt',

            'format': 'json',
            'host': '172.17.0.1',

            'in': {
                'subscribe': '/powerflow-dpsim'
            },
            'out': {
                'publish': '/dpsim-powerflow'
            }
        }

        paths.append({
            'in': 'dpsim1',
            'out': 'broker1',

            'hooks': [
                {
                    'type': 'limit_rate',
                    'rate': 50
                }
            ]
        })

    log.info('VILLASnode config: \n%s', json.dumps(config, indent=2))

    return VILLASnode(config=config,
                      log_filename=log_filename)

def dpsim():
    name = 'CIGRE-MV-Profiles'
    files = glob.glob('build/_deps/cim-data-src/CIGRE_MV/NEPLAN/CIGRE_MV_no_tapchanger_With_LoadFlow_Results/*.xml')
    log.info('CIM files: %s', files)

    reader = dpsimpy.CIMReader()
    system = reader.loadCIM(50, files, dpsimpy.Domain.SP, dpsimpy.PhaseType.Single, dpsimpy.GeneratorType.PVNode)

    csv_files = glob.glob('build/_deps/profile-data-src/CIGRE_MV_NoTap/load_profiles/')[0]
    log.info('CSV files: %s', csv_files)

    assignList = {
        'LOAD-H-1': 'Load_H_1',
        'LOAD-H-3': 'Load_H_3',
        'LOAD-H-4': 'Load_H_4',
        'LOAD-H-5': 'Load_H_5',
        'LOAD-H-6': 'Load_H_6',
        'LOAD-H-8': 'Load_H_8',
        'LOAD-H-10': 'Load_H_10',
        'LOAD-H-11': 'Load_H_11',
        'LOAD-H-12': 'Load_H_12',
        'LOAD-H-14': 'Load_H_14',
        'LOAD-I-1': 'Load_I_1',
        'LOAD-I-3': 'Load_I_3',
        'LOAD-I-7': 'Load_I_7',
        'LOAD-I-9': 'Load_I_9',
        'LOAD-I-10': 'Load_I_10',
        'LOAD-I-12': 'Load_I_12',
        'LOAD-I-13': 'Load_I_13',
        'LOAD-I-14': 'Load_I_14'
    }

    csvreader = dpsimpy.CSVReader(csv_files, assignList, dpsimpy.LogLevel.info)
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
    log.info('Node list: %s', node_list)

    for i in range(15):
        objname = 'N'+str(i)
        sim.export_attribute(sim.get_idobj_attr(objname, 'v')
            .derive_coeff(0,0)
            .derive_mag(), (i*2))
        sim.export_attribute(sim.get_idobj_attr(objname, 'v')
            .derive_coeff(0,0)
            .derive_phase(), (i*2)+1)

    for node in system.nodes:
        logger.log_attribute(node.name()+'.V', 'v', node)

    return sim, intf

def test_shmem_cigre_mv_pf_profiles():
    logging.basicConfig(format='[%(asctime)s %(name)s %(levelname)s] %(message)s', datefmt='%H:%M:%S', level=logging.INFO)

    sim, intf = dpsim()
    node = villas(intf, mqtt=False)

    node.start()

    sim.run(1)

    node.stop()

if __name__ == '__main__':
    test_shmem_cigre_mv_pf_profiles()
