# This example reads a sine signal from villas that modifies the active power set point
# of the PQ load. The n2 voltage is exported to a file via MQTT and VILLASnode.

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

def villas():

    # log_filename=datetime.now().strftime(f'{base}-villas-%y-%m-%d_%H_%M_%S.log')

    # nodes = {
    #     'mqtt0': {
    #         'type': 'mqtt',
    #         'format': 'json',
    #         'host': 'mqtt',
    #         'in': {
    #             'subscribe': '/dpsim0-mqtt0',
    #             'signals':  [
    #                 {
    #                     'name': 'i_intf',
    #                     'type': 'complex'
    #                 }
    #             ]
    #         },
    #         'out': {
    #             'publish': '/mqtt0-dpsim0'
    #         }
    #     },
    #     'mqtt1': {
    #         'type': 'mqtt',
    #         'format': 'json',
    #         'host': 'mqtt',
    #         'in': {
    #             'subscribe': '/dpsim1-mqtt1',
    #             'signals':  [
    #                 {
    #                     'name': 'v_intf',
    #                     'type': 'complex'
    #                 }
    #             ]
    #         },
    #         'out': {
    #             'publish': '/mqtt1-dpsim1'
    #         }
    #     }
    # }

    # paths = [
    #     {
    #         'in': 'mqtt0',
    #         'out': 'mqtt1',
    #         'reverse': True,
    #         'hooks': [{'type':'print'}]
    #     }
    # ]

    # config = {
    #     'nodes': nodes,
    #     'paths': paths
    # }
    # config['nodes']['mqtt0']['out']['hooks'] = [{'type':'print'}]
    # config['nodes']['mqtt1']['out']['hooks'] = [{'type':'print'}]

    # nodes = {
    #     'mqtt0': {
    #         'type': 'mqtt',
    #         'format': 'json',
    #         'host': 'mqtt',
    #         'in': {
    #             'subscribe': '/dpsim0-mqtt0',
    #             'signals':  [
    #                 {
    #                     'name': 'i_intf',
    #                     'type': 'complex'
    #                 }
    #             ]
    #         },
    #         'out': {
    #             'publish': '/mqtt0-dpsim0'
    #         }
    #     },
    #     'mqtt1': {
    #         'type': 'mqtt',
    #         'format': 'json',
    #         'host': 'mqtt',
    #         'in': {
    #             'subscribe': '/dpsim1-mqtt1',
    #             'signals':  [
    #                 {
    #                     'name': 'v_intf',
    #                     'type': 'complex'
    #                 }
    #             ]
    #         },
    #         'out': {
    #             'publish': '/mqtt1-dpsim1'
    #         }
    #     },
    #     'file1': {
    #         'type': 'file',
    #         'uri': f'{base}-results-%y-%m-%d_%H_%M_%S.csv'
    #     }
    # }

    # paths = [
    #     {
    #         'in': 'mqtt1',
    #         'out': 'file1',
    #         'hooks': [{'type':'print'}]
    #     }
    # ]

    # config = {
    #     'nodes': nodes,
    #     'paths': paths
    # }
    # config['nodes']['mqtt0']['out']['hooks'] = [{'type':'print'}]

    # log.info('VILLASnode config: \n%s', json.dumps(config, indent=2))

    # node = VILLASnode(config=config, log_filename=log_filename)
    # node.start()

    # return VILLASnode(config=config, log_filename=log_filename)

    villas_conf = """
        {
        "nodes": {
            "mqtt0": {
            "type": "mqtt",
            "format": "json",
            "host": "mqtt",
            "in": {
                "subscribe": "/dpsim0-mqtt0",
                "signals": [
                {
                    "name": "i_intf",
                    "type": "complex"
                }
                ]
            },
            "out": {
                "publish": "/mqtt0-dpsim0",
                "hooks": [
                {
                    "type": "print"
                }
                ]
            }
            },
            "mqtt1": {
            "type": "mqtt",
            "format": "json",
            "host": "mqtt",
            "in": {
                "subscribe": "/dpsim1-mqtt1",
                "signals": [
                {
                    "name": "v_intf",
                    "type": "complex"
                }
                ]
            },
            "out": {
                "publish": "/mqtt1-dpsim1",
                "hooks": [
                {
                    "type": "print"
                }
                ]
            }
            }
        },
        "paths": [
            {
            "in": "mqtt0",
            "out": "mqtt1",
            "hooks": [
                {
                "type": "print"
                }
            ]
            },
            {
            "in": "mqtt1",
            "out": "mqtt0",
            "hooks": [
                {
                "type": "print"
                }
            ]
            }
        ]
        }
        """

    with open("villas-node.conf", "w") as text_file:
        text_file.write("%s" % villas_conf)

    os.system('villas-node villas-node.conf')

def dpsim0():
    sim_name = "DistributedVILLAS0"
    time_step = 0.01
    final_time = 10

    n1 = dpsimpy.dp.SimNode('n1', dpsimpy.PhaseType.Single, [10])
    n2 = dpsimpy.dp.SimNode('n2', dpsimpy.PhaseType.Single, [5])

    evs = dpsimpy.dp.ph1.VoltageSource('v_intf', dpsimpy.LogLevel.debug)
    evs.set_parameters(complex(5, 0))

    vs1 = dpsimpy.dp.ph1.VoltageSource('vs_1', dpsimpy.LogLevel.debug)
    vs1.set_parameters(complex(10, 0))

    r12 = dpsimpy.dp.ph1.Resistor('r_12', dpsimpy.LogLevel.debug)
    r12.set_parameters(1)

    evs.connect([dpsimpy.dp.SimNode.gnd, n2])
    vs1.connect([dpsimpy.dp.SimNode.gnd, n1])
    r12.connect([n1, n2])

    sys = dpsimpy.SystemTopology(50, [n1, n2], [evs, vs1, r12])

    dpsimpy.Logger.set_log_dir('logs/' + sim_name)

    logger = dpsimpy.Logger(sim_name)
    logger.log_attribute('v1', 'v', n1)
    logger.log_attribute('v2', 'v', n2)
    logger.log_attribute('r12', 'i_intf', r12)
    logger.log_attribute('ievs', 'i_intf', evs)
    logger.log_attribute('vevs', 'v_intf', evs)

    sim = dpsimpy.RealTimeSimulation(sim_name)
    sim.set_system(sys)
    sim.set_time_step(time_step)
    sim.set_final_time(final_time)

    intf_config = {
        "type": "mqtt",
        "format": "json",
        "host": "mqtt",
        "in": {
            "subscribe": "/mqtt0-dpsim0"
        },
        "out": {
            "publish": "/dpsim0-mqtt0"
        }
    }

    # file_config = '''{
    #     "type": "file",
    #     "format": "csv",
    #     "uri": "dpsim-mqtt-distributed-villas-results-22-09-12_16_27_53.csv",
    #     "in": {
    #         "signals": [
    #             {
    #                 "name": "v_intf",
    #                 "type": "complex"
    #             }
    #         ]
    #     }
    # }'''
    
    intf = dpsimpyvillas.InterfaceVillas(name="dpsim0-mqtt0", config=intf_config)
    # intf = dpsimpyvillas.InterfaceVillas(name="file-dpsim0", config=file_config)

    sim.add_interface(intf, True)
    sim.import_attribute(evs.attr('V_ref'), 0)
    sim.export_attribute(evs.attr('i_intf').derive_coeff(0,0), 0)
  
    sim.run(1)
    # return sim, intf

def dpsim1():
    sim_name = "DistributedVILLAS1"
    time_step = 0.01
    final_time = 10

    n2 = dpsimpy.dp.SimNode('n2', dpsimpy.PhaseType.Single, [5])

    ecs = dpsimpy.dp.ph1.CurrentSource('i_intf', dpsimpy.LogLevel.debug)
    ecs.set_parameters(complex(5, 0))
    r02 = dpsimpy.dp.ph1.Resistor('r_02', dpsimpy.LogLevel.debug)
    r02.set_parameters(1)

    ecs.connect([dpsimpy.dp.SimNode.gnd, n2])
    r02.connect([dpsimpy.dp.SimNode.gnd, n2])

    sys = dpsimpy.SystemTopology(50, [n2], [ecs, r02])

    dpsimpy.Logger.set_log_dir('logs/' + sim_name)

    logger = dpsimpy.Logger(sim_name)
    logger.log_attribute('v2', 'v', n2)
    logger.log_attribute('r02', 'i_intf', r02)
    logger.log_attribute('vecs', 'v_intf', ecs)
    logger.log_attribute('iecs', 'i_intf', ecs)

    sim = dpsimpy.RealTimeSimulation(sim_name)
    sim.set_system(sys)
    sim.set_time_step(time_step)
    sim.set_final_time(final_time)

    intf_config = {
        "type": "mqtt",
        "format": "json",
        "host": "mqtt",
        "in": {
            "subscribe": "/mqtt1-dpsim1"
        },
        "out": {
            "publish": "/dpsim1-mqtt1"
        }
    }

    intf = dpsimpyvillas.InterfaceVillas(name="dpsim1-mqtt1", config=intf_config)
    sim.add_interface(intf, True)
    sim.import_attribute(ecs.attr('I_ref'), 0)
    sim.export_attribute(ecs.attr('v_intf').derive_coeff(0,0).derive_scaled(complex(-1,0)), 0)
  
    sim.run(1)

    # return sim, intf

if __name__ == '__main__':
    logging.basicConfig(format='[%(asctime)s %(name)s %(levelname)s] %(message)s', datefmt='%H:%M:%S', level=logging.DEBUG)

    # p_node = villas()
    p_node = Process(target=villas)
    p_sim0 = Process(target=dpsim0)
    p_sim1 = Process(target=dpsim1)
    
    p_node.start()
    p_sim0.start()
    p_sim1.start()

    p_sim0.join()
    p_sim1.join()

    print('Both simulations have ended!')

    p_node.join()
    # p_node.stop()

    # sim, intf = dpsim0() #intf needs to be extracted from the dpsim-function since the interface object gets deleted otherwise leading to SegFault when starting the simulation
    # # node = villas()
    
    # # node.start()
    # sim.run(1)
    # # node.stop()