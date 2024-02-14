# Adapted from shmem_cigre_mv_pf_profiles using the new attribute system

import glob
import logging
import os
import sys

import dpsimpy
import dpsimpyvillas

# Setup
name = 'CIGRE-MV-Profiles'
time_step = 1
final_time = 30

# Setup stdout logger
base = os.path.splitext(os.path.basename(sys.argv[0]))[0]
log = logging.getLogger(base)

# Read topology from CIM
files = glob.glob('build/_deps/cim-data-src/CIGRE_MV/NEPLAN/CIGRE_MV_no_tapchanger_With_LoadFlow_Results/*.xml') # Downloaded by CMake
log.info('CIM files: %s', files)
reader = dpsimpy.CIMReader(name)
system = reader.loadCIM(50, files, dpsimpy.Domain.SP, dpsimpy.PhaseType.Single, dpsimpy.GeneratorType.PVNode)

# Map from CSV to simulation names
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

# Read profiles from CSV
csv_files = 'build/_deps/profile-data-src/CIGRE_MV_NoTap/load_profiles/'
log.info('CSV files: %s', csv_files)
csvreader = dpsimpy.CSVReader(name, csv_files, assignList, dpsimpy.LogLevel.info)
csvreader.assignLoadProfile(system, 0, 1, 300, dpsimpy.CSVReaderMode.MANUAL, dpsimpy.CSVReaderFormat.SECONDS)

# Instantiate logger
logger = dpsimpy.Logger(name)

# setup VILLASnode
intf_mqtt = dpsimpyvillas.InterfaceVillas(name='MQTT', config='''{
    "type": "mqtt",
    "host": "mqtt",
    "in": {
        "subscribe": "mqtt-dpsim"
    },
    "out": {
        "publish": "dpsim-mqtt"
    }
}''')

# Setup simulation
sim = dpsimpy.RealTimeSimulation(name)
sim.set_system(system)
sim.set_domain(dpsimpy.Domain.SP)
sim.set_solver(dpsimpy.Solver.NRP)
sim.set_time_step(time_step)
sim.set_final_time(final_time)
sim.add_logger(logger)
sim.add_interface(intf_mqtt)

# Setup exports
for i in range(15):
    objname = 'N'+str(i)
    intf_mqtt.export_attribute(sim \
        .get_idobj_attr(objname, 'v') \
        .derive_coeff(0,0) \
        .derive_mag(), 2*i)
    intf_mqtt.export_attribute(sim \
        .get_idobj_attr(objname, 'v') \
        .derive_coeff(0,0) \
        .derive_phase(), 2*i+1)

# Log exports
for node in system.nodes:
    sim.log_idobj_attribute(node.name(), 'v')

sim.run(1)
