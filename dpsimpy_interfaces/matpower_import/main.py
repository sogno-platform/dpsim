import pandas as pd
import numpy as np
from mpc_objects import mpcObjects
from dpsimpy_objects import *
from results import *
import sys
sys.path.append('/dpsim/build') #add build directory (where dpsimpy is located) to sys.path 
import dpsimpy

## mpc ".mat" file path
mat_file_path= '/dpsim/dpsimpy_interfaces/matpower_import/test_cases'
mat_file_name='case9.mat'
mpc_objects = mpcObjects(mat_file_path, mat_file_name)
mpc_objects.process_mpc_raw()

## Create dpsimpy system
dpsimpy_busses_dict, dpsimpy_comp_dict, system = create_dpsimpy_system(mpc_objects)

## Run dpsim simulation
# Logging of bus complex voltage and power
sim_name = mat_file_name[:-4]
dpsimpy.Logger.set_log_dir('logs/' + sim_name)

logger = dpsimpy.Logger(sim_name)
for node in system.nodes:    
    logger.log_attribute(node.name()+'.V', 'v', node)
    logger.log_attribute(node.name()+'.S', 's', node)

# Parametrize and run simulation
sim = dpsimpy.Simulation(sim_name, dpsimpy.LogLevel.info)
sim.set_system(system)
sim.set_time_step(1)
sim.set_final_time(1)
sim.set_domain(dpsimpy.Domain.SP)
sim.set_solver(dpsimpy.Solver.NRP)
sim.do_init_from_nodes_and_terminals(False)
sim.add_logger(logger)
sim.run()

## Power flow results
results= results(sim_name, system, mpc_objects)