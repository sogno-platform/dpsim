import pandas as pd
import numpy as np
import scipy.io
import os

class mpcObjects:
   
    def __init__(self, input_file_dir, input_file_name):
        #join path and name of .mat file containing the mpc struct
        self.mpc_input_file = os.path.join(input_file_dir, input_file_name)
        #read input file (returns multidimensional dict)
        self.mpc_raw= scipy.io.loadmat(self.mpc_input_file)

    def process_mpc_raw(self):

        version_idx= 0
        base_pow_idx= 1
        bus_data_idx= 2
        gen_data_idx= 3
        branch_data_idx= 4
        # gencost_data_idx= 5

        ## Process raw mpc data and create corresponding dataframes
        #Version
        self.mpc_version= self.mpc_raw['mpc'][0][0][version_idx]

        #System frequency (not included in mpc but needed for setting dpsimpy component parameters i.e inductances, capacitances ..)
        self.mpc_freq= 50
        self.mpc_omega= 2*np.pi*50

        #Base power (MVA)
        self.mpc_base_power_MVA=self.mpc_raw['mpc'][0][0][base_pow_idx][0][0]

        #### Busses #####
        mpc_bus_raw=self.mpc_raw['mpc'][0][0][bus_data_idx]

        bus_data_header= ["bus_i", "type", "Pd", "Qd", "Gs", "Bs", "area", 
                        "Vm", "Va", "baseKV", "zone", "Vmax", "Vmin"]
        
        self.mpc_bus_data= pd.DataFrame(mpc_bus_raw, columns = bus_data_header)

        #scipy.io.loadmat loads all matrix entries as double. Convert specific columns back to int
        self.mpc_bus_data['bus_i'] = self.mpc_bus_data['bus_i'].astype(int)
        self.mpc_bus_data['type'] = self.mpc_bus_data['type'].astype(int)
        self.mpc_bus_data['area'] = self.mpc_bus_data['area'].astype(int)
        self.mpc_bus_data['zone'] = self.mpc_bus_data['zone'].astype(int)
        
        #### Generators
        mpc_gen_raw=self.mpc_raw['mpc'][0][0][gen_data_idx]
        
        gen_data_header=["bus", "Pg", "Qg", "Qmax", "Qmin", "Vg", "mBase", "status",
                        "Pmax", "Pmin", "Pc1", "Pc2", "Qc1min", "Qc1max", "Qc2min", 
                        "Qc2max", "ramp_agc", "ramp_10", "ramp_30", "ramp_q", "apf"]

        self.mpc_gen_data = pd.DataFrame(mpc_gen_raw, columns = gen_data_header)

        self.mpc_gen_data['bus'] = self.mpc_gen_data['bus'].astype(int)
        self.mpc_gen_data['status'] = self.mpc_gen_data['status'].astype(int)
        
        #### Branches
        mpc_branch_raw=self.mpc_raw['mpc'][0][0][branch_data_idx]

        branch_data_header=["fbus", "tbus", "r", "x", "b", "rateA", "rateB", 
                            "rateC", "ratio", "angle", "status", "angmin", "angmax"]

        self.mpc_branch_data = pd.DataFrame(mpc_branch_raw, columns = branch_data_header)

        self.mpc_branch_data['fbus'] = self.mpc_branch_data['fbus'].astype(int)
        self.mpc_branch_data['tbus'] = self.mpc_branch_data['tbus'].astype(int)
        self.mpc_branch_data['status'] = self.mpc_branch_data['status'].astype(int)

        #### Generator costs
        ## tbd
    







