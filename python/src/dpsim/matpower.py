import pandas as pd
import numpy as np
import scipy.io
from enum import Enum
import dpsimpy

# define dpsimpy domains
class Domain(Enum):
    PF = 1
    SP = 2
    DP = 3
    EMT = 4
    
# default multiplier for matpower data
mw_w = 1e6
kv_v = 1e3
        
class Reader:

    def __init__(self, mpc_file_path, mpc_name = 'mpc', mpc_dyn_file_path=None, mpc_dyn_name = 'mpc'):
        """
        Read input file (returns multidimensional dict)
        
        @param mpc_file_path: path to the mpc file containing the static data (normal mpc file)
        @param mpc_name: name of the struct inside of mpc_file_path
        @param mpc_dyn_file_path: Optional parameters, used for dynamic simulations in DPsim.
                                  Indicates the path to the dynamic mpc file containing the dynamic data
        @param mpc_dyn_name: name of the struct inside of mpc_dyn_file_path
        """
        self.mpc_raw = scipy.io.loadmat(mpc_file_path, simplify_cells= True)
        self.mpc_name = mpc_name
        
        self.dyn_data = False
        if (mpc_dyn_file_path is not None):
            self.dyn_data = True
            self.mpc_raw_dyn = scipy.io.loadmat(mpc_dyn_file_path, simplify_cells= True)
            self.mpc_dyn_name = mpc_dyn_name

    def process_mpc(self, frequency):
        """
        Process raw mpc data and create corresponding dataframes
        @param frequency: system frequency
        """
        
        # Version
        self.mpc_version = self.mpc_raw[self.mpc_name]['version']

        # System frequency (not included in mpc but needed for setting dpsimpy component parameters i.e inductances, capacitances ..)
        self.mpc_freq = frequency
        self.mpc_omega = 2 * np.pi * frequency

        # Base power (MVA)
        self.mpc_base_power_MVA =  self.mpc_raw[self.mpc_name]['baseMVA']

        #### Busses #####
        mpc_bus_raw = self.mpc_raw[self.mpc_name]['bus']
        bus_data_header = ["bus_i", "type", "Pd", "Qd", "Gs", "Bs", "area",
                           "Vm", "Va", "baseKV", "zone", "Vmax", "Vmin"]
        self.mpc_bus_data = pd.DataFrame(mpc_bus_raw, columns = bus_data_header)
        
        # scipy.io.loadmat loads all matrix entries as double. Convert specific columns back to int
        self.mpc_bus_data['bus_i'] = self.mpc_bus_data['bus_i'].astype(int)
        self.mpc_bus_data['type'] = self.mpc_bus_data['type'].astype(int)
        self.mpc_bus_data['area'] = self.mpc_bus_data['area'].astype(int)
        self.mpc_bus_data['zone'] = self.mpc_bus_data['zone'].astype(int)
        
        
        #### Generators #####
        mpc_gen_raw = self.mpc_raw[self.mpc_name]['gen']
        gen_data_header = ["bus", "Pg", "Qg", "Qmax", "Qmin", "Vg", "mBase", "status",
                           "Pmax", "Pmin", "Pc1", "Pc2", "Qc1min", "Qc1max", "Qc2min",
                           "Qc2max", "ramp_agc", "ramp_10", "ramp_30", "ramp_q", "apf"]
        self.mpc_gen_data = pd.DataFrame(mpc_gen_raw, columns = gen_data_header)

        # scipy.io.loadmat loads all matrix entries as double. Convert specific columns back to int
        self.mpc_gen_data['bus'] = self.mpc_gen_data['bus'].astype(int)
        self.mpc_gen_data['status'] = self.mpc_gen_data['status'].astype(int)
       
       
        #### Branches #####
        # extract only first 13 columns since following columns include results
        mpc_branch_raw = self.mpc_raw[self.mpc_name]['branch'][:, :13]
        branch_data_header = ["fbus", "tbus", "r", "x", "b", "rateA", "rateB",
                            "rateC", "ratio", "angle", "status", "angmin", "angmax"]
        self.mpc_branch_data = pd.DataFrame(mpc_branch_raw, columns = branch_data_header)

        # scipy.io.loadmat loads all matrix entries as double. Convert specific columns back to int
        self.mpc_branch_data['fbus'] = self.mpc_branch_data['fbus'].astype(int)
        self.mpc_branch_data['tbus'] = self.mpc_branch_data['tbus'].astype(int)
        self.mpc_branch_data['status'] = self.mpc_branch_data['status'].astype(int)
        
        
        #### TODO Generator costs


        #### Additional fields: bus_names, bus_assets #####
        ## Obs.: not part of the original mpc format!
        self.mpc_bus_names_dict=dict()
        self.mpc_bus_assets_dict=dict()

        if 'bus_names' in self.mpc_raw[self.mpc_name]:
            self.mpc_bus_names_dict=dict(zip(self.mpc_bus_data['bus_i'], self.mpc_raw[self.mpc_name]['bus_names']))

        if 'bus_assets' in self.mpc_raw[self.mpc_name]:
            self.mpc_bus_assets_dict=dict(zip(self.mpc_bus_data['bus_i'],self.mpc_raw[self.mpc_name]['bus_assets']))

    def process_mpc_dyn(self):
        #### Dynamic generator data
        if 'gen_dyn' in self.mpc_raw_dyn[self.mpc_dyn_name]:
            mpc_dyn_gen_data = self.mpc_raw_dyn[self.mpc_dyn_name]['gen_dyn']
            
            dyn_gen_data_header = ["bus", "model", "H", "Ra", "Xl", "Td0_t", "Td0_s", "Tq0_t", "Tq0_s", "Xd", "Xd_t", "Xd_s", "Xq", "Xq_t", "Xq_s"]
            self.mpc_dyn_gen_data = pd.DataFrame(mpc_dyn_gen_data, columns = dyn_gen_data_header)
        
            # scipy.io.loadmat loads all matrix entries as double. Convert specific columns back to int
            self.mpc_dyn_gen_data['bus'] = self.mpc_dyn_gen_data['bus'].astype(int)
            self.mpc_dyn_gen_data['model'] = self.mpc_dyn_gen_data['model'].astype(int)
            
    def create_dpsim_objects(self, domain, frequency, log_level=dpsimpy.LogLevel.info):
        
        self.log_level = log_level
        self.domain = domain
        
        # process mpc files
        self.process_mpc(frequency=frequency)
        if (self.dyn_data):
            self.process_mpc_dyn()        

        # get the correct module to be used (dpsimpy.sp, dpsimpy.dp or dpsimpy.emt) 
        dpsimpy_components = None
        if (self.domain == Domain.PF):
            dpsimpy_components = getattr(dpsimpy, "sp")
        elif (self.domain == Domain.SP):
            dpsimpy_components = getattr(dpsimpy, "sp")
        elif (self.domain == "dp"):
            dpsimpy_components = getattr(dpsimpy, Domain.DP)
        elif (self.domain == Domain.EMT):
            dpsimpy_components = getattr(dpsimpy, "emt")
        else:
            print('ERROR: domain {} is not supported in dpsimpy'.format(self.domain))
            
        # return values: nodes and components
        self.dpsimpy_busses_dict = {}
        self.dpsimpy_comp_dict = {}

        # initialize number of loads to 0
        self.loads_count = 0
        
        # initialize number of network injections to 0
        self.inj = 0

        for index, bus in self.mpc_bus_data.iterrows():
            # create dpsimpy nodes
            bus_index = str(self.mpc_bus_data.at[index,'bus_i'])
            bus_name = "N" + str(bus_index)
            bus_name = bus_index
            bus_assets= []
            if self.mpc_bus_names_dict:
                bus_name= self.mpc_bus_names_dict[int(bus_index)]
            if self.mpc_bus_assets_dict:
                if isinstance(self.mpc_bus_assets_dict[int(bus_index)], str):
                    bus_assets.append(self.mpc_bus_assets_dict[int(bus_index)])
                else:
                    bus_assets=list(self.mpc_bus_assets_dict[int(bus_index)])
            
            self.dpsimpy_busses_dict[bus_index] = dpsimpy_components.SimNode(bus_name, dpsimpy.PhaseType.Single)

            # for each bus type create corresponding dpsimpy component
            # 1 = PQ, 2 = PV, 3 = ref, 4 = isolated
            bus_type = self.mpc_bus_data.at[index,'type']

            # PQ busses
            if bus_type == 1:
                if not bus_assets: 
                    # aggregated load
                    self.map_energy_consumer(index, bus_index, dpsimpy_components, bus_type=dpsimpy.PowerflowBusType.PQ)
                else:
                    for i, comp in enumerate(bus_assets): 
                        # WARN: now the loads are created and the first load is initialized with the total power P_d Q_d!!!!!!!!                       
                        if i==0:
                            self.map_energy_consumer(index, bus_index, dpsimpy_components, load_name=comp, bus_type=dpsimpy.PowerflowBusType.PQ)
                        else:
                            load_baseV = self.mpc_bus_data.at[index,'baseKV'] * kv_v
                            self.dpsimpy_comp_dict[comp] = [dpsimpy_components.ph1.Load(comp, log_level)]
                            self.dpsimpy_comp_dict[comp][0].set_parameters(0, 0, load_baseV)
                            self.dpsimpy_comp_dict[comp].append([self.dpsimpy_busses_dict[bus_index]])

            # Generators
            elif bus_type == 2:
                # map SG
                self.map_synchronous_machine(index, bus_index, dpsimpy_components)
                
                # check if there is a load connected to PV bus (and create it)
                self.map_energy_consumer(index, bus_index, dpsimpy_components, bus_type=dpsimpy.PowerflowBusType.PV)

            # Slack bus
            elif bus_type == 3:
                #TODO: Use SG or NetworkInjection????
                # # set initial Q is not implemented for network injection in DPsim. 
                # # The mapping was changed here to test case ieee14/ieee39.
                # # This mapping does not work with asm uc because Pg and Qg of the slack are equal to zero
                # # TODO implement initial reactive power for slack in Dpsim.
                # # This way the mapping can be done exclusively with extnet componenent
                
                #self.map_network_injection(index, bus_index)
                self.map_synchronous_machine(index, bus_index, dpsimpy_components, bus_type=dpsimpy.PowerflowBusType.VD)

                # check if there is a load connected to slack bus (and create it)
                self.map_energy_consumer(index, bus_index, dpsimpy_components, bus_type=dpsimpy.PowerflowBusType.VD)
                
            #isolated
            elif bus_type == 4:
                print("isolated bus type")
            else:
                print("bus type error")

        ### branches ####
        line = 0
        trafo = 0
        for index, branch in self.mpc_branch_data.iterrows():

            branch_ratio = self.mpc_branch_data.at[index,'ratio']

            ### Distinction between lines and transformer is done now with the off nominal ratio.
            ### -> need to be changed cause the transformers have 0 off nominal ratio in the matpower use cases
            ### distinguish with busses:

            fbus = self.mpc_branch_data.at[index,'fbus']
            tbus = self.mpc_branch_data.at[index,'tbus']

            tmp_fbus = self.mpc_bus_data.loc[self.mpc_bus_data['bus_i'] == fbus]
            tmp_tbus = self.mpc_bus_data.loc[self.mpc_bus_data['bus_i'] == tbus]

            fbus_baseV = self.mpc_bus_data.at[tmp_fbus.first_valid_index(),'baseKV']*kv_v
            tbus_baseV = self.mpc_bus_data.at[tmp_tbus.first_valid_index(),'baseKV']*kv_v

            # Lines
            # if (branch_ratio == 0) and (fbus_baseV != tbus_baseV):
            if (branch_ratio == 0):
                line = line + 1
                line_name = "line%s_%s-%s" %(line, self.mpc_branch_data.at[index,'fbus'] , self.mpc_branch_data.at[index,'tbus'])

                line_fbus = self.mpc_branch_data.at[index,'fbus']
                line_tbus = self.mpc_branch_data.at[index,'tbus']

                tmp_fbus = self.mpc_bus_data.loc[self.mpc_bus_data['bus_i'] == line_fbus]
                tmp_tbus = self.mpc_bus_data.loc[self.mpc_bus_data['bus_i'] == line_tbus]

                line_fbus_baseV = self.mpc_bus_data.at[tmp_fbus.first_valid_index(),'baseKV']*kv_v
                line_tbus_baseV = self.mpc_bus_data.at[tmp_tbus.first_valid_index(),'baseKV']*kv_v

                line_baseZ = line_tbus_baseV*line_tbus_baseV / (self.mpc_base_power_MVA*mw_w)
                line_r = self.mpc_branch_data.at[index,'r'] * line_baseZ
                line_x = self.mpc_branch_data.at[index,'x'] * line_baseZ
                line_b = self.mpc_branch_data.at[index,'b'] / line_baseZ
                line_l = line_x / self.mpc_omega
                line_c = line_b / self.mpc_omega
                line_g = 0 # line conductance is not included in mpc

                self.dpsimpy_comp_dict[line_name] = [dpsimpy_components.ph1.PiLine(line_name, log_level)]
                self.dpsimpy_comp_dict[line_name][0].set_parameters(line_r, line_l, line_c, line_g)
                if (self.domain == Domain.PF):
                    self.dpsimpy_comp_dict[line_name][0].set_base_voltage(line_tbus_baseV)
                # add connections
                self.dpsimpy_comp_dict[line_name].append([self.dpsimpy_busses_dict[str(line_fbus)], self.dpsimpy_busses_dict[str(line_tbus)]])

            # Transformers
            else:
                ### Distinction between lines and transformer is done now with the off nominal ratio.
                ### -> need to be changed cause the transformers with no tap (position) have 0 off nominal ratio in the matpower use cases
                ### TODO distinguish with volatge busses (from<>to):
                branch_ratio= self.mpc_branch_data.at[index,'ratio']
                trafo = trafo + 1
                transf_name = "transformer%s_%s-%s" %(trafo, self.mpc_branch_data.at[index,'fbus'] , self.mpc_branch_data.at[index,'tbus'])
                transf_s = self.mpc_branch_data.at[index,'rateA']*mw_w # Matpower: Used to specify branch flow limits.  By default these are limits on apparent power with units in MV

                transf_fbus = self.mpc_branch_data.at[index,'fbus']
                transf_tbus = self.mpc_branch_data.at[index,'tbus']

                tmp_fbus = self.mpc_bus_data.loc[self.mpc_bus_data['bus_i'] == transf_fbus]
                tmp_tbus = self.mpc_bus_data.loc[self.mpc_bus_data['bus_i'] == transf_tbus]


                transf_fbus_baseV = self.mpc_bus_data.at[tmp_fbus.first_valid_index(),'baseKV']*kv_v
                transf_tbus_baseV = self.mpc_bus_data.at[tmp_tbus.first_valid_index(),'baseKV']*kv_v

                transf_primary_v = self.mpc_bus_data.at[tmp_fbus.first_valid_index(),'Vm']*transf_fbus_baseV
                transf_secondary_v = self.mpc_bus_data.at[tmp_tbus.first_valid_index(),'Vm']*transf_tbus_baseV

                transf_ratioAbsNominal = transf_primary_v / transf_secondary_v
                transf_ratioAbs= transf_primary_v* branch_ratio / transf_secondary_v

                # From MATPOWER-manual taps at “from”bus,  impedance at “to” bus,  i.e.  ifr=x=b= 0,tap=|Vf|/|Vt|
                # transform impedances to absolute values
                transf_baseZ = transf_tbus_baseV*transf_tbus_baseV / (self.mpc_base_power_MVA*mw_w)
                # transf_baseZ = transf_tbus_baseV*transf_tbus_baseV / (transf_s)

                # DPsim convention: impedance values must be referred to high voltage side (and base voltage set to higher voltage)
                ## refer to high voltage side and set base voltage to higher voltage
                if transf_primary_v > transf_secondary_v:
                    transf_baseV= transf_fbus_baseV
                    transf_r = self.mpc_branch_data.at[index,'r']* transf_baseZ * transf_ratioAbs**2
                    transf_x = self.mpc_branch_data.at[index,'x']* transf_baseZ * transf_ratioAbs**2
                    transf_l = transf_x / self.mpc_omega
                else:
                    transf_baseV= transf_tbus_baseV
                    transf_r = self.mpc_branch_data.at[index,'r']* transf_baseZ
                    transf_x = self.mpc_branch_data.at[index,'x']* transf_baseZ
                    transf_l = transf_x / self.mpc_omega

                self.dpsimpy_comp_dict[transf_name] = [dpsimpy_components.ph1.Transformer(transf_name, log_level)]
                self.dpsimpy_comp_dict[transf_name][0].set_parameters(transf_primary_v, transf_secondary_v, transf_s, np.abs(transf_ratioAbs), np.angle(transf_ratioAbs), transf_r, transf_l)
                # print(transf_primary_v, transf_secondary_v, transf_s, np.abs(transf_ratioAbs), np.angle(transf_ratioAbs), transf_r, transf_l)
                if (self.domain == Domain.PF):
                    self.dpsimpy_comp_dict[transf_name][0].set_base_voltage(transf_baseV)

                # add connections
                self.dpsimpy_comp_dict[transf_name].append([self.dpsimpy_busses_dict[str(transf_fbus)], self.dpsimpy_busses_dict[str(transf_tbus)]])

    def map_synchronous_machine(self, index, bus_index, dpsimpy_components, bus_type=dpsimpy.PowerflowBusType.PV):
        #
        gen_name = "Gen_N" + str(bus_index)
                
        # relevant data from self.mpc_gen_data. Identification with bus number available in mpc_bus_data and mpc_gen_data
        gen_data = self.mpc_gen_data.loc[self.mpc_gen_data['bus'] == self.mpc_bus_data.at[index,'bus_i']]
        gen_baseS = gen_data['mBase']*mw_w # gen base MVA default is mpc.baseMVA
        gen_baseV = self.mpc_bus_data.at[index,'baseKV']*kv_v # gen base kV
        gen_v = gen_data['Vg']*gen_baseV   # gen set point voltage (gen['Vg'] in p.u.)
        gen_p = gen_data['Pg']*mw_w   # gen ini. active power (gen['Pg'] in MVA)
        gen_q = gen_data['Qg']*mw_w   # gen ini. reactive power (gen['Qg'] in MVAr)
        gen_nom_s = abs(complex(gen_data['Pmax'], gen_data['Qmax'])) # gen nominal power (set default to mpc.baseMVA ? )
                
        if (self.domain == Domain.PF):
            gen = dpsimpy_components.ph1.SynchronGenerator(gen_name, self.log_level)
            gen.set_parameters(gen_nom_s, gen_baseV, gen_p, gen_v, bus_type, gen_q)
            gen.set_base_voltage(gen_baseV) 
            gen.modify_power_flow_bus_type(bus_type)          
        else:
            # get dynamic data of the generator
            gen_dyn_row_idx =  self.mpc_dyn_gen_data.index[self.mpc_dyn_gen_data['bus'] == int(bus_index)].tolist()[0]
            # TODO:throw error if len(self.mpc_dyn_gen_data.index[self.mpc_dyn_gen_data['bus'] == int(bus_index)].tolist() > 1) ?? 
            # --> two gens associated to one node...

            #
            gen_model = self.mpc_dyn_gen_data['model'][gen_dyn_row_idx]
            H = self.mpc_dyn_gen_data['H'][gen_dyn_row_idx]
            Ra = self.mpc_dyn_gen_data['Ra'][gen_dyn_row_idx]
            Ll = self.mpc_dyn_gen_data['Xl'][gen_dyn_row_idx]
            Td0_t = self.mpc_dyn_gen_data['Td0_t'][gen_dyn_row_idx]
            Td0_s = self.mpc_dyn_gen_data['Td0_s'][gen_dyn_row_idx]
            Tq0_t = self.mpc_dyn_gen_data['Tq0_t'][gen_dyn_row_idx]
            Tq0_s = self.mpc_dyn_gen_data['Tq0_s'][gen_dyn_row_idx]
            Ld = self.mpc_dyn_gen_data['Xd'][gen_dyn_row_idx]
            Ld_t = self.mpc_dyn_gen_data['Xd_t'][gen_dyn_row_idx]
            Ld_s = self.mpc_dyn_gen_data['Xd_s'][gen_dyn_row_idx]
            Lq = self.mpc_dyn_gen_data['Xq'][gen_dyn_row_idx]
            Lq_t = self.mpc_dyn_gen_data['Xq_t'][gen_dyn_row_idx]
            Lq_s = self.mpc_dyn_gen_data['Xq_s'][gen_dyn_row_idx]
                
            if (gen_model==3):
                gen = dpsimpy_components.ph1.SynchronGenerator3OrderVBR(gen_name, self.log_level)
                gen.set_operational_parameters_per_unit(nom_power=gen_nom_s, nom_voltage=gen_baseV, nom_frequency=self.frequency, 
                                                        H=H, Ld=Ld, Lq=Lq, L0=Ll, Ld_t=Ld_t, Td0_t=Td0_t)
            elif (gen_model==4):
                gen = dpsimpy_components.ph1.SynchronGenerator4OrderVBR(gen_name, self.log_level)
                gen.set_operational_parameters_per_unit(nom_power=gen_nom_s, nom_voltage=gen_baseV, nom_frequency=self.frequency, 
                                        H=H, Ld=Ld, Lq=Lq, L0=Ll, Ld_t=Ld_t, Lq_t=Lq_t, Td0_t=Td0_t, Tq0_t=Tq0_t)
            elif (gen_model==5):
                gen = dpsimpy_components.ph1.SynchronGenerator5bOrderVBR(gen_name, self.log_level)
                gen.set_operational_parameters_per_unit(nom_power=gen_nom_s, nom_voltage=gen_baseV, nom_frequency=self.frequency, 
                                        H=H, Ld=Ld, Lq=Lq, L0=Ll, Ld_t=Ld_t, Lq_t=Lq_t, Td0_t=Td0_t, Tq0_t=Tq0_t,
                                        Ld_s=Ld_s, Lq_s=Lq_s, Td0_s=Td0_s, Tq0_s=Tq0_s, Taa=0)
            elif (gen_model==6):
                gen = dpsimpy_components.ph1.SynchronGenerator6bOrderVBR(gen_name, self.log_level)
                gen.set_operational_parameters_per_unit(nom_power=gen_nom_s, nom_voltage=gen_baseV, nom_frequency=self.frequency, 
                                        H=H, Ld=Ld, Lq=Lq, L0=Ll, Ld_t=Ld_t, Lq_t=Lq_t, Td0_t=Td0_t, Tq0_t=Tq0_t,
                                        Ld_s=Ld_s, Lq_s=Lq_s, Td0_s=Td0_s, Tq0_s=Tq0_s)		
            else:
                raise Exception('Matpower reader does not support the generator model {}. Supported models are: "3", "4", "5", "6".'.format(gen_model))
                
        self.dpsimpy_comp_dict[gen_name] = [gen]
        self.dpsimpy_comp_dict[gen_name].append([self.dpsimpy_busses_dict[bus_index]]) # [to bus]
                   
    def map_energy_consumer(self, index, bus_index, dpsimpy_components, load_name=None, bus_type = dpsimpy.PowerflowBusType.PQ):
        # check if there is a load connected to PV bus
        P_d = self.mpc_bus_data.at[index,'Pd'] * mw_w
        Q_d = self.mpc_bus_data.at[index,'Qd'] * mw_w
                       
        if (bus_type != dpsimpy.PowerflowBusType.PQ):
            if (P_d==0) and (Q_d==0):
                # no load must be added
                return

        if (load_name==None):
            # increment number of loads by 1
            self.loads_count += 1
            load_name = "load%s" %self.loads_count
        
        load_p = self.mpc_bus_data.at[index,'Pd'] * mw_w
        load_q = self.mpc_bus_data.at[index,'Qd'] * mw_w
        load_baseV = self.mpc_bus_data.at[index,'baseKV'] * kv_v

        load = dpsimpy_components.ph1.Load(load_name, self.log_level)
        load.set_parameters(load_p, load_q, load_baseV)
        if (self.domain==Domain.PF and bus_type==dpsimpy.PowerflowBusType.PQ):
            load.modify_power_flow_bus_type(bus_type)
        
        self.dpsimpy_comp_dict[load_name] = [load]
        self.dpsimpy_comp_dict[load_name].append([self.dpsimpy_busses_dict[bus_index]]) # [to bus]
               
    def map_network_injection(self, index, bus_index):
        
        self.inj = self.inj + 1
        extnet_name = "extnet%s" %self.inj

        # # relevant data from self.mpc_gen_data. Identification with bus number available in mpc_bus_data and mpc_gen_data
        extnet = self.mpc_gen_data.loc[self.mpc_gen_data['bus'] == self.mpc_bus_data.at[index,'bus_i']]

        # extnet_baseS= extnet['mBase']*mw_w # default is mpc.baseMVA
        extnet_baseV = self.mpc_bus_data.at[index,'baseKV']*kv_v
        extnet_v = extnet['Vg']*extnet_baseV

        self.dpsimpy_comp_dict[extnet_name] = [dpsimpy.sp.ph1.NetworkInjection(extnet_name, self.log_level)]
        self.dpsimpy_comp_dict[extnet_name][0].set_parameters(extnet_v)
        self.dpsimpy_comp_dict[extnet_name][0].set_base_voltage(extnet_baseV)
        self.dpsimpy_comp_dict[extnet_name][0].modify_power_flow_bus_type(dpsimpy.PowerflowBusType.VD)

        # add connections
        self.dpsimpy_comp_dict[extnet_name].append([self.dpsimpy_busses_dict[bus_index]]) # [to bus]
                
                
    def load_mpc(self, frequency=60, domain=Domain.PF):
        """
        Read mpc files and create DPsim topology
        
        @param frequency: system frequency
        @param domain: domain to be used in DPsim
        """
        self.create_dpsim_objects(domain=domain, frequency=frequency)

        system_comp = []
        system_nodes = []

        for key, value in self.dpsimpy_comp_dict.items():
            dpsim_component = value[0]
            connection_nodes = value[1]
            dpsim_component.connect(connection_nodes)

            system_comp.append(dpsim_component)

            for n in connection_nodes:
                if n in system_nodes:
                    continue
                else:
                    system_nodes.append(n)

        system = dpsimpy.SystemTopology(self.mpc_freq, system_nodes, system_comp)

        return system

