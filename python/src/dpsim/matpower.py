import pandas as pd
import numpy as np
import scipy.io
import sys
sys.path.append('/dpsim/build')
import dpsimpy

class Reader:

    def __init__(self, mpc_file_path, mpc_name = 'mpc'):
        # read input file (returns multidimensional dict)
        self.mpc_raw = scipy.io.loadmat(mpc_file_path, simplify_cells= True)
        self.mpc_name = mpc_name

    def process_mpc(self):

        # Process raw mpc data and create corresponding dataframes
        # Version
        self.mpc_version = self.mpc_raw[self.mpc_name]['version']

        # System frequency (not included in mpc but needed for setting dpsimpy component parameters i.e inductances, capacitances ..)
        self.mpc_freq = 50
        self.mpc_omega = 2*np.pi*50

        # Base power (MVA)
        self.mpc_base_power_MVA =  self.mpc_raw[self.mpc_name]['baseMVA']

        #### Busses
        mpc_bus_raw = self.mpc_raw[self.mpc_name]['bus']

        bus_data_header = ["bus_i", "type", "Pd", "Qd", "Gs", "Bs", "area",
                           "Vm", "Va", "baseKV", "zone", "Vmax", "Vmin"]

        self.mpc_bus_data = pd.DataFrame(mpc_bus_raw, columns = bus_data_header)

        # scipy.io.loadmat loads all matrix entries as double. Convert specific columns back to int
        self.mpc_bus_data['bus_i'] = self.mpc_bus_data['bus_i'].astype(int)
        self.mpc_bus_data['type'] = self.mpc_bus_data['type'].astype(int)
        self.mpc_bus_data['area'] = self.mpc_bus_data['area'].astype(int)
        self.mpc_bus_data['zone'] = self.mpc_bus_data['zone'].astype(int)

        #### Generators
        mpc_gen_raw = self.mpc_raw[self.mpc_name]['gen']

        gen_data_header = ["bus", "Pg", "Qg", "Qmax", "Qmin", "Vg", "mBase", "status",
                           "Pmax", "Pmin", "Pc1", "Pc2", "Qc1min", "Qc1max", "Qc2min",
                           "Qc2max", "ramp_agc", "ramp_10", "ramp_30", "ramp_q", "apf"]

        self.mpc_gen_data = pd.DataFrame(mpc_gen_raw, columns = gen_data_header)

        self.mpc_gen_data['bus'] = self.mpc_gen_data['bus'].astype(int)
        self.mpc_gen_data['status'] = self.mpc_gen_data['status'].astype(int)

        #### Branches
        # extract only first 13 columns since following columns include results
        mpc_branch_raw = self.mpc_raw[self.mpc_name]['branch'][:, :13]

        branch_data_header = ["fbus", "tbus", "r", "x", "b", "rateA", "rateB",
                            "rateC", "ratio", "angle", "status", "angmin", "angmax"]

        self.mpc_branch_data = pd.DataFrame(mpc_branch_raw, columns = branch_data_header)

        self.mpc_branch_data['fbus'] = self.mpc_branch_data['fbus'].astype(int)
        self.mpc_branch_data['tbus'] = self.mpc_branch_data['tbus'].astype(int)
        self.mpc_branch_data['status'] = self.mpc_branch_data['status'].astype(int)

        #### TODO Generator costs

        #### additional fields: bus_names, bus_assets #####
        self.mpc_bus_names_dict=dict()
        self.mpc_bus_assets_dict=dict()

        if 'bus_names' in self.mpc_raw[self.mpc_name]:
            self.mpc_bus_names_dict=dict(zip(self.mpc_bus_data['bus_i'], self.mpc_raw[self.mpc_name]['bus_names']))

        if 'bus_assets' in self.mpc_raw[self.mpc_name]:
            self.mpc_bus_assets_dict=dict(zip(self.mpc_bus_data['bus_i'],self.mpc_raw[self.mpc_name]['bus_assets']))

    def create_dpsim_objects(self):

        self.process_mpc()

        # return values: nodes and components
        dpsimpy_busses_dict = {}
        dpsimpy_comp_dict = {}

        # default multiplier for matpower data
        mw_w = 1e6
        kv_v = 1e3

        # Nodes
        bus = 0
        load = 0
        generator = 0
        inj = 0

        for index, bus in self.mpc_bus_data.iterrows():
            # create dpsimpy nodes
            bus = bus + 1
            bus_index = str(self.mpc_bus_data.at[index,'bus_i'])
            bus_name = bus_index
            bus_assets= []
            if self.mpc_bus_names_dict:
                bus_name= self.mpc_bus_names_dict[int(bus_index)]
            if self.mpc_bus_assets_dict:
                if isinstance(self.mpc_bus_assets_dict[int(bus_index)], str):
                    bus_assets.append(self.mpc_bus_assets_dict[int(bus_index)])
                else:
                    bus_assets=list(self.mpc_bus_assets_dict[int(bus_index)])
            
            dpsimpy_busses_dict[bus_index] = dpsimpy.sp.SimNode(bus_name, dpsimpy.PhaseType.Single)

            # for each bus type create corresponding dpsimpy component
            # 1 = PQ, 2 = PV, 3 = ref, 4 = isolated
            bus_type = self.mpc_bus_data.at[index,'type']

            # PQ busses
            if bus_type == 1:
                if not bus_assets: #an aggregated load is automatically generated for PQ busses even if P_d and Q_d are equal to zero
                        load = load + 1
                        # load_name = "load%s" %bus_index
                        load_name='aggregated Load %s' %load
                        load_p = self.mpc_bus_data.at[index,'Pd'] * mw_w
                        load_q = self.mpc_bus_data.at[index,'Qd'] * mw_w
                        load_baseV = self.mpc_bus_data.at[index,'baseKV'] * kv_v

                        dpsimpy_comp_dict[load_name] = [dpsimpy.sp.ph1.Load(load_name, dpsimpy.LogLevel.info)]
                        dpsimpy_comp_dict[load_name][0].set_parameters(load_p, load_q, load_baseV)
                        dpsimpy_comp_dict[load_name][0].modify_power_flow_bus_type(dpsimpy.PowerflowBusType.PQ)
                        # add connections
                        dpsimpy_comp_dict[load_name].append([dpsimpy_busses_dict[bus_index]]) # [to bus]
                else:
                    for i, comp in enumerate(bus_assets): # WARN: now the loads are created and the first load is initialized with the total power P_d Q_d!!!!!!!!
                        load = load + 1
                        # load_name = "load%s" %bus_index
                        load_name=comp
                        load_p = self.mpc_bus_data.at[index,'Pd'] * mw_w
                        load_q = self.mpc_bus_data.at[index,'Qd'] * mw_w
                        load_baseV = self.mpc_bus_data.at[index,'baseKV'] * kv_v
                        dpsimpy_comp_dict[load_name] = [dpsimpy.sp.ph1.Load(load_name, dpsimpy.LogLevel.info)]
                        dpsimpy_comp_dict[load_name][0].modify_power_flow_bus_type(dpsimpy.PowerflowBusType.PQ)
                        if i==0:
                            dpsimpy_comp_dict[load_name][0].set_parameters(load_p, load_q, load_baseV)
                        else:
                            dpsimpy_comp_dict[load_name][0].set_parameters(0, 0, load_baseV)
                        # add connections
                        dpsimpy_comp_dict[load_name].append([dpsimpy_busses_dict[bus_index]]) # [to bus]

            # Generators
            elif bus_type == 2:
                generator = generator + 1
                gen_name = "gen%s" %generator

                # relevant data from self.mpc_gen_data. Identification with bus number available in mpc_bus_data and mpc_gen_data
                gen = self.mpc_gen_data.loc[self.mpc_gen_data['bus'] == self.mpc_bus_data.at[index,'bus_i']]

                gen_baseS = gen['mBase']*mw_w # gen base MVA default is mpc.baseMVA
                gen_baseV = self.mpc_bus_data.at[index,'baseKV']*kv_v # gen base kV
                gen_v = gen['Vg']*gen_baseV   # gen set point voltage (gen['Vg'] in p.u.)
                gen_p = gen['Pg']*mw_w   # gen ini. active power (gen['Pg'] in MVA)
                gen_q = gen['Qg']*mw_w   # gen ini. reactive power (gen['Qg'] in MVAr)
                gen_nom_s = abs(complex(gen['Pmax'], gen['Qmax'])) # gen nominal power (set default to mpc.baseMVA ? )

                dpsimpy_comp_dict[gen_name] = [dpsimpy.sp.ph1.SynchronGenerator(gen_name, dpsimpy.LogLevel.info)]
                dpsimpy_comp_dict[gen_name][0].set_parameters(gen_nom_s, gen_baseV, gen_p, gen_v, dpsimpy.PowerflowBusType.PV, gen_q)
                dpsimpy_comp_dict[gen_name][0].set_base_voltage(gen_baseV)

                # add connections
                dpsimpy_comp_dict[gen_name].append([dpsimpy_busses_dict[bus_index]]) # [to bus]

                # check if there is a load connected to PV bus
                P_d = self.mpc_bus_data.at[index,'Pd'] * mw_w
                Q_d = self.mpc_bus_data.at[index,'Qd'] * mw_w

                if (P_d !=0) or (Q_d!=0):
                    load = load + 1
                    load_name = "load%s" %load
                    load_p = self.mpc_bus_data.at[index,'Pd'] * mw_w
                    load_q = self.mpc_bus_data.at[index,'Qd'] * mw_w
                    load_baseV = self.mpc_bus_data.at[index,'baseKV'] * kv_v

                    dpsimpy_comp_dict[load_name] = [dpsimpy.sp.ph1.Load(load_name, dpsimpy.LogLevel.info)]
                    dpsimpy_comp_dict[load_name][0].set_parameters(load_p, load_q, load_baseV)
                    dpsimpy_comp_dict[load_name][0].modify_power_flow_bus_type(dpsimpy.PowerflowBusType.PQ)

                    # add connections
                    dpsimpy_comp_dict[load_name].append([dpsimpy_busses_dict[bus_index]]) # [to bus]
                
                del P_d, Q_d

            # Network injection (slack bus)
            elif bus_type == 3:
                inj = inj + 1
                extnet_name = "extnet%s" %inj

                # # relevant data from self.mpc_gen_data. Identification with bus number available in mpc_bus_data and mpc_gen_data
                extnet = self.mpc_gen_data.loc[self.mpc_gen_data['bus'] == self.mpc_bus_data.at[index,'bus_i']]

                # extnet_baseS= extnet['mBase']*mw_w # default is mpc.baseMVA
                extnet_baseV = self.mpc_bus_data.at[index,'baseKV']*kv_v
                extnet_v = extnet['Vg']*extnet_baseV

                dpsimpy_comp_dict[extnet_name] = [dpsimpy.sp.ph1.NetworkInjection(extnet_name, dpsimpy.LogLevel.info)]
                dpsimpy_comp_dict[extnet_name][0].set_parameters(extnet_v)
                dpsimpy_comp_dict[extnet_name][0].set_base_voltage(extnet_baseV)
                dpsimpy_comp_dict[extnet_name][0].modify_power_flow_bus_type(dpsimpy.PowerflowBusType.VD)

                # add connections
                dpsimpy_comp_dict[extnet_name].append([dpsimpy_busses_dict[bus_index]]) # [to bus]

                # # set initial Q is not implemented for slack in DPsim. The mapping was changed here to test cas39.
                # # This mapping does not work with asm uc because Pg and Qg of the slack are equal to zero
                # # TODO implement initial reactive power for slack in Dpsim.
                # # This way the maaping can be done exclusively with extnet compoe
                # # nent

                # # relevant data from self.mpc_gen_data. Identification with bus number available in mpc_bus_data and mpc_gen_data
                # gen = self.mpc_gen_data.loc[self.mpc_gen_data['bus'] == self.mpc_bus_data.at[index,'bus_i']]

                # gen_baseS = gen['mBase']*mw_w # gen base MVA default is mpc.baseMVA
                # gen_baseV = self.mpc_bus_data.at[index,'baseKV']*kv_v # gen base kV
                # gen_v = gen['Vg']*gen_baseV   # gen set point voltage (gen['Vg'] in p.u.)
                # gen_p = gen['Pg']*mw_w   # gen ini. active power (gen['Pg'] in MVA)
                # gen_q = gen['Qg']*mw_w   # gen ini. reactive power (gen['Qg'] in MVAr)
                # gen_nom_s = abs(complex(gen['Pmax'], gen['Qmax'])) # gen nominal power (set default to mpc.baseMVA ? )

                # dpsimpy_comp_dict[extnet_name] = [dpsimpy.sp.ph1.SynchronGenerator(extnet_name, dpsimpy.LogLevel.info)]
                # dpsimpy_comp_dict[extnet_name][0].set_parameters(gen_nom_s, gen_baseV, gen_p, gen_v, dpsimpy.PowerflowBusType.VD, gen_q)
                # dpsimpy_comp_dict[extnet_name][0].set_base_voltage(gen_baseV)

                # # add connections
                # dpsimpy_comp_dict[extnet_name].append([dpsimpy_busses_dict[bus_index]]) # [to bus]

                # check if there is a load connected to PV bus
                P_d = self.mpc_bus_data.at[index,'Pd'] * mw_w
                Q_d = self.mpc_bus_data.at[index,'Qd'] * mw_w

                if (P_d !=0) or (Q_d!=0):
                    load = load + 1
                    load_name = "load%s" %load
                    load_p = self.mpc_bus_data.at[index,'Pd'] * mw_w
                    load_q = self.mpc_bus_data.at[index,'Qd'] * mw_w
                    load_baseV = self.mpc_bus_data.at[index,'baseKV'] * kv_v

                    dpsimpy_comp_dict[load_name] = [dpsimpy.sp.ph1.Load(load_name, dpsimpy.LogLevel.info)]
                    dpsimpy_comp_dict[load_name][0].set_parameters(load_p, load_q, load_baseV)
                    # dpsimpy_comp_dict[load_name][0].modify_power_flow_bus_type(dpsimpy.PowerflowBusType.PQ)

                    # add connections
                    dpsimpy_comp_dict[load_name].append([dpsimpy_busses_dict[bus_index]]) # [to bus]
                
                del P_d, Q_d

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

                dpsimpy_comp_dict[line_name] = [dpsimpy.sp.ph1.PiLine(line_name, dpsimpy.LogLevel.info)]
                dpsimpy_comp_dict[line_name][0].set_parameters(line_r, line_l, line_c, line_g)
                dpsimpy_comp_dict[line_name][0].set_base_voltage(line_tbus_baseV)
                # add connections
                dpsimpy_comp_dict[line_name].append([dpsimpy_busses_dict[str(line_fbus)], dpsimpy_busses_dict[str(line_tbus)]])

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

                dpsimpy_comp_dict[transf_name] = [dpsimpy.sp.ph1.Transformer(transf_name, dpsimpy.LogLevel.info)]
                dpsimpy_comp_dict[transf_name][0].set_parameters(transf_primary_v, transf_secondary_v, transf_s, np.abs(transf_ratioAbs), np.angle(transf_ratioAbs), transf_r, transf_l)
                # print(transf_primary_v, transf_secondary_v, transf_s, np.abs(transf_ratioAbs), np.angle(transf_ratioAbs), transf_r, transf_l)
                dpsimpy_comp_dict[transf_name][0].set_base_voltage(transf_baseV)

                # add connections
                dpsimpy_comp_dict[transf_name].append([dpsimpy_busses_dict[str(transf_fbus)], dpsimpy_busses_dict[str(transf_tbus)]])

        return dpsimpy_busses_dict, dpsimpy_comp_dict

    def load_mpc(self):

        dpsimpy_busses_dict, dpsimpy_comp_dict = self.create_dpsim_objects()

        system_comp = []
        system_nodes = []

        for key, value in dpsimpy_comp_dict.items():
            dpsimpy_component = value[0]
            connection_nodes = value[1]
            dpsimpy_component.connect(connection_nodes)

            system_comp.append(dpsimpy_component)

            for n in connection_nodes:
                if n in system_nodes:
                    continue
                else:
                    system_nodes.append(n)

        system = dpsimpy.SystemTopology(self.mpc_freq, system_nodes, system_comp)

        return system

