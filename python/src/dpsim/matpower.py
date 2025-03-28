import pandas as pd
import numpy as np
import scipy.io

import dpsimpy

class Reader:

    def __init__(self, mpc_file_path, mpc_name = 'mpc'):
        # read input file (returns multidimensional dict)
        self.mpc_raw = scipy.io.loadmat(mpc_file_path)
        self.mpc_name = mpc_name

    def process_mpc(self):

        version_idx = 0
        base_pow_idx = 1
        bus_data_idx = 2
        gen_data_idx = 3
        branch_data_idx = 4
        # gencost_data_idx= 5

        # Process raw mpc data and create corresponding dataframes
        # Version
        self.mpc_version = self.mpc_raw[self.mpc_name][0][0][version_idx]

        # System frequency (not included in mpc but needed for setting dpsimpy component parameters i.e inductances, capacitances ..)
        self.mpc_freq = 50
        self.mpc_omega = 2*np.pi*50

        # Base power (MVA)
        self.mpc_base_power_MVA = self.mpc_raw[self.mpc_name][0][0][base_pow_idx][0][0]

        #### Busses
        mpc_bus_raw = self.mpc_raw[self.mpc_name][0][0][bus_data_idx]

        bus_data_header = ["bus_i", "type", "Pd", "Qd", "Gs", "Bs", "area",
                           "Vm", "Va", "baseKV", "zone", "Vmax", "Vmin"]

        self.mpc_bus_data = pd.DataFrame(mpc_bus_raw, columns = bus_data_header)

        # scipy.io.loadmat loads all matrix entries as double. Convert specific columns back to int
        self.mpc_bus_data['bus_i'] = self.mpc_bus_data['bus_i'].astype(int)
        self.mpc_bus_data['type'] = self.mpc_bus_data['type'].astype(int)
        self.mpc_bus_data['area'] = self.mpc_bus_data['area'].astype(int)
        self.mpc_bus_data['zone'] = self.mpc_bus_data['zone'].astype(int)

        #### Generators
        mpc_gen_raw = self.mpc_raw[self.mpc_name][0][0][gen_data_idx]

        gen_data_header = ["bus", "Pg", "Qg", "Qmax", "Qmin", "Vg", "mBase", "status",
                           "Pmax", "Pmin", "Pc1", "Pc2", "Qc1min", "Qc1max", "Qc2min",
                           "Qc2max", "ramp_agc", "ramp_10", "ramp_30", "ramp_q", "apf"]

        self.mpc_gen_data = pd.DataFrame(mpc_gen_raw, columns = gen_data_header)

        self.mpc_gen_data['bus'] = self.mpc_gen_data['bus'].astype(int)
        self.mpc_gen_data['status'] = self.mpc_gen_data['status'].astype(int)

        #### Branches
        # extract only first 13 columns since following columns include results
        mpc_branch_raw = self.mpc_raw[self.mpc_name][0][0][branch_data_idx][:, :13]

        branch_data_header = ["fbus", "tbus", "r", "x", "b", "rateA", "rateB",
                            "rateC", "ratio", "angle", "status", "angmin", "angmax"]

        self.mpc_branch_data = pd.DataFrame(mpc_branch_raw, columns = branch_data_header)

        self.mpc_branch_data['fbus'] = self.mpc_branch_data['fbus'].astype(int)
        self.mpc_branch_data['tbus'] = self.mpc_branch_data['tbus'].astype(int)
        self.mpc_branch_data['status'] = self.mpc_branch_data['status'].astype(int)

        #### TODO Generator costs

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
            # create dpsimpy busses
            bus = bus + 1
            bus_index = str(self.mpc_bus_data.at[index,'bus_i'])
            bus_name = bus_index
            dpsimpy_busses_dict[bus_name] = dpsimpy.sp.SimNode(bus_name, dpsimpy.PhaseType.Single)

            # for each bus type create corresponding dpsimpy component
            # 1 = PQ, 2 = PV, 3 = ref, 4 = isolated
            bus_type = self.mpc_bus_data.at[index,'type']

            # Loads
            if bus_type == 1:
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
                # gen_q = gen['Qg']*mw_w   # gen ini. reactive power (gen['Qg'] in MVAr)
                gen_nom_s = abs(complex(gen['Pmax'], gen['Qmax'])) # gen nominal power (set default to mpc.baseMVA ? )

                dpsimpy_comp_dict[gen_name] = [dpsimpy.sp.ph1.SynchronGenerator(gen_name, dpsimpy.LogLevel.info)]
                dpsimpy_comp_dict[gen_name][0].set_parameters(gen_nom_s, gen_baseV, gen_p, gen_v, dpsimpy.PowerflowBusType.PV)
                dpsimpy_comp_dict[gen_name][0].set_base_voltage(gen_baseV)

                # add connections
                dpsimpy_comp_dict[gen_name].append([dpsimpy_busses_dict[bus_index]]) # [to bus]

            # Network injection (slack bus)
            elif bus_type == 3:
                inj = inj + 1
                extnet_name = "extnet%s" %inj

                # relevant data from self.mpc_gen_data. Identification with bus number available in mpc_bus_data and mpc_gen_data
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

            # Lines
            if branch_ratio == 0:
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

                transf_offNom_ratio = self.mpc_branch_data.at[index,'ratio']
                transf_primary_v = transf_primary_v/ transf_offNom_ratio
                transf_ratio = transf_primary_v / transf_secondary_v


                transf_baseZ = transf_tbus_baseV*transf_tbus_baseV / (self.mpc_base_power_MVA*mw_w)
                transf_r = self.mpc_branch_data.at[index,'r']* transf_baseZ
                transf_x = self.mpc_branch_data.at[index,'x']* transf_baseZ
                transf_l = transf_x / self.mpc_omega

                dpsimpy_comp_dict[transf_name] = [dpsimpy.sp.ph1.Transformer(transf_name, dpsimpy.LogLevel.info)]
                dpsimpy_comp_dict[transf_name][0].set_parameters(transf_primary_v, transf_secondary_v, np.abs(transf_ratio), np.angle(transf_ratio), transf_r, transf_l)
                dpsimpy_comp_dict[transf_name][0].set_base_voltage(transf_tbus_baseV)

                print(transf_primary_v, transf_secondary_v, np.abs(transf_ratio), np.angle(transf_ratio), transf_r, transf_l)
                print(transf_tbus_baseV)

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

