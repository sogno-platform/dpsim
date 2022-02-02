import sys
sys.path.append('/task4.3/dpsim/build') #add dpsimpy build directory to sys.path.
# print('\n'.join(sys.path))

import pandas as pd
import numpy as np
import dpsimpy



def create_dpsimpy_system(mpc_objects):

    dpsimpy_busses_dict, dpsimpy_comp_dict= create_dpsimpy_objects(mpc_objects)

    system_comp=[]
    system_nodes=[]
    
    for key, value in dpsimpy_comp_dict.items():
        dpsimpy_component= value[0]
        connection_nodes= value[1]
        dpsimpy_component.connect(connection_nodes)

        system_comp.append(dpsimpy_component)

        for n in connection_nodes:
            if n in system_nodes:
                continue
            else:
                system_nodes.append(n)

    system = dpsimpy.SystemTopology(mpc_objects.mpc_freq, system_nodes, system_comp)

    return dpsimpy_busses_dict, dpsimpy_comp_dict, system

def create_dpsimpy_objects(mpc_objects):

    #### Nodes ####
    dpsimpy_busses_dict= {}
    dpsimpy_comp_dict= {}
    mw_w = 1e6
    kv_v = 1e3
    b= 0
    lo= 0
    s= 0
    g= 0
    t= 0
    li=0
    
    for index, bus in mpc_objects.mpc_bus_data.iterrows():        
        #create dpsimpy busses
        b+=1
        bus_index= str(mpc_objects.mpc_bus_data.at[index,'bus_i'])
        bus_name= bus_index
        dpsimpy_busses_dict[bus_name]= dpsimpy.sp.SimNode(bus_name, dpsimpy.PhaseType.Single)

        #For each bus type create corresponding dpsimpy component
        bus_type= mpc_objects.mpc_bus_data.at[index,'type'] # 1 = PQ, 2 = PV, 3 = ref, 4 = isolated

        #Loads
        if bus_type == 1:
            lo+=1
            load_name = "load%s" %lo
            load_p= mpc_objects.mpc_bus_data.at[index,'Pd']*mw_w
            load_q= mpc_objects.mpc_bus_data.at[index,'Qd']*mw_w
            load_baseV= mpc_objects.mpc_bus_data.at[index,'baseKV']*kv_v
            
            dpsimpy_comp_dict[load_name]= [dpsimpy.sp.ph1.Load(load_name, dpsimpy.LogLevel.info)]
            dpsimpy_comp_dict[load_name][0].set_parameters(load_p, load_q, load_baseV)
            dpsimpy_comp_dict[load_name][0].modify_power_flow_bus_type(dpsimpy.PowerflowBusType.PQ)

            # add connections
            dpsimpy_comp_dict[load_name].append([dpsimpy_busses_dict[bus_index]]) # [to bus]

        #Generators
        elif bus_type == 2:
            g+=1
            gen_name= "gen%s" %g

            # relevant data from mpc_objects.mpc_gen_data. Identification with bus number available in mpc_bus_data and mpc_gen_data 
            gen= mpc_objects.mpc_gen_data.loc[mpc_objects.mpc_gen_data['bus'] == mpc_objects.mpc_bus_data.at[index,'bus_i']]

            gen_baseS= gen['mBase']*mw_w # gen base MVA default is mpc.baseMVA
            gen_baseV= mpc_objects.mpc_bus_data.at[index,'baseKV']*kv_v # gen base kV
            gen_v= gen['Vg']*gen_baseV   # gen set point voltage (gen['Vg'] in p.u.)
            gen_p= gen['Pg']*mw_w   # gen ini. active power (gen['Pg'] in MVA)
            # gen_q= gen['Qg']*mw_w   # gen ini. reactive power (gen['Qg'] in MVAr)
            gen_nom_s= abs(complex(gen['Pmax'], gen['Qmax'])) # gen nominal power (set default to mpc.baseMVA ? )

            dpsimpy_comp_dict[gen_name]= [dpsimpy.sp.ph1.SynchronGenerator(gen_name, dpsimpy.LogLevel.info)]
            dpsimpy_comp_dict[gen_name][0].set_parameters(gen_nom_s, gen_baseV, gen_p, gen_v, dpsimpy.PowerflowBusType.PV)
            dpsimpy_comp_dict[gen_name][0].set_base_voltage(gen_baseV)

            # add connections
            dpsimpy_comp_dict[gen_name].append([dpsimpy_busses_dict[bus_index]]) # [to bus]

        #Network injection (slack bus)
        elif bus_type == 3:
            s+=1
            extnet_name= "extnet%s" %s

            # relevant data from mpc_objects.mpc_gen_data. Identification with bus number available in mpc_bus_data and mpc_gen_data 
            extnet= mpc_objects.mpc_gen_data.loc[mpc_objects.mpc_gen_data['bus'] == mpc_objects.mpc_bus_data.at[index,'bus_i']]

            # extnet_baseS= extnet['mBase']*mw_w # default is mpc.baseMVA
            extnet_baseV= mpc_objects.mpc_bus_data.at[index,'baseKV']*kv_v
            extnet_v= extnet['Vg']*extnet_baseV

            dpsimpy_comp_dict[extnet_name]= [dpsimpy.sp.ph1.NetworkInjection(extnet_name, dpsimpy.LogLevel.info)]
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
    for index, branch in mpc_objects.mpc_branch_data.iterrows():
    
        branch_ratio= mpc_objects.mpc_branch_data.at[index,'ratio']

        #Lines
        if branch_ratio == 0:
            li+=1
            line_name = "line%s_%s-%s" %(li, mpc_objects.mpc_branch_data.at[index,'fbus'] , mpc_objects.mpc_branch_data.at[index,'tbus'])
      
            line_fbus= mpc_objects.mpc_branch_data.at[index,'fbus']
            line_tbus= mpc_objects.mpc_branch_data.at[index,'tbus']

            tmp_fbus= mpc_objects.mpc_bus_data.loc[mpc_objects.mpc_bus_data['bus_i'] == line_fbus]
            tmp_tbus= mpc_objects.mpc_bus_data.loc[mpc_objects.mpc_bus_data['bus_i'] == line_tbus]

            line_fbus_baseV=mpc_objects.mpc_bus_data.at[tmp_fbus.first_valid_index(),'baseKV']*kv_v
            line_tbus_baseV=mpc_objects.mpc_bus_data.at[tmp_tbus.first_valid_index(),'baseKV']*kv_v

            line_baseZ= line_tbus_baseV*line_tbus_baseV / (mpc_objects.mpc_base_power_MVA*mw_w)
            line_r= mpc_objects.mpc_branch_data.at[index,'r'] * line_baseZ
            line_x= mpc_objects.mpc_branch_data.at[index,'x'] * line_baseZ
            line_b= mpc_objects.mpc_branch_data.at[index,'b'] / line_baseZ
            line_l= line_x / mpc_objects.mpc_omega
            line_c= line_b/ mpc_objects.mpc_omega
            line_g= 0 # line conductance is not included in mpc
            
            dpsimpy_comp_dict[line_name] = [dpsimpy.sp.ph1.PiLine(line_name, dpsimpy.LogLevel.info)]
            dpsimpy_comp_dict[line_name][0].set_parameters(line_r, line_l, line_c, line_g)
            dpsimpy_comp_dict[line_name][0].set_base_voltage(line_tbus_baseV)
            # add connections
            dpsimpy_comp_dict[line_name].append([dpsimpy_busses_dict[str(line_fbus)], dpsimpy_busses_dict[str(line_tbus)]])

        # #Transformers
        else:
            t+=1
            transf_name = "transformer%s_%s-%s" %(t, mpc_objects.mpc_branch_data.at[index,'fbus'] , mpc_objects.mpc_branch_data.at[index,'tbus'])
            transf_s= mpc_objects.mpc_branch_data.at[index,'rateA']*mw_w # Matpower: Used to specify branch flow limits.  By default these are limits on apparent power with units in MV

            transf_fbus= mpc_objects.mpc_branch_data.at[index,'fbus']
            transf_tbus= mpc_objects.mpc_branch_data.at[index,'tbus']

            tmp_fbus= mpc_objects.mpc_bus_data.loc[mpc_objects.mpc_bus_data['bus_i'] == transf_fbus]
            tmp_tbus= mpc_objects.mpc_bus_data.loc[mpc_objects.mpc_bus_data['bus_i'] == transf_tbus]


            transf_fbus_baseV=mpc_objects.mpc_bus_data.at[tmp_fbus.first_valid_index(),'baseKV']*kv_v
            transf_tbus_baseV=mpc_objects.mpc_bus_data.at[tmp_tbus.first_valid_index(),'baseKV']*kv_v

            transf_primary_v= mpc_objects.mpc_bus_data.at[tmp_fbus.first_valid_index(),'Vm']*transf_fbus_baseV
            transf_secondary_v= mpc_objects.mpc_bus_data.at[tmp_tbus.first_valid_index(),'Vm']*transf_tbus_baseV
            
            transf_offNom_ratio= mpc_objects.mpc_branch_data.at[index,'ratio']
            transf_primary_v= transf_primary_v/ transf_offNom_ratio
            transf_ratio= transf_primary_v / transf_secondary_v

            
            transf_baseZ= transf_tbus_baseV*transf_tbus_baseV / (mpc_objects.mpc_base_power_MVA*mw_w)
            transf_r= mpc_objects.mpc_branch_data.at[index,'r']* transf_baseZ
            transf_x= mpc_objects.mpc_branch_data.at[index,'x']* transf_baseZ 
            transf_l= transf_x / mpc_objects.mpc_omega

            dpsimpy_comp_dict[transf_name]= [dpsimpy.sp.ph1.Transformer(transf_name, dpsimpy.LogLevel.info)]
            dpsimpy_comp_dict[transf_name][0].set_parameters(transf_primary_v, transf_secondary_v, np.abs(transf_ratio), np.angle(transf_ratio), transf_r, transf_l)
            dpsimpy_comp_dict[transf_name][0].set_base_voltage(transf_tbus_baseV)

            print(transf_primary_v, transf_secondary_v, np.abs(transf_ratio), np.angle(transf_ratio), transf_r, transf_l)
            print(transf_tbus_baseV)

            # add connections
            dpsimpy_comp_dict[transf_name].append([dpsimpy_busses_dict[str(transf_fbus)], dpsimpy_busses_dict[str(transf_tbus)]])

    return dpsimpy_busses_dict, dpsimpy_comp_dict
