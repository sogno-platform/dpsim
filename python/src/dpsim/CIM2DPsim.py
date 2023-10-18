from enum import Enum
import logging
import numpy as np
import pandas as pd
import cimpy
import sys
sys.path.insert(0,'/home/mmo/git/Can/dpsim/build')
import dpsimpy

# configure logging
logging.basicConfig(filename='CIM2Dpsim.log', encoding='utf-8', level=logging.DEBUG)

# define dpsimpy domains
class Domain(Enum):
    PF = 1
    SP = 2
    DP = 3
    EMT = 4

# define dpsimpy SG models
class SGModels(Enum):
    VBR_3Order = 1
    VBR_4Order = 2
    VBR_5Order = 3
    VBR_6Order = 4
    
class Multiplier(Enum):
    p = 1
    n = 2
    micro = 3
    m = 4
    c = 5
    d = 6
    k = 7
    M = 8
    G = 9
    T = 10
    
def unitValue(value, multiplier=Multiplier.k):
    if (multiplier==Multiplier.p):
        return value * 1e-12
    elif (multiplier==Multiplier.n):
        return value * 1e-9
    elif (multiplier==Multiplier.micro):
        return value * 1e-6
    elif (multiplier==Multiplier.m):
        return value * 1e-3
    elif (multiplier==Multiplier.c):
        return value * 1e-2
    elif (multiplier==Multiplier.d):
        return value * 1e-1
    elif (multiplier==Multiplier.k):
        return value * 1e3
    elif (multiplier==Multiplier.M):
        return value * 1e6
    elif (multiplier==Multiplier.G):
        return value * 1e9
    elif (multiplier==Multiplier.T):
        return value * 1e12


def CIM2DPsim(CIM_network, domain, frequency = 60, log_level=dpsimpy.LogLevel.info, gen_model=SGModels.VBR_4Order):

    # Nodes dictionary
    nodes = dict()
    
    # Components dictionary
    components_dict = dict()
    
    # Synchronous Machines dictionary
    # TODO: remove this list?
    SynchronousMachineTCR_Dict = dict()

    res = CIM_network["topology"]

    # get the correct module to be used (dpsimpy.sp, dpsimpy.dp or dpsimpy.emt) 
    dpsimpy_components = None
    if (domain == Domain.PF) or (domain == Domain.SP):
        dpsimpy_components = dpsimpy.sp.ph1
        dpsimpy_components.SimNode = getattr(dpsimpy.sp, "SimNode")
    elif (domain == Domain.DP):
        dpsimpy_components = dpsimpy.dp.ph1
        dpsimpy_components.SimNode = getattr(dpsimpy.dp, "SimNode") 
    elif (domain == Domain.EMT):
        dpsimpy_components = dpsimpy.emt.ph3
        dpsimpy_components.SimNode = getattr(dpsimpy.emt, "SimNode")  
    else:
        raise Exception('ERROR: domain {} is not supported in dpsimpy'.format(domain))

    ### Components
    for i in res:
        
        # PiLine
        if isinstance(res[i], cimpy.cgmes_v2_4_15.ACLineSegment):
            pi_line = dpsimpy_components.PiLine(res[i].mRID, res[i].name, log_level)
            if (domain==Domain.EMT):
                pi_line.set_parameters(R=dpsimpy.Math.single_phase_parameter_to_three_phase(res[i].r), 
                                       L=dpsimpy.Math.single_phase_parameter_to_three_phase(res[i].x/(2*np.pi*frequency)), 
                                       C=dpsimpy.Math.single_phase_parameter_to_three_phase(res[i].bch/(2*np.pi*frequency)), 
                                       G=dpsimpy.Math.single_phase_parameter_to_three_phase(res[i].gch))
            else:
                pi_line.set_parameters(R=res[i].r,                              #line resistance                         
                                       L=res[i].x/(2*np.pi*frequency),          #line inductance
                                       C=res[i].bch/(2*np.pi*frequency),        #line capacitance
                                       G=res[i].gch)                            #line conductance
            if (domain == Domain.PF):
                # Set BaseVoltage of ACLineSegment to PiLine
                baseVoltage = unitValue(res[i].BaseVoltage.nominalVoltage, Multiplier.k)
                pi_line.set_base_voltage(baseVoltage)

            components_dict[res[i].mRID] = {"Element": pi_line, "Nodes": []}
            logging.debug('Created ACLineSegment: name={}, L={}, R={}, C={}, G={}'.format(
                res[i].name, res[i].r, res[i].x/(2*np.pi*frequency), res[i].bch/(2*np.pi*frequency), res[i].gch))
        
        # External network injection
        elif isinstance(res[i], cimpy.cgmes_v2_4_15.ExternalNetworkInjection):
            # Slack
            slack = dpsimpy_components.NetworkInjection(res[i].mRID, res[i].name, log_level)
            if (domain == Domain.PF):
                baseVoltage = 0
                for obj in res.values():
                    if isinstance(obj , cimpy.cgmes_v2_4_15.BaseVoltage):
                        if obj.ConductingEquipment != 'list':
                            for comp in obj.ConductingEquipment:
                                if comp.mRID == res[i].mRID:
                                    baseVoltage = unitValue(obj.BaseVoltage.nominalVoltage, Multiplier.k)
                                    break

                if (baseVoltage == 0):
                    # Take baseVoltage of topologicalNode where equipment is connected to
                    for obj in res.values():
                        if isinstance(obj, cimpy.cgmes_v2_4_15.TopologicalNode):
                                for term in obj.Terminal:
                                    if term.ConductingEquipment.mRID == res[i].mRID:
                                        baseVoltage = unitValue(obj.BaseVoltage.nominalVoltage, Multiplier.k)
                                        break
                if (baseVoltage == 0):
                    baseVoltage = 1
                slack.set_base_voltage(baseVoltage)

                if res[i].RegulatingControl != None:
                    voltageSetPoint = res[i].RegulatingControl.targetValue * baseVoltage
                else:
                    voltageSetPoint = baseVoltage
                    logging.warn('No voltage set-point defined in ExternalNetworkInjection "{}". Using 1 per unit.'.format(res[i].name))
                
                slack.set_parameters(voltage_set_point = voltageSetPoint)
                logging.debug('Created ExternalNetworkInjection: name={}, baseVoltage={}[V], voltageSetPoint={}[V]'.format(res[i].name, baseVoltage, voltageSetPoint))
            else:
                baseVoltage = 1
                for obj in res.values():
                    if isinstance(obj, cimpy.cgmes_v2_4_15.TopologicalNode):
                        for term in obj.Terminal:
                            if term.ConductingEquipment.mRID == res[i].mRID:
                                baseVoltage = unitValue(obj.BaseVoltage.nominalVoltage, Multiplier.k)
                                break
                try:
                    res[i].RegulatingControl.targetValue
                except:
                    voltageRef = 0
                else:
                    voltageRef = res[i].RegulatingControl.targetValue * baseVoltage

                slack.set_parameters(V_ref = voltageRef)
                logging.debug('Created ExternalNetworkInjection: name={}, baseVoltage={}[V], voltageRef={}[V]'.format(res[i].name, baseVoltage, voltageRef))
                       
            components_dict[res[i].mRID] = {"Element": slack, "Nodes": []}

        # Synchronous machine (power flow)
        elif isinstance(res[i], cimpy.cgmes_v2_4_15.SynchronousMachine) and (domain==Domain.PF):
            gen_pf = dpsimpy.sp.ph1.SynchronGenerator(res[i].mRID, res[i].name, log_level)

            # get base power
            gen_baseS = 0
            if not np.isclose(res[i].ratedS, 0):
                gen_baseS = unitValue(res[i].ratedS, Multiplier.M)
            else:
                raise Exception('ERROR: Uninitalized ratedS for SynchronousMachine {}'.format(res[i].name))
            
            # get base voltage
            gen_baseV = 0
            if not np.isclose(res[i].ratedU, 0):
                gen_baseV = unitValue(res[i].ratedU, Multiplier.k)
            else:
                #shoud we use baseVoltage instead?
                raise Exception('ERROR: Uninitalized ratedU for SynchronousMachine {}'.format(res[i].name))
        
            # get initial power
            gen_p = unitValue(res[i].GeneratingUnit.initialP, Multiplier.M)
            
            # get reactive power
            gen_q = unitValue(res[i].q, Multiplier.M)
            """
            else:
                # try to get power from terminals
                for obj in res.values():
                    if isinstance(obj, cimpy.cgmes_v2_4_15.SvPowerFlow):
                        if obj.Terminal.ConductingEquipment.mRID == res[i].mRID:
                            print("Test")
                            gen_p = getattr(obj, "p", 0)
                            print(gen_p)
                            break
                if np.isclose(gen_p, 0):
                    raise Exception('ERROR: Uninitalized initialP for SynchronousMachine {}'.format(res[i].name))
            """
            
            # get voltage set point
            gen_v = unitValue(res[i].RegulatingControl.targetValue, Multiplier.k)     
            if np.isclose(gen_v, 0):
                gen_v=gen_baseV
                logging.warn('No voltage set-point defined in SynchronousMachine "{}". Using set-point=baseVoltage.'.format(res[i].name))
            
            # TODO: set base power is not implemented in DPsim
            # set Parameters
            gen_pf.set_parameters(rated_apparent_power=gen_baseS, rated_voltage=gen_baseV, 
                      set_point_active_power=gen_p, set_point_voltage=gen_v, 
                      set_point_reactive_power=gen_q, 
                      powerflow_bus_type=dpsimpy.PowerflowBusType.PV)
            gen_pf.set_base_voltage(gen_baseV)
            gen_pf.modify_power_flow_bus_type(dpsimpy.PowerflowBusType.PV)

            components_dict[res[i].mRID] = {"Element": gen_pf, "Nodes": []}
            logging.debug('Created SynchronGenerator: name={}, rated_apparent_power={}[VA], rated_voltage={}[V], set_point_active_power={}[W], set_point_voltage={}[V], set_point_reactive_power={}[VAR]'.format(
                res[i].name, gen_baseS, gen_baseV, gen_p, gen_v, gen_q))
        
        # Synchronous machine (dynamic simulation)
        elif isinstance(res[i], cimpy.cgmes_v2_4_15.SynchronousMachineTimeConstantReactance) and (domain!=Domain.PF):
            # Synchron Generator
            name=res[i].SynchronousMachine.name
            mRID=res[i].SynchronousMachine.mRID
            nom_power=unitValue(res[i].SynchronousMachine.ratedS, Multiplier.M)
            nom_voltage=unitValue(res[i].SynchronousMachine.ratedU, Multiplier.k)
            
            # dynamic parameters
            L0=0.15   ## TODO
            H=res[i].inertia
            Ld=res[i].xDirectSync
            Lq=res[i].xQuadSync
            Ld_t=res[i].xDirectTrans
            Lq_t=res[i].xQuadTrans
            Td0_t=res[i].tpdo
            Tq0_t=res[i].tpqo
            Ld_s=res[i].xDirectSubtrans
            Lq_s=res[i].xQuadSubtrans
            Td0_s=res[i].tppdo
            Tq0_s=res[i].tppqo
            
            gen=None
            if (gen_model==SGModels.VBR_3Order):      
                gen = dpsimpy_components.SynchronGenerator3OrderVBR(mRID, name, log_level)
                gen.set_operational_parameters_per_unit(nom_power=nom_power, nom_voltage=nom_voltage, nom_frequency=frequency, H=H,
                                                        Ld=Ld, Lq=Lq, L0=L0, Ld_t=Ld_t, Td0_t=Td0_t)
                logging.debug('Created 3Order VBR SynchronGenerator: name={}, nom_power={}[VA], nom_voltage={}[W], nom_frequency={}[Hz], H={}, Ld={}, Lq={}, L0={}, Ld_t={}, Td0_t={}'.format(
                    name, nom_power, nom_voltage, frequency, H, Ld, Lq, L0, Ld_t, Td0_t))                                                                                                                                                                                            
            elif (gen_model==SGModels.VBR_4Order):
                gen = dpsimpy_components.SynchronGenerator4OrderVBR(mRID, name, log_level)
                gen.set_operational_parameters_per_unit(nom_power=nom_power, nom_voltage=nom_voltage, nom_frequency=frequency, H=H,
                                                        Ld=Ld, Lq=Lq, L0=L0, Ld_t=Ld_t, Lq_t=Lq_t, Td0_t=Td0_t, Tq0_t=Tq0_t)		
                logging.debug('Created 4Order VBR SynchronGenerator: name={}, nom_power={}[VA], nom_voltage={}[W], nom_frequency={}[Hz], H={}, Ld={}, Lq={}, L0={}, Ld_t={}, Lq_t={}, Td0_t={}, Tq0_t={}'.format(
                    name, nom_power, nom_voltage, frequency, H, Ld, Lq, L0, Ld_t, Lq_t, Td0_t, Tq0_t))  
            elif (gen_model==SGModels.VBR_5Order):
                gen = dpsimpy_components.SynchronGenerator5OrderVBR(mRID, name, log_level)
                gen.set_operational_parameters_per_unit(nom_power=nom_power, nom_voltage=nom_voltage, nom_frequency=frequency, H=H, Ld=Ld, Lq=Lq, L0=L0, Ld_t=Ld_t, 
                                                        Lq_t=Lq_t, Td0_t=Td0_t, Tq0_t=Tq0_t, Ld_s=Ld_s, Lq_s=Lq_s, Td0_s=Td0_s, Tq0_s=Tq0_s)	
                logging.debug('Created 5Order VBR SynchronGenerator: name={}, nom_power={}[VA], nom_voltage={}[W], nom_frequency={}[Hz], H={}, Ld={}, Lq={}, L0={}, Ld_t={}, Lq_t={}, Td0_t={}, Tq0_t={}, Ld_s={}, Lq_s={}, Td0_s={}, Tq0_s={}'.format(
                    name, nom_power, nom_voltage, frequency, H, Ld, Lq, L0, Ld_t, Lq_t, Td0_t, Tq0_t, Ld_s, Lq_s, Td0_s, Tq0_s))        
            elif (gen_model==SGModels.VBR_6Order):
                gen = dpsimpy_components.SynchronGenerator6bOrderVBR(mRID, name, log_level)
                gen.set_operational_parameters_per_unit(nom_power=nom_power, nom_voltage=nom_voltage, nom_frequency=frequency, H=H, Ld=Ld, Lq=Lq, L0=L0, Ld_t=Ld_t, 
                                                        Lq_t=Lq_t, Td0_t=Td0_t, Tq0_t=Tq0_t, Ld_s=Ld_s, Lq_s=Lq_s, Td0_s=Td0_s, Tq0_s=Tq0_s)	
                logging.debug('Created 6Order VBR SynchronGenerator: name={}, nom_power={}[VA], nom_voltage={}[W], nom_frequency={}[Hz], H={}, Ld={}, Lq={}, L0={}, Ld_t={}, Lq_t={}, Td0_t={}, Tq0_t={}, Ld_s={}, Lq_s={}, Td0_s={}, Tq0_s={}'.format(
                    name, nom_power, nom_voltage, frequency, H, Ld, Lq, L0, Ld_t, Lq_t, Td0_t, Tq0_t, Ld_s, Lq_s, Td0_s, Tq0_s)) 
                
            # MOVE THIS PART TO A FUNCTION "initialize_from_steady_state_hyp", and get values from SV files
            """
            gen_p = unitValue(res[i].SynchronousMachine.p, Multiplier.M)
            gen_q = unitValue(res[i].GeneratingUnit.initialP, Multiplier.M)
            init_electrical_power = complex( float(str(res[i].SynchronousMachine.p )), float(str(res[i].SynchronousMachine.q)))
            init_mechanical_power = res[i].SynchronousMachine.p
            for obj in res.values():
                if isinstance(obj, cimpy.cgmes_v2_4_15.SvVoltage):
                    for term in obj.TopologicalNode.Terminal:
                        if term.ConductingEquipment.mRID == res[i].SynchronousMachine.mRID:
                            betrag = getattr(obj, "v", 0)
                            phase = getattr(obj, "angle", 0)
                            init_complex_terminal_voltage = betrag * np.exp(1j * phase)
                            break
            gen.set_initial_values(init_complex_electrical_power=init_electrical_power, init_mechanical_power=init_mechanical_power, 
                           init_complex_terminal_voltage=init_complex_terminal_voltage)
            """
            
            components_dict[mRID] = {"Element": gen, "Nodes": [], "Sync_Machine": res[i].SynchronousMachine}
            SynchronousMachineTCR_Dict[mRID] = res[i].SynchronousMachine
        
        # Energy Consumer
        elif isinstance(res[i], cimpy.cgmes_v2_4_15.EnergyConsumer) or isinstance(res[i], cimpy.cgmes_v2_4_15.ConformLoad):          
            # Use EnergyConsumer.P and EnergyConsumer.Q if available
            p = unitValue(getattr(res[i], "p", 0), Multiplier.M)
            q = unitValue(getattr(res[i], "q", 0), Multiplier.M)
            if np.isclose(p,0) and np.isclose(q,0):
                # otherwise use SvPowerFlow data if exists
                for obj in res.values():
                    if isinstance(obj, cimpy.cgmes_v2_4_15.SvPowerFlow):
                        if obj.Terminal.ConductingEquipment.mRID == res[i].mRID:
                            p = unitValue(getattr(obj, "p", 0), Multiplier.M) 
                            q = unitValue(getattr(obj, "q", 0), Multiplier.M)
            
            if p == 0 and q == 0:
                logging.debug('Uninitialized value for p and q of load "{}". Load will be skipped!',format(res[i].name))
                break
            if (domain == Domain.EMT):
                p = dpsimpy.Math.single_phase_parameter_to_three_phase(p/3)
                q = dpsimpy.Math.single_phase_parameter_to_three_phase(q/3)

            for obj in res.values():
                if isinstance(obj, cimpy.cgmes_v2_4_15.SvVoltage):
                    for term in obj.TopologicalNode.Terminal:
                        if term.ConductingEquipment.mRID == res[i].mRID:
                            nom_voltage = unitValue(getattr(obj, "v", 0), Multiplier.M)
                            break
            if nom_voltage == 0:
                nom_voltage = unitValue(res[i].BaseVoltage.nominalVoltage, Multiplier.k)         

            if (domain == Domain.PF):
                load = dpsimpy_components.Load(res[i].mRID, res[i].name, log_level)
                load.set_parameters(p, q, nom_voltage)
                load.modify_power_flow_bus_type(dpsimpy.PowerflowBusType.PQ)
            elif (domain == Domain.SP):
                load = dpsimpy_components.Load(res[i].mRID, res[i].name, log_level)
                #load.set_parameters(p, q, nom_voltage)
                load.set_power(p, q)
            else:
                #DP, EMT
                load = dpsimpy_components.RXLoad(res[i].mRID, res[i].name, log_level)
                #load.set_parameters(p, q, nom_voltage)
                load.set_power(p, q)
                

            components_dict[res[i].mRID] = {"Element": load, "Nodes": []}
            logging.debug('Created EnergyConsumer: name={}, p={}[W], q={}[VAR], nom_voltage={}[V]'.format(
                res[i].name, p, q, nom_voltage))

        # Power Transformer
        elif isinstance(res[i], cimpy.cgmes_v2_4_15.PowerTransformer):
            transformer = dpsimpy_components.Transformer(res[i].mRID, res[i].name, log_level)

            # get power transformer ends
            end1=None
            end2=None
            end1=res[i].PowerTransformerEnd[0] if res[i].PowerTransformerEnd[0].endNumber==1 else res[i].PowerTransformerEnd[1]
            end2=res[i].PowerTransformerEnd[1] if res[i].PowerTransformerEnd[1].endNumber==2 else res[i].PowerTransformerEnd[0]
            
            # get values of end1
            rated_voltage_end1 = unitValue(end1.ratedU, Multiplier.k)
            rated_power_end1 = unitValue(end1.ratedS, Multiplier.M)
            r_end1 = end1.r
            x_end1 = end1.x
            
            # get values of end2
            rated_voltage_end2 = unitValue(end2.ratedU, Multiplier.k) 
            rated_power_end2 = unitValue(end2.ratedS, Multiplier.M)
            r_end2 = end2.r
            x_end2 = end2.x
            
            # Calculate resistance and inductance referred to higher voltage side
            resistance=0
            inductance=0
            omega=2*np.pi*frequency
            
            # TODO: consider tap changer!!
            # TODO: consider phase of transformers
            ratio=0
            if (rated_voltage_end1>=rated_voltage_end2):
                # high voltage side is end1
                ratio=rated_voltage_end1/rated_voltage_end2
                if (np.abs(x_end1)>1e-12):
                    # transformer impedance is refered to end2
                    inductance=x_end1/omega
                    resistance=r_end1
                elif (np.abs(x_end2)>1e-12):
                    # transformer impedance is refered to end1 --> end1
                    inductance=(x_end2/omega)*(ratio**2)
                    resistance=r_end2*(ratio**2)
                    
                if (domain==Domain.EMT):
                    transformer.set_parameters(nom_voltage_end_1=rated_voltage_end1, nom_voltage_end_2=rated_voltage_end2, 
                                               ratio_abs=ratio, ratio_phase=0, 
                                               resistance=dpsimpy.Math.single_phase_parameter_to_three_phase(resistance), 
                                               inductance=dpsimpy.Math.single_phase_parameter_to_three_phase(inductance))
                else:
                    transformer.set_parameters(nom_voltage_end_1=rated_voltage_end1, nom_voltage_end_2=rated_voltage_end2, 
                                               ratio_abs=ratio, ratio_phase=0, resistance=resistance, inductance=inductance)
                if (domain==Domain.PF): 
                    transformer.set_base_voltage(rated_voltage_end1)
                logging.debug('Created PowerTransformer: name={}, nom_voltage_end_1={}, nom_voltage_end_2={}, ratio_abs={}, ratio_phase={}, resistance={}, inductance={}'.format(
                                res[i].name, rated_voltage_end1, rated_voltage_end2, ratio, 0, resistance, inductance))
            elif (rated_voltage_end2>=rated_voltage_end1): 
                # high voltage side is end2
                ratio=rated_voltage_end2/rated_voltage_end1
                if (np.abs(x_end2)>1e-12):
                    # transformer impedance is refered to end2
                    inductance=x_end2/omega
                    resistance=r_end2
                elif (np.abs(x_end1)>1e-12):
                    #transformer impedance is refered to end1 --> end2
                    inductance=(x_end1/omega)*(ratio**2)
                    resistance=r_end1*(ratio**2)
                if (domain==Domain.EMT):
                    transformer.set_parameters(nom_voltage_end_1=rated_voltage_end1, nom_voltage_end_2=rated_voltage_end2, 
                                               ratio_abs=ratio, ratio_phase=0, 
                                               resistance=dpsimpy.Math.single_phase_parameter_to_three_phase(resistance), 
                                               inductance=dpsimpy.Math.single_phase_parameter_to_three_phase(inductance))
                else:
                    transformer.set_parameters(nom_voltage_end_1=rated_voltage_end2, nom_voltage_end_2=rated_voltage_end1, 
                                               ratio_abs=ratio, ratio_phase=0, resistance=resistance, inductance=inductance)
                if (domain==Domain.PF): 
                    transformer.set_base_voltage(rated_voltage_end2)
                logging.debug('Created PowerTransformer: name={}, nom_voltage_end_1={}, nom_voltage_end_2={}, ratio_abs={}, ratio_phase={}, resistance={}, inductance={}'.format(
                                res[i].name, rated_voltage_end2, rated_voltage_end1, ratio, 0, resistance, inductance))
            
            components_dict[res[i].mRID] = {"Element": transformer, "Nodes": [None, None]}
            

    ### Nodes
    for j in res:
        if isinstance(res[j], cimpy.cgmes_v2_4_15.TopologicalNode):
            node = None
            if (domain == Domain.EMT):      
                node = dpsimpy_components.SimNode(res[j].mRID, dpsimpy.PhaseType.ABC)
            else:
                node = dpsimpy_components.SimNode(res[j].mRID, dpsimpy.PhaseType.Single)     

            # set name of node
            node.attr("name").set(res[j].name)
            
            # add node to list of nodes
            nodes[res[j].mRID] = node
            logging.debug('Created Node: name={}'.format(res[j].name))
            
            # get terminals of node            
            terminals = res[j].Terminal
            
            # connect component to nodes via terminals
            for terminal in terminals:                         
                component_mRID = terminal.ConductingEquipment.mRID
                if isinstance(terminal.ConductingEquipment, cimpy.cgmes_v2_4_15.SynchronousMachine) and (len(SynchronousMachineTCR_Dict) != 0):
                    # Match the Nodes from SyncMachine to SynchMachineTCR    
                    for syn_machine_tcr_mRID in SynchronousMachineTCR_Dict:
                        if terminal.ConductingEquipment.mRID == SynchronousMachineTCR_Dict[syn_machine_tcr_mRID].mRID:
                            component_mRID = syn_machine_tcr_mRID
                            break
                    components_dict[component_mRID]["Nodes"].append(node)
                    
                elif isinstance(terminal.ConductingEquipment, cimpy.cgmes_v2_4_15.PowerTransformer):
                    trafo = terminal.ConductingEquipment
                    end1=trafo.PowerTransformerEnd[0] if trafo.PowerTransformerEnd[0].endNumber==1 else trafo.PowerTransformerEnd[1]
                    end2=trafo.PowerTransformerEnd[1] if trafo.PowerTransformerEnd[1].endNumber==2 else trafo.PowerTransformerEnd[0]
                    # get voltages
                    rated_voltage_end1 = end1.ratedU, Multiplier.k
                    rated_voltage_end2 = end2.ratedU, Multiplier.k
                                                
                    if (rated_voltage_end1>=rated_voltage_end2):
                        # HV side is the first node, LV side is the second node
                        if (end1.Terminal.TopologicalNode.mRID==res[j].mRID):
                            components_dict[component_mRID]["Nodes"][0]=node
                        elif (end2.Terminal.TopologicalNode.mRID==res[j].mRID):
                            components_dict[component_mRID]["Nodes"][1]=node
                    else:
                        # LV side is the first node, HV side is the second node
                        if (end1.Terminal.TopologicalNode.mRID==res[j].mRID):
                            components_dict[component_mRID]["Nodes"][1]=node
                        elif (end2.Terminal.TopologicalNode.mRID==res[j].mRID):
                            components_dict[component_mRID]["Nodes"][0]=node
                else:
                    if (component_mRID in components_dict):
                        components_dict[component_mRID]["Nodes"].append(node)
                   
                   
    # Connect the components with nodes (dpsimpy components)
    component_list = []                                             
    for comp_ID in components_dict:
        components_dict[comp_ID]["Element"].connect(components_dict[comp_ID]["Nodes"])                  
        component_list.append(components_dict[comp_ID]["Element"])
   
    # create SystemTopology object
    node_list = list(nodes.values())
    system = dpsimpy.SystemTopology(frequency, node_list, component_list)

    return system

def get_powerflow_results(CIM_network):
    # This function return the power flow results of the nodes which are stored in the SV profile

    res = CIM_network["topology"]
    pf_results = pd.DataFrame(columns=['Bus', 'Vmag [kV]', 'Vangle [rad]', 'P [MW]', 'Q [MVAr]'])
    
    # create list with all SvVoltage objects
    SvVoltage_list = []
    for mRID, comp in res.items():
        if isinstance(comp, cimpy.cgmes_v2_4_15.SvVoltage):
            SvVoltage_list.append(comp)
            
    # create list with all SvPowerFlow objects
    SvPowerFlow_list = []
    for mRID, comp in res.items():
        if isinstance(comp, cimpy.cgmes_v2_4_15.SvPowerFlow):
            SvPowerFlow_list.append(comp)
         
    # iterate throw all nodes
    for mRID, node in res.items():
        if isinstance(node, cimpy.cgmes_v2_4_15.TopologicalNode):
            
            p = 0
            q = 0
            v_mag = 0
            v_angle = 0
            
            # get node voltage
            for elem in SvVoltage_list:
                if mRID==elem.TopologicalNode.mRID:
                    #v_mag=unitValue(elem.v, Multiplier.k)
                    v_mag=elem.v
                    v_angle=elem.angle*np.pi/180
                    break

            # get terminals of node            
            terminals = node.Terminal
            
            # get node power = -Gen. power - Load power
            for terminal in terminals:                         
                if isinstance(terminal.ConductingEquipment, cimpy.cgmes_v2_4_15.SynchronousMachine):
                   for pf_elem in SvPowerFlow_list:
                        if pf_elem.Terminal.mRID==terminal.mRID:
                            p = p - pf_elem.p
                            q = q - pf_elem.q
                if (isinstance(terminal.ConductingEquipment, cimpy.cgmes_v2_4_15.EnergyConsumer) or 
                   isinstance(terminal.ConductingEquipment, cimpy.cgmes_v2_4_15.ConformLoad)):
                    for pf_elem in SvPowerFlow_list:
                        if pf_elem.Terminal.mRID==terminal.mRID:
                            p = p - pf_elem.p
                            q = q - pf_elem.q
                            
            pf_results.loc[len(pf_results)] = {'Bus': node.name, 'Vmag [kV]':v_mag, 'Vangle [rad]':v_angle, 'P [MW]':p, 'Q [MVAr]':q}       

    return pf_results