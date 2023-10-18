from enum import Enum
import numpy as np
import logging
from cimpy import utils
import sys
sys.path.insert(0,'/home/mmo-cya/dpsim/build')
import dpsimpy

# configure logging
logging.basicConfig(filename='DPsim2CIM.log', encoding='utf-8', level=logging.DEBUG)

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
    
def node_of_comp(system, comp_name):
    """
    node_of_comp returns  a list containing the nodes connected with this component
    
    :param comp_name:
    """ 
    
    comp_ptr = None
    for comp in system.components:
        if comp.name()==comp_name:
            comp_ptr = comp

    node_list = []
    for node, comp in system.components_at_node.items():
        if (comp_ptr in comp):
            node_list.append(node)
            
    return node_list

def DPsimToCIMpy(DPsim_system_PF, DPsim_simulation, DPsim_system_dyn=None, frequency=60):
    # network is a Dictionary for CIM Objects and the output of this function
    cim_topology = {'meta_info': {
                'namespaces': {
                    'rdf': 'http://www.w3.org/1999/02/22-rdf-syntax-ns#',
                    'cim': 'http://iec.ch/TC57/2013/CIM-schema-cim16#',
                    'md': 'http://iec.ch/TC57/61970-552/ModelDescription/1#',
                    'entsoe': 'http://entsoe.eu/CIM/SchemaExtension/3/1#'},            
                'urls': {}, 
                'author': 'DPsim'},
                'topology': {}
                }
    
    for node in DPsim_system_PF.nodes:
        v = np.linalg.norm(DPsim_simulation.get_idobj_attr(node.name(), 'v').get()[0])
        v = unitValue(v, Multiplier.m) 
        angle = np.angle(DPsim_simulation.get_idobj_attr(node.name(), 'v').get()[0])
        baseVoltage = unitValue(get_node_base_voltage(DPsim_system_PF, node), Multiplier.m)
        uid = DPsim_simulation.get_idobj_attr(node.name(), 'uid').get()             
        cim_topology = utils.add_TopologicalNode(cim_topology=cim_topology, version="cgmes_v2_4_15", 
                                                 name=node.name(), mRID=uid, baseVoltage=baseVoltage, v=v, angle=angle)
        logging.debug('Added TopologicalNode: \n\tname={}\n\tbaseVoltage={}[kV]\n\tv={}\n\tangle={}[rad]"'.format(node.name(), baseVoltage, v, angle))

    for comp in DPsim_system_PF.components:
        if isinstance(comp, dpsimpy.sp.ph1.PiLine):
            omega = 2*np.pi*frequency
            r = comp.attr("R_series").get()
            x = comp.attr("L_series").get() * omega
            bch = comp.attr("C_parallel").get() * omega
            gch = comp.attr("G_parallel").get()
            base_voltage = unitValue(comp.attr("base_Voltage").get(), Multiplier.m) 
            
            # determine the connected Nodes
            node_list =  node_of_comp(DPsim_system_PF, comp.name())
            cim_topology = utils.add_ACLineSegment(cim_topology=cim_topology, version="cgmes_v2_4_15", name=comp.name(), 
                                                   node1_name=node_list[0].name(), node2_name=node_list[1].name(), r=r, 
                                                   x=x, bch=bch, gch=gch, baseVoltage=baseVoltage)        
                
            logging.debug('Added PiLine: \n\tname={}, \n\tL={}[H], \n\tR={}[Ohm], \n\tC={}[F], \n\tG={}[S], \n\tbaseVoltage={}[kV], \n\tnode1 name={}, \n\tnode2 name={}'.format(
                    comp.name(), r, x, bch, gch, base_voltage, node_list[0].name, node_list[1].name))

        elif "Load" in str(type(comp)):
            p = unitValue(comp.attr("P").get(), Multiplier.micro) 
            q = unitValue(comp.attr("Q").get(), Multiplier.micro) 
            Vnom = unitValue(comp.attr("V_nom").get(), Multiplier.m) 
            
            #determine the connected Node
            node = node_of_comp(DPsim_system_PF, comp.name())
            cim_topology = utils.add_EnergyConsumer(cim_topology=cim_topology, version="cgmes_v2_4_15", name=comp.name(), 
                             node_name=node[0].name(), p_nom=p, q_nom=q, p_init=p, q_init=q, baseVoltage=Vnom)
            logging.debug('Added Load: \n\tname={}, \n\tp={}[MW], \n\tq={}[MVAr], \n\tbaseVoltage={}[kV], \n\tnode name={}'.format(
                            comp.name, p, q, Vnom, node[0].name))

        elif "Transformer" in str(type(comp)):
            r = comp.attr("R").get()
            x = comp.attr("L").get()
            mNominalVoltageEnd1 = unitValue(comp.attr("nominal_voltage_end1").get(), Multiplier.m) 
            mNominalVoltageEnd2 = unitValue(comp.attr("nominal_voltage_end2").get(), Multiplier.m) 
            base_voltage =  unitValue(comp.attr("base_Voltage").get(), Multiplier.m) 
            
            #determine the connected Node
            node_list = node_of_comp(DPsim_system_PF, comp.name())
            base_voltage_n1 = get_node_base_voltage(DPsim_system_PF, node_list[0])
            base_voltage_n2 = get_node_base_voltage(DPsim_system_PF, node_list[1])
            if (base_voltage_n1 >= unitValue(base_voltage_n2), Multiplier.m) :
                node1_name = node_list[0].name()
                node2_name = node_list[1].name()
                mNominalVoltageEnd1 = unitValue(base_voltage_n1, Multiplier.m)
                mNominalVoltageEnd2 = unitValue(base_voltage_n2, Multiplier.m)
            else:
                node2_name = node_list[0].name()
                node1_name = node_list[1].name()
                mNominalVoltageEnd2 = unitValue(base_voltage_n1, Multiplier.m)
                mNominalVoltageEnd1 = unitValue(base_voltage_n2, Multiplier.m)
                
            cim_topology = utils.add_PowerTransfomer(cim_topology=cim_topology, version="cgmes_v2_4_15", name=comp.name(),
                                                node1_name=node1_name, node2_name=node2_name, 
                                                r=r, x=x, nominal_voltage_end1=mNominalVoltageEnd1, nominal_voltage_end2=mNominalVoltageEnd2)
            logging.debug('Added Transformer: \n\tname={}, \n\tr={}[Ohm], \n\tx={}[Ohm], \n\tbaseVoltage={}[kV], \n\tnode1 name={}, \n\tnode2 name={}'.format(
                            comp.name(), r, x, base_voltage, node1_name, node2_name))
    
        elif "SynchronGenerator":
            p = unitValue(comp.get_apparent_power().real, Multiplier.micro)
            q = unitValue(comp.get_apparent_power().imag, Multiplier.micro)
            # TODO: THIS VALUE IS NOT THE BASE POWER
            ratedS = unitValue(np.abs(comp.get_apparent_power()), Multiplier.micro)   # TODO: CHECK! MAYBE USE mAppparentPower?
            ratedU = unitValue(getattr(comp, "Vnom", 0), Multiplier.m)
            targetValue = unitValue(comp.attr("V_set").get(), Multiplier.m)
            initialP = p

            #determine the connected Node
            node = node_of_comp(DPsim_system_PF, comp.name())
            
            # Add Synchronous Machine to the network
            cim_topology = utils.add_SynchronousMachine(cim_topology=cim_topology, version="cgmes_v2_4_15", name=comp.name(), node_name=node[0].name(), 
                                                        ratedS=ratedS, ratedU=ratedU, p=p, q=q, targetValue=targetValue, initialP=initialP)
            logging.debug('Added SynchronGenerator: \n\tname={}, \n\tp={}[MW], \n\tq={}[MVAr], \n\tratedU={}[kV], \n\tratedS={}[MVA], \n\ttargetValue={}, \n\tinitial={}, \n\tnode name={}'.format(
                    comp.name(), p, q, ratedU, ratedS, targetValue, initialP, node[0].name()))
            
            if (DPsim_system_dyn is not None):
                dyn_comp = DPsim_system_dyn.component(comp.name())
                # Synchronous Machine TimeConstantReactance Parameters
                inertia = dyn_comp.attr("H").get()
                statorResistance = 0            # TODO FIX IN DPSIM
                statorLeakageReactance = 0      # TODO FIX IN DPSIM
                tpdo = dyn_comp.attr("Td0_t").get()
                tpqo = dyn_comp.attr("Tq0_t").get()
                tppdo = dyn_comp.attr("Td0_s").get()
                tppqo = dyn_comp.attr("Tq0_s").get()
                xDirectSubtrans = dyn_comp.attr("Ld_s").get()
                xDirectSync = dyn_comp.attr("Ld").get()
                xDirectTrans = dyn_comp.attr("Ld_t").get()
                xQuadSubtrans = dyn_comp.attr("Lq_s").get()
                xQuadSync =  dyn_comp.attr("Lq").get()
                xQuadTrans = dyn_comp.attr("Lq_t").get()
                
                # search for mRID of the SG
                sg_mRID = ""
                for cim_comp_mRID, cim_comp in cim_topology["topology"].items():
                    if hasattr(cim_comp, "name"):
                        if cim_comp.name == comp.name():
                            sg_mRID=cim_comp_mRID
                    
                # Extend SynchronousMachine with dynamic Parameters
                cim_topology = utils.extend_SynchronousMachineTimeConstantReactance(cim_topology=cim_topology, version="cgmes_v2_4_15", SynchronousMachine_mRID=sg_mRID, 
                                                                                damping=0, inertia=inertia, statorResistance=statorResistance, statorLeakageReactance=statorLeakageReactance, 
                                                                                tpdo=tpdo, tpqo=tpqo, tppdo=tppdo, tppqo=tppqo, xDirectSubtrans=xDirectSubtrans, xDirectSync=xDirectSync, 
                                                                                xDirectTrans=xDirectTrans, xQuadSubtrans=xQuadSubtrans, xQuadSync=xQuadSync, xQuadTrans=xQuadTrans)
                logging.debug('Added SynchronousMachineTimeConstantReactance: \n\tSynchronousMachine mRID={}, \n\tDamping={}, \n\tH={}, \n\tstatorResistance={}[pu], \n\tstatorLeakageReactance={}[pu], \n\ttpdo={}[s], \n\ttpqo={}[s], \n\ttppdo={}[s], \n\ttppqo={}[s], \n\txDirectSubtrans={}[pu], \n\txDirectSync={}[pu], \n\txDirectTrans={}[pu], \n\txQuadSubtrans={}[pu], \n\txQuadSync={}[pu], \n\txQuadTrans={}[pu]'.format(
                                                                                  sg_mRID, 0, inertia, statorResistance, statorLeakageReactance, tpdo, tpqo, tppdo, tppqo, xDirectSubtrans, xDirectSync, xDirectTrans, xQuadSubtrans, xQuadSync, xQuadTrans))
        """            
        elif "NetworkInjection" in str(type(comp)):
            # determine the connected Node
            Node_name = node_of_comp(DPsim_system, comp.uid)
            cim_topology = utils.add_external_network_injection(cim_topology, "cgmes_v2_4_15", Node_name, 1)
        """
        
    return cim_topology


def get_node_base_voltage(DPsim_system_PF, node):
    base_voltage = 0

    for comp in DPsim_system_PF.components_at_node[node]:
        if isinstance(comp, dpsimpy.sp.ph1.AvVoltageSourceInverterDQ):
            base_voltage = unitValue(np.mag(comp.attr("vnom").get()), Multiplier.m) 
            logging.info('Choose base voltage {}kV of {} for node={}'.format(base_voltage, comp.name(), node.name()))
            break
        elif isinstance(comp, dpsimpy.sp.ph1.PiLine):
            base_voltage = unitValue(comp.attr("base_Voltage").get(), Multiplier.m)   
            logging.info('Choose base voltage {}kV of {} for node {}'.format(base_voltage, comp.name(), node.name()))
            break
        elif isinstance(comp, dpsimpy.sp.ph1.Transformer):
            if comp.get_terminal(0).node().name() == node.name():
                base_voltage = unitValue(comp.attr("nominal_voltage_end1").get(), Multiplier.m)
                logging.info('Choose base voltage {}kV of {} for node {} TransformerEnd: 1'.format(base_voltage, comp.name(), node.name()))
                break
            elif comp.get_terminal(1).node().name() == node.name():
                base_voltage = unitValue(comp.attr("nominal_voltage_end2").get(), Multiplier.m)
                logging.info('Choose base voltage {}kV of {} for node {} TransformerEnd: 2'.format(base_voltage, comp.name(), node.name()))
                break
        elif isinstance(comp, dpsimpy.sp.ph1.SynchronGenerator):
            base_voltage = unitValue(comp.attr("base_Voltage").get(), Multiplier.m)   
            logging.info('Choose base voltage {}kV of {} for node {}'.format(base_voltage, comp.name(), node.name()))
            break
        elif isinstance(comp, dpsimpy.sp.ph1.SynchronGenerator):
            base_voltage = unitValue(comp.attr("base_Voltage").get(), Multiplier.m)   
            logging.info('Choose base voltage {}kV of {} for node {}'.format(base_voltage, comp.name(), node.name()))
            break  
        #elif isinstance(comp, dpsimpy.sp.ph1.Load):
        #    print(comp)
        #    print(comp.attr("base_Voltage"))
        #    base_voltage = unitValue(comp.attr("base_Voltage"), Multiplier.m)   
        #    logging.info('Choose base voltage {}kV of {} for node "{}"', base_voltage, comp.name(), node.namev)
        #    break
        elif isinstance(comp, dpsimpy.sp.ph1.NetworkInjection):
            base_voltage = unitValue(comp.attr("V_nom").get(), Multiplier.m)   
            logging.info('Choose base voltage {}kV of {} for node {}'.format(base_voltage, comp.name(), node.name()))
            break
        else:
             logging.info('Unable to get base voltage at {}'.format(node.name()))
                     
    return base_voltage