import warnings
import pandas as pd 
import numpy as np
import json
import cimpy

import sys
sys.path.insert(0,'/home/mmo/git/Can/dpsim/build')
import dpsimpy

import CIM2DPsim
from CIM2DPsim import Domain
from CIM2DPsim import SGModels

TIME_STEP_PF = 0.1
FINAL_TIME_PF = 0.1

class DPsimLauncher:
    def __init__(self, file_path = '/home/mmo-cya/dpsim/python/src/dpsim/Config_network.json'):
        # data contained in the json file
        self.data = {}
        # cimpy topology
        self.cimpy_topology = None
        # log level used in simulations
        self.loglevel = dpsimpy.LogLevel.info
        # list of domains used in simulations
        self.domains = []
        # simulation start time
        self.start_time = 0
        # simulation end time
        self.end_time = 1
        # simulation step time
        self.time_steps = []
        # simulation name
        self.sim_name = "sim_name"
        # syn gen model used in dynamic simulations
        self.syn_gen_model = SGModels.VBR_4Order
        # system frequency
        self.frequency = 60
        #
        self.init_from_nodes_and_terminals = True
        
        # power flow simulations
        self.init_from_power_flow = True
        self.sim_name_pf = ""
        self.system_pf = None
        self.sim_pf = None
        
        # dynamic simulation
        self.system = None
        self.sim = None
        
        # read json file
        self.read_json(file_path)
                
    def read_json(self, file_path):
        """
        Diese Funktion liest eine Konfigurationsdatei im JSON-Format ein, in der der Dateipfad zu dem CIM-Netzwerk und deren Parameter zur Simulation
        mittels DPsim enthalten sind. Das Netzwerk wird eingelesen und ins DPsim-Format übersetzt. Der ausgewählte Fehler wird dem Netzwerk hinzugefügt und anschließend wird 
        die Simulation ausgeführt. Ausgegeben werden das neue DPsim Netzwerk und die Simulationsergebnisse mit dem gewünschten Solver.
        """
        
        # Read json file
        try:
            with open(file_path, 'r') as json_file:
                self.data = json.load(json_file)
        except FileNotFoundError:
            raise Exception(f"Die Datei '{file_path}' can not be found!")
        except json.JSONDecodeError:
            raise Exception(f"The file '{file_path}' is not a valid JSON File!")

        # Convert input topology to cimpy
        self.cimpy_topology = cimpy.cim_import(self.data['CIMFiles'], 'cgmes_v2_4_15')
    
        # get loglevel
        if "LogLevel" in self.data["SimulationParameters"].keys():
            if self.data["SimulationParameters"]["LogLevel"] == "debug":
                self.loglevel = dpsimpy.LogLevel.debug
            elif self.data["SimulationParameters"]["LogLevel"] == "info":
                self.loglevel = dpsimpy.LogLevel.info
            elif self.data["SimulationParameters"]["LogLevel"] == "off":
                self.loglevel = dpsimpy.LogLevel.off            
    
        # get domain
        if "Domains" in self.data["SolverParameters"].keys():
            for domain in self.data["SolverParameters"]["Domains"]:
                if domain == "RMS":
                    self.domains.append(Domain.SP)
                elif domain == "DP":
                    self.domains.append(Domain.DP)
                elif domain == "EMT":
                    self.domains.append(Domain.EMT)
                else:
                    raise Exception('ERROR: domain {} is not supported in dpsimpy'.format(self.data["SolverParameters"]["Domain"]))

        # get start time
        if "StartTime" in self.data["SimulationParameters"].keys():
            self.start_time = self.data["SimulationParameters"]["StartTime"]

        # get end time
        if "EndTime" in self.data["SimulationParameters"].keys():
            self.end_time = self.data["SimulationParameters"]["EndTime"]

        # get time step
        if "TimeSteps" in self.data["SimulationParameters"].keys():
            self.time_steps = self.data["SimulationParameters"]["TimeSteps"]  

        # get simulation name
        if "SimulationName" in self.data["SimulationParameters"].keys():
            self.sim_name = self.data["SimulationParameters"]["SimulationName"] 

        # check if pf simulation for initializartion has to be performed
        if "InitFromPF" in (self.data['SimulationParameters']):
            self.init_from_power_flow = self.data['SimulationParameters']["InitFromPF"]
            
        # check if pf simulation for initializartion has to be performed
        if "InitFromNodesAndTerminals" in (self.data['SimulationParameters']):
            self.init_from_nodes_and_terminals = self.data['SimulationParameters']["InitFromNodesAndTerminals"]

        # get syn gen model to use in dynamic simulation
        # TODO: ADD MORE SYNGEN MODELS
        if "SGModel" in self.data["SimulationParameters"].keys():
            if self.data["SimulationParameters"] == "3Order":
                self.syn_gen_model = SGModels.VBR_3Order
            elif self.data["SimulationParameters"] == "4Order":
                self.syn_gen_model = SGModels.VBR_4Order
            elif self.data["SimulationParameters"] == "5Order":
                self.syn_gen_model = SGModels.VBR_5Order
            elif self.data["SimulationParameters"] == "6Order":
                self.syn_gen_model = SGModels.VBR_6Order
            else:
                # TODO: WARN MESSAGE
                pass
            
    def read_variables_to_log(self, system, sim, logger):
        # TODO: Add more variables and components
        for component_type, var_dict in self.data["LoggerVariables"].items():
            if component_type=="Node":
                for variable in var_dict.keys():
                    if variable=="Voltages":
                        if (var_dict["Voltages"][0]=="all"):
                            for node in system.nodes:
                                logger.log_attribute(node.name()+'.V', 'v', node)
                        else:
                            for node_name in var_dict["Voltages"]:
                                # search for node in sim_pf
                                node_found = False
                                for node in system.nodes:
                                    if (node.name() == node_name):
                                        node_found = True
                                        break
                                if (node_found==False):
                                    # TODO: change exception by warning
                                    raise Exception('Node {} was not found!'.format(node_name))

                                logger.log_attribute(node_name+'.V', 'v', node)
                    elif variable=="Power":
                        if (var_dict["Power"][0]=="all"):
                            for node in system.nodes:
                                logger.log_attribute(node.name()+'.S', 's', node)
                        else:
                            for node_name in var_dict["Power"]:
                                # search for node in sim_pf
                                node_found = False
                                for node in system.nodes:
                                    if (node.name() == node_name):
                                        node_found = True
                                        break
                                if (node_found==False):
                                    # TODO: change exception by warning
                                    raise Exception('Node {} was not found!'.format(node_name))
                                logger.log_attribute(node_name+'.S', 's', node)
            elif component_type=="Component":
                for variable in var_dict.keys():
                    if variable=="Currents":
                        if (var_dict["Currents"][0]=="all"):
                            for comp in system.components:
                                logger.log_attribute(comp.name()+'.I', 'i_intf', comp)
                        else:
                            for comp_name in var_dict["Currents"]:
                                # search for component in system topology
                                comp_found = False
                                for comp in system.components:
                                    if (comp.name() == comp_name):
                                        comp_found = True
                                        break
                                if (comp_found==False):
                                    # TODO: change exception by warning?
                                    raise Exception('Component {} was not found!'.format(comp_name))
                                
                                logger.log_attribute(comp_name+'.I', 'i_intf', comp)
                    else:
                        # Generic
                        # All attributes of components can be logged provided that they exist
                        for comp_name in var_dict[variable]:
                            # search for component in system topology
                            comp_found = False
                            comp = None
                            for comp in system.components:
                                if (comp.name() == comp_name):
                                    comp_found = True
                                    break
                            if (comp_found==False):
                                # TODO: change exception by warning?
                                warnings.warn('The component {} was not found. The attribute {} will no be logged!'.format(comp.name(), variable))
                            
                            # check if attribute exists
                            
                            try:
                                sim.get_idobj_attr(comp.name(), variable)
                                logger.log_attribute(comp.name()+'.{}'.format(variable), variable, comp)
                            except:
                                warnings.warn('The component {} has no attribute "{}". This attribute will no be logged!'.format(comp.name(), variable))
                                continue
                            
                        
        return logger
    
    def start_pf_simulation(self):
        if self.init_from_power_flow:
            self.run_pf_simulation()
        else:
            #TODO
            #load initial pf results from cim files
            pass
    
    def run_pf_simulation(self):
        # 
        self.sim_name_pf = self.sim_name + "_PF"
        dpsimpy.Logger.set_log_dir("logs/" + self.sim_name_pf)

        # create dpsim topology for pf simulation for initialization
        self.system_pf = CIM2DPsim.CIM2DPsim(self.cimpy_topology, domain=Domain.PF, log_level=self.loglevel, frequency=self.frequency)
    
        #set reference node 
        reference_comp=None
        for node, comp_list in self.system_pf.components_at_node.items():
            if (node.name()==self.data['SimulationParameters']['ReferenceNode']):
                for comp in comp_list:
                    if (isinstance(comp, dpsimpy.sp.ph1.SynchronGenerator) or isinstance(comp, dpsimpy.sp.ph1.NetworkInjektion)):
                        reference_comp=comp
                        break
                if (reference_comp is None):  
                    raise Exception('No SynchronGenerator or ExternalNetworkInjection is connected to node: {}!'.format(node.name()))    
        if (reference_comp is None):  
            raise Exception('No node named: {} was found!'.format(node.name()))
    
        reference_comp.modify_power_flow_bus_type(dpsimpy.PowerflowBusType.VD)
    
        # create logger add variables to be logged
        logger = dpsimpy.Logger(self.sim_name_pf)
        logger = self.read_variables_to_log(self.system_pf, self.sim_pf, logger)
        
        # start power flow simulation
        self.sim_pf = dpsimpy.Simulation(self.sim_name_pf, self.loglevel)
        self.sim_pf.set_system(self.system_pf)
        self.sim_pf.set_domain(dpsimpy.Domain.SP)
        self.sim_pf.set_solver(dpsimpy.Solver.NRP)
        #self.sim_pf.set_solver_component_behaviour(dpsimpy.SolverBehaviour.Initialization)
        self.sim_pf.do_init_from_nodes_and_terminals(False)
        self.sim_pf.add_logger(logger)
        self.sim_pf.set_time_step(TIME_STEP_PF)
        self.sim_pf.set_final_time(FINAL_TIME_PF)
        self.sim_pf.run()        
    
    def get_pf_results(self):
        # return table contianing the node voltages (in kV) and node powers  (in MW)
        pf_results = pd.DataFrame(columns=['Bus', 'V_mag[kV]', 'V_angle[deg]', 'P[MW]', 'Q[MW]'])
        for idx, node in enumerate(self.system_pf.nodes):
            voltage=node.attr("v").get()[0][0]
            power=node.attr("s").get()[0][0]
            pf_results.loc[idx] = ([node.name()] + [round(np.absolute(voltage) / 1000, 4)]
                + [round(np.angle(voltage*180/np.pi), 4)] 
                + [round(1e-6 * np.real(power), 4)] 
                + [round(1e-6 * np.imag(power), 5)])

        return pf_results

    def run_dynamic_simulations(self):
        for domain in self.domains:
            for time_step in self.time_steps:
                self.start_dynamic_simulation(domain, time_step)
        
    def start_dynamic_simulation(self, domain, time_step):
        self.name_dyn = self.sim_name + "_" + domain.name + "_TS" + str(time_step)
        dpsimpy.Logger.set_log_dir("logs/" + self.name_dyn)

         # create dpsim topology for dynamic simulation
        self.system = CIM2DPsim.CIM2DPsim(self.cimpy_topology, domain=domain, log_level=self.loglevel, frequency=self.frequency, gen_model=self.syn_gen_model)
        
        ### Simulation
        self.sim = dpsimpy.Simulation(self.name_dyn, self.loglevel)
        self.sim.set_system(self.system)
        
        #
        if domain == Domain.SP:
            self.sim.set_domain(dpsimpy.Domain.SP)
            self.system.init_with_powerflow(systemPF=self.system_pf, domain=dpsimpy.Domain.SP)
        elif domain == Domain.DP:
            self.sim.set_domain(dpsimpy.Domain.DP)
            self.system.init_with_powerflow(systemPF=self.system_pf, domain=dpsimpy.Domain.DP)
        elif domain == Domain.EMT:
            self.sim.set_domain(dpsimpy.Domain.EMT)
            self.system.init_with_powerflow(systemPF=self.system_pf, domain=dpsimpy.Domain.EMT)
        
        # add events
        self.read_events(domain)
        self.sim.set_system(self.system)
        
        #
        self.sim.do_init_from_nodes_and_terminals(self.init_from_nodes_and_terminals)
        # TODO: ADD SOLVERIMPLEMENTATION to json file
        self.sim.set_direct_solver_implementation(dpsimpy.DirectLinearSolverImpl.SparseLU)
        # TODO: DPSim: AUTOMATIC DETECTION OF system_matrix_recomputation?
        self.sim.do_system_matrix_recomputation(True)
        #
        #self.sim.set_start_time(self.start_time)
        self.sim.set_time_step(time_step)
        self.sim.set_final_time(self.end_time)
        
        # create logger add variables to be logged
        logger = dpsimpy.Logger(self.name_dyn)
        logger = self.read_variables_to_log(self.system, self.sim, logger)
        self.sim.add_logger(logger)
        
        # run simulation
        self.sim.run()
        
    def read_events(self, domain):
        # Füge der Topologie den gewünschten Event hinzu
        if "Events" in self.data:
            if self.data["Events"]['EventType'] == "ShortCircuit":
                event_params = self.data["Events"]['EventParameters']
                
                # get event parameters
                
                # get event start time
                event_start_time = 0
                if "EventStartTime" in self.data["Events"].keys():
                    event_start_time = self.data["Events"]["EventStartTime"]
                else:
                    raise Exception('Paramenter "EventStartTime" is not in the json file!') 
                
                # get event end time
                event_end_time = 0
                if "EventEndTime" in self.data["Events"].keys():
                    event_end_time = self.data["Events"]["EventEndTime"]  
                else:
                    raise Exception('Paramenter "EventEndTime" is not in the json file!')
                
                # get name of nodes to which the switch has to be connected
                node_name = ""
                if "NodeName" in event_params.keys():
                    node_name = event_params['NodeName']
                else:
                    raise Exception('Paramenter "NodeName" is not in the json file!')
                    
                # get node to which the short switch has to be connected
                node = None
                for node_ in self.system.nodes:
                    if node_name == node_.name():
                        node = node_
                        break
                if (node is None):
                    # TODO: change exception by warning
                    raise Exception('Node {} was not found!'.format(node_name()))   
                    
                open_resistance = 1e12
                if "FaultOpenResistance" in event_params.keys():
                    open_resistance = event_params['FaultOpenResistance']
                else:
                    warnings.warn('Paramenter "FaultOpenResistance" is not in the json file!\n FaultOpenResistance set to 1e12')
                    
                closed_resistance = 1e-3  
                if "FaultClosedResistance" in event_params.keys():
                    closed_resistance = event_params['FaultClosedResistance']
                else:
                    warnings.warn('Paramenter "FaultClosedResistance" is not in the json file!\n FaultClosedResistance set to 1e-3')
                    
                # Füge switch mit Erdung hinzu
                switch = None
                gnd = None
                if domain == Domain.SP:
                    switch = dpsimpy.sp.ph1.Switch('Fault_' + node_name, self.loglevel)
                    gnd = dpsimpy.sp.SimNode.gnd
                elif domain == Domain.DP:
                    switch = dpsimpy.dp.ph1.Switch('Fault_' + node_name, self.loglevel)
                    gnd = dpsimpy.dp.SimNode.gnd
                elif domain == Domain.EMT:
                    switch = dpsimpy.emt.ph3.SeriesSwitch('Fault_' + node_name, self.loglevel)
                    gnd = dpsimpy.emt.SimNode.gnd 
                switch.set_parameters(open_resistance, closed_resistance)
                switch.open()
                self.system.add(switch)
                self.system.connect_component(switch, [gnd, node])

                # Event hinzufügen           
                sw_event_1 = dpsimpy.event.SwitchEvent(event_start_time, switch, True)
                self.sim.add_event(sw_event_1)
                sw_event_2 = dpsimpy.event.SwitchEvent(event_end_time, switch, False)
                self.sim.add_event(sw_event_2)
            
            elif self.data["Events"]['EventType'] == "LoadStep":
                event_params = self.data["Events"]['EventParameters']
                
                # get event start time
                event_start_time = 0
                if "EventStartTime" in self.data["Events"].keys():
                    event_start_time = self.data["Events"]["EventStartTime"]
                else:
                    raise Exception('Paramenter "EventStartTime" is not in the json file!') 
                
                # get event end time
                event_end_time = 999999999
                if "EventEndTime" in self.data["Events"].keys():
                    event_end_time = self.data["Events"]["EventEndTime"]  
                else:
                    warnings.warn('Paramenter "EventEndTime" is not in the json file! EventEndTime set to {}'.format(event_end_time))
                
                node_name = ""
                if "NodeName" in event_params.keys():
                    node_name = event_params['NodeName']
                else:
                    raise Exception('Paramenter "NodeName" is not in the json file!')
                
                node = None
                for node_ in self.system.nodes:
                    if node_name == node_.name():
                        node = node_
                        break
                if (node is None):
                    # TODO: change exception by warning
                    raise Exception('Node {} was not found!'.format(node_name))   
                
                open_resistance = 1e18
                closed_resistance = None
                additional_active_power = 0
                additional_reactive_power = 0
                if "AdditionalActivePower" in event_params.keys():
                    additional_active_power = event_params['AdditionalActivePower'] * 1e6
                if "AdditionalReactivePower" in event_params.keys():
                    additional_reactive_power = event_params['AdditionalReactivePower'] * 1e6
                if (additional_active_power==0 and additional_reactive_power==0):
                    warnings.warn('Paramenter "AdditionalActivePower" and "AdditionalReactivePower" are equal to zero!\n Event "LoadStep" will be skipped')
                    return
                
                closed_resistance= np.abs(node.initial_single_voltage()**2) / additional_active_power
                                
                # TODO: add impedance
                #closed_impendace = ...
                
                # Füge switch mit Erdung hinzu
                switch = None
                gnd = None
                if domain == Domain.SP:
                    switch = dpsimpy.sp.ph1.Switch('Fault_' + node_name, self.loglevel)
                    gnd = dpsimpy.sp.SimNode.gnd
                elif domain == Domain.DP:
                    switch = dpsimpy.dp.ph1.Switch('Fault_' + node_name, self.loglevel)
                    gnd = dpsimpy.dp.SimNode.gnd
                elif domain == Domain.EMT:
                    switch = dpsimpy.emt.ph3.SeriesSwitch('Fault_' + node_name, self.loglevel)
                    gnd = dpsimpy.emt.SimNode.gnd 
                switch.set_parameters(open_resistance, closed_resistance)
                switch.open()
                self.system.add(switch)
                self.system.connect_component(switch, [gnd, node])

                # Event hinzufügen
                sw_event_1 = dpsimpy.event.SwitchEvent(event_start_time, switch, True)
                self.sim.add_event(sw_event_1)
                sw_event_2 = dpsimpy.event.SwitchEvent(event_end_time, switch, False)
                self.sim.add_event(sw_event_2)
                
            elif self.data["Events"]['EventType'] == "LineDisconnection":
                event_params = self.data["Events"]['EventParameters']
                
                # get event parameters
                
                # get event start time
                event_start_time = 0
                if "EventStartTime" in self.data["Events"].keys():
                    event_start_time = self.data["Events"]["EventStartTime"]
                else:
                    raise Exception('Paramenter "EventStartTime" is not in the json file!') 
                
                # get event end time
                #event_end_time = 0
                #if "EventEndTime" in self.data["Events"].keys():
                #    event_end_time = self.data["Events"]["EventEndTime"]  
                #else:
                #    raise Exception('Paramenter "EventEndTime" is not in the json file!')
                
                # get parameters
                open_resistance = 1e18
                if "SwitchOpenResistance" in event_params.keys():
                    open_resistance = event_params['SwitchOpenResistance']
                else:
                    warnings.warn('Paramenter "SwitchOpenResistance" is not in the json file!\n SwitchOpenResistance set to 1e18')
                    
                closed_resistance = 1e-6  
                if "SwitchClosedResistance" in event_params.keys():
                    closed_resistance = event_params['SwitchClosedResistance']
                else:
                    warnings.warn('Paramenter "SwitchClosedResistance" is not in the json file!\n SwitchClosedResistance set to 1e-6  ')
                    
                # search for line in dpsim topology
                line = None
                for comp in self.system.components:
                    if comp.name() == event_params["LineName"]:
                        line=comp
                        break
                if line==None:
                    raise Exception('Line name={} was not found in dpsim topology!'.format(event_params["LineName"]))
                
                # Add dummy node between node0 of line and node_dummy
                node_dummy = None
                if domain == Domain.SP:
                    node_dummy = dpsimpy.sp.ph1.SimNode('DummyNode_' + line.name(), dpsimpy.PhaseType.Single)
                elif domain == Domain.DP:
                    node_dummy = dpsimpy.dp.ph1.SimNode('DummyNode_' + line.name(), dpsimpy.PhaseType.Single)
                elif domain == Domain.EMT:
                    node_dummy = dpsimpy.emt.ph3.SimNode('DummyNode_' + line.name(), dpsimpy.PhaseType.ABC)
                self.system.add_node(node_dummy)
                
                # get nodes of line
                node0 = comp.get_terminal(0).node()
                node1 = comp.get_terminal(1).node()
                
                # Add switch between node0 and dummy_node
                switch = None
                if domain == Domain.SP:
                    switch = dpsimpy.sp.ph1.Switch('Switch_' + line.name(), self.loglevel)
                elif domain == Domain.DP:
                    switch = dpsimpy.dp.ph1.Switch('Switch_' + line.name(), self.loglevel)
                elif domain == Domain.EMT:
                    switch = dpsimpy.emt.ph3.SeriesSwitch('Switch_' + line.name(), self.loglevel)
                switch.set_parameters(open_resistance, closed_resistance)
                switch.open()
                
                # remove connection of line
                for key, comps_at_node_list in self.system.components_at_node.items():
                    if comp in comps_at_node_list:
                        comps_at_node_list_new = self.system.components_at_node
                        comps_at_node_list_new[key] = [comp_ for comp_ in comps_at_node_list_new[key] if comp_ != comp]
                        self.system.components_at_node = comps_at_node_list_new
                
                # reconnect line
                self.system.connect_component(comp, [node0, node_dummy])
                                        
                # initialize dummy node with voltage=voltage_node0
                node_dummy.set_initial_voltage(node0.initial_single_voltage())
                
                # connect switch
                self.system.add(switch)
                self.system.connect_component(switch, [node_dummy, node1])

                # Event hinzufügen           
                sw_event_1 = dpsimpy.event.SwitchEvent(event_start_time, switch, True)
                self.sim.add_event(sw_event_1)

            else:
                raise Exception('Please specify a valid event case. The possibilities are: "ShortCircuit" or "LineDisconnection"')
                
            """
            if self.data["Events"]['EventType'] == "LineDisconnection":
                event_params = self.data["Events"]['EventParameters']

                for comp in self.system.components:
                    if event_params['LineName'] == comp.name():
                        # comp ist die zu entfernende Freileitung
                        # Entferne Freileitung aus Komponentenliste    
                        self.system.components = [x for x in self.system.components if x != comp]

                        # Entferne Freileitung aus den Knoten-Listen
                        for key, value in self.system.components_at_node.items():
                            if comp in value:
                                obj_dict = system.components_at_node
                                obj_dict[key] = [c for c in obj_dict[key] if c != comp]
                                system.components_at_node = obj_dict

                    ### DPsim SP simulation
                    sim_parameters = data['SimulationParameters']
                    name = sim_parameters['SimulationName']
                    dpsimpy.Logger.set_log_dir("logs/" + name)

                    ### Simulation
                    sim = dpsimpy.Simulation(name, dpsimpy.LogLevel.debug)
                    sim.set_system(system)
                    sim.set_domain(dpsimpy.Domain.SP)

                    sim.do_init_from_nodes_and_terminals(True)
                    sim.set_direct_solver_implementation(dpsimpy.DirectLinearSolverImpl.SparseLU)
                    sim.do_system_matrix_recomputation(True)

                    # Initialisiere Spannungen an Knoten vom dynamischen System
                    self.system.init_with_powerflow(self.system_pf, domain=self.domain)

                    logger = dpsimpy.Logger(name)
                    sim.add_logger(logger)
                    for node_i in system.nodes:
                        logger.log_attribute(node_i.name + '.V', 'v', node_i)

                    sim.set_time_step(sim_parameters['TimeStep'])
                    sim.set_final_time(sim_parameters['EndTime'])

                    sim.run()

                    return system
        """
    
    def get_path_to_results(self):
        # This function return a list containing the relative path to the files with the results of the dynamic simulations
        dict_result_files = []
        for time_step in self.time_steps:
            path_dict = {}
            path_dict["TimeStep"] = time_step
            path_dict["Domain"] = []
            path_dict["Path"] = []
            for domain in self.domains:                
                path_dict["Domain"].append(domain.name)
                file_name = self.sim_name + "_" + domain.name + "_TS" + str(time_step)
                path_dict["Path"].append("logs/" + file_name + "/" + file_name + ".csv")
            
            dict_result_files.append(path_dict)    
        
        return dict_result_files