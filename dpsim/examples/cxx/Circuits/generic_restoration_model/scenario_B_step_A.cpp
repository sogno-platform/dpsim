/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/
#include <DPsim.h>
#include "../../Examples.h"

using namespace DPsim;
using namespace CPS;
using namespace CIM::Examples::Grids::generic_model_B_A;

ScenarioConfig generic_model_B_A;

//Switch to trigger fault at generator terminal
Real SwitchOpen = 1e12;
Real SwitchClosed = 1e-12;
//void scenario_B_step_A(String simName, Real timeStep, Real finalTime){
void scenario_B_step_A(String simName, Real timeStep, Real finalTime, Bool startFaultEvent, Bool endFaultEvent, Real startTimeFault, Real endTimeFault, Bool useVarResSwitch, Real cmdInertia_G1, Real cmdDamping_G1) {
	// ----- POWERFLOW FOR INITIALIZATION -----
	Real timeStepPF = finalTime;
	Real finalTimePF = finalTime+timeStepPF;
	String simNamePF = simName + "_PF";
	Logger::setLogDir("logs/" + simNamePF);

	// Components // EDITED ARMIN
	auto BUS_gas_PF = SimNode<Complex>::make("BUS_gas", PhaseType::Single);
	auto BUS_b_PF = SimNode<Complex>::make("BUS_b", PhaseType::Single);
	auto BUS_a_PF = SimNode<Complex>::make("BUS_a", PhaseType::Single);
	auto BUS_psha_PF = SimNode<Complex>::make("BUS_psha", PhaseType::Single);
	auto BUS_a_load_PF = SimNode<Complex>::make("BUS_a_load", PhaseType::Single);
	auto BUS_b_load_PF = SimNode<Complex>::make("BUS_b_load", PhaseType::Single);
	//auto BUSpsh = SimNode<Complex>::make("BUSpsh", PhaseType::Single);

	//Synchronous generator 1 // EDITED ARMIN
	auto GEN_gas_PF = SP::Ph1::SynchronGenerator::make("GEN_gas", Logger::Level::debug);
	// setPointVoltage is defined as the voltage at the transfomer primary side and should be transformed to network side
	GEN_gas_PF->setParameters(generic_model_B_A.nomPower_G1, generic_model_B_A.nomPhPhVoltRMS_G1, generic_model_B_A.initActivePower_G1, 10.5e3, PowerflowBusType::VD);
	GEN_gas_PF->setBaseVoltage(10.5e3);

	//auto extnetPF = SP::Ph1::NetworkInjection::make("Slack", Logger::Level::debug);
	//extnetPF->setParameters(generic_model_B_A.nomPhPhVoltRMS_G1/*scenario.systemNominalVoltage*/);
	//extnetPF->setBaseVoltage(generic_model_B_A.nomPhPhVoltRMS_G1/*scenario.systemNominalVoltage*/);
	//extnetPF->modifyPowerFlowBusType(PowerflowBusType::VD);

	auto transformer = std::make_shared<SP::Ph1::Transformer>("trafo_gas", Logger::Level::debug);
    transformer->setParameters(generic_model_B_A.nomPhPhVoltRMS_G1 /*nomVoltageEnd1*/, generic_model_B_A.Vnom /*nomVoltageEnd2*/, generic_model_B_A.nomPower_G1 /*ratedPower*/, (generic_model_B_A.nomPhPhVoltRMS_G1/generic_model_B_A.Vnom)*(1 + (2.5/100)*0)/*ratioAbs*/, 5.0*30*0 /*ratioPhase*/, /*2*1.9129660649*/ 1.82078864*2 /*resistance*/, /*2*0.163042774914*/ 0.15518646*2 /*inductance*/);
    //Real baseVolt = voltageNode1 >= voltageNode2 ? voltageNode1 : voltageNode2;
    transformer->setBaseVoltage(generic_model_B_A.Vnom);
	
	//use Shunt as Load for powerflow		
	//auto shunt_SR_acb_PF = SP::Ph1::Shunt::make("shunt_SR_acb", Logger::Level::debug);
	//shunt_SR_acb_PF->setParameters(generic_model_B_A.shuntConduntanceA /*conduntance*/, generic_model_B_A.shuntSusceptanceB /*susceptance*/);
	//shunt_SR_acb_PF->setBaseVoltage(generic_model_B_A.Vnom); // 2.0659e-07 - 2.0659e-05i
	//2.0659e-07 - 2.0659e-05i  // when we not divide and not mult.
	//1.3773e-08 - 1.3773e-06i  // when we *15
	//3.0989e-06 - 3.0989e-04i   // when we divide by 15
	//4.1318e-07 - 4.1318e-05i  // when we divide by 2
	//6.1977e-07 - 6.1977e-05i  // when we divide by 3
	//auto shunt_SR_bcb_PF = SP::Ph1::Shunt::make("shunt_SR_acb", Logger::Level::debug);
	//shunt_SR_bcb_PF->setParameters(generic_model_B_A.shuntConduntanceB, generic_model_B_A.shuntSusceptanceB);
	//shunt_SR_bcb_PF->setBaseVoltage(generic_model_B_A.Vnom);
	
	auto dummy_load_bus_psha_PF = SP::Ph1::Load::make("dummy_load_bus_psha", Logger::Level::debug);
	dummy_load_bus_psha_PF->setParameters(0,0, 220e3);


	auto load_bus_a_PF = SP::Ph1::Load::make("load_bus_a", Logger::Level::debug);
	load_bus_a_PF->setParameters(generic_model_B_A.activePower_L_bus_A, generic_model_B_A.reactivePower_L_bus_A, 220e3); // 1 step
	//load_bus_A_PF->setParameters(0.0228, 2.2802e6, 220e3); // 2 steps
	//load_bus_A_PF->setParameters(0.2e6, 15.3e6, 220e3);
	//load_bus_A_PF->setParameters(0.0, 0.0, 220e3);

	auto load_bus_b_PF = SP::Ph1::Load::make("load_bus_b", Logger::Level::debug);
	load_bus_b_PF->setParameters(generic_model_B_A.activePower_L_bus_B, generic_model_B_A.reactivePower_L_bus_A, 220e3);   // 1 step
	//load_bus_B_PF->setParameters(0.0228e6, 2.2786e6, 220e3);    // 2 steps
	//load_bus_B_PF->setParameters(0.2e6, 15.3e6, 220e3);
	//load_bus_B_PF->setParameters(0.0, 0.0, 220e3);

	

	//example
	//auto dummy_load_bus_psha_PF = SP::Ph1::Shunt::make("dummy_load_bus_psha_PF", Logger::Level::debug);
	//shunt_BUS_psha_PF->setParameters(generic_model_B_A.activePower_L / std::pow(generic_model_B_A.Vnom, 2), - generic_model_B_A.reactivePower_L / std::pow(generic_model_B_A.Vnom, 2));
	//shunt_BUS_psha_PF->setBaseVoltage(generic_model_B_A.Vnom);

	//Line1
	auto cable_PF = SP::Ph1::PiLine::make("cable", Logger::Level::debug);
	cable_PF->setParameters(generic_model_B_A.cableResistance /*R/km * km*/, generic_model_B_A.cableInductance /*L/km * km*/, generic_model_B_A.cableCapacitance /*Capacitance*/, generic_model_B_A.cableConductance);
	cable_PF->setBaseVoltage(generic_model_B_A.Vnom);
		
	auto line_3_PF = SP::Ph1::PiLine::make("line_3", Logger::Level::debug);
	line_3_PF->setParameters(generic_model_B_A.lineResistance3, generic_model_B_A.lineInductance3, generic_model_B_A.lineCapacitance3, generic_model_B_A.lineConductance3);
	line_3_PF->setBaseVoltage(generic_model_B_A.Vnom);


	auto line_a_load_PF = SP::Ph1::PiLine::make("line_a_load", Logger::Level::debug);
	line_a_load_PF->setParameters(1e-7, 1e-7, 0, 0);
	line_a_load_PF->setBaseVoltage(generic_model_B_A.Vnom);

	auto line_b_load_PF = SP::Ph1::PiLine::make("line_b_load", Logger::Level::debug);
	line_b_load_PF->setParameters(1e-7, 1e-7, 0, 0);
	line_b_load_PF->setBaseVoltage(generic_model_B_A.Vnom);


	// switch
	/*
	auto breaker_a_PF = SP::Ph1::Switch::make("breaker_a", Logger::Level::debug);
	breaker_a_PF->setParameters(1000, 1e-5, true); // true means that the switch is closed

	auto breaker_b_PF = SP::Ph1::Switch::make("breaker_b", Logger::Level::debug);
	breaker_b_PF->setParameters(1000, 1e-5, true);
	*/


	//auto line_gas_b_PF = SP::Ph1::PiLine::make("line_gas_b_PF", Logger::Level::debug);
	//line_gas_b_PF->setParameters(generic_model_B_A.lineResistance12, generic_model_B_A.lineInductance12, generic_model_B_A.lineCapacitance12, generic_model_B_A.lineConductance12);
	//line_gas_b_PF->setBaseVoltage(generic_model_B_A.Vnom);
	
	// Topology
	GEN_gas_PF->connect({ BUS_gas_PF });
	//extnetPF->connect({ BUS_gas_PF });
	transformer->connect({ BUS_gas_PF, BUS_b_PF});
	cable_PF->connect({ BUS_a_PF, BUS_b_PF });
	line_3_PF->connect({ BUS_a_PF, BUS_psha_PF});
	//shunt_SR_acb_PF->connect({ BUS_a_PF });
	//shunt_SR_bcb_PF->connect({ BUS_b_PF });
	load_bus_a_PF->connect({ BUS_a_load_PF });
	load_bus_b_PF->connect({ BUS_b_load_PF });
	//breaker_a_PF->connect({ BUS_a_PF, BUS_a_load_PF });
	//breaker_b_PF->connect({ BUS_b_PF, BUS_b_load_PF });
	line_a_load_PF->connect({ BUS_a_PF, BUS_a_load_PF});
	line_b_load_PF->connect({ BUS_b_PF, BUS_b_load_PF});
	dummy_load_bus_psha_PF->connect({ BUS_psha_PF });


	auto systemPF = SystemTopology(50, // das ist freq??
			SystemNodeList{BUS_a_PF, BUS_b_PF, BUS_gas_PF, BUS_psha_PF, BUS_a_load_PF, BUS_b_load_PF},
			SystemComponentList{GEN_gas_PF, line_a_load_PF, line_b_load_PF,/*breaker_a_PF, breaker_b_PF,*/ load_bus_a_PF, load_bus_b_PF, cable_PF, line_3_PF, dummy_load_bus_psha_PF, transformer});

	// Logging
	auto loggerPF = DataLogger::make(simNamePF);
	loggerPF->logAttribute("V_BUS_a_PF", BUS_a_PF->attribute("v"));
	loggerPF->logAttribute("V_BUS_b_PF", BUS_b_PF->attribute("v"));
	loggerPF->logAttribute("V_BUS_psha_PF", BUS_psha_PF->attribute("v"));
	loggerPF->logAttribute("V_BUS_gas_PF", BUS_gas_PF->attribute("v"));

	// Simulation
	Simulation simPF(simNamePF, Logger::Level::debug);
	simPF.setSystem(systemPF);
	simPF.setTimeStep(timeStepPF);
	simPF.setFinalTime(finalTimePF);
	simPF.setDomain(Domain::SP);
	simPF.setSolverType(Solver::Type::NRP);
	simPF.setSolverAndComponentBehaviour(Solver::Behaviour::Initialization);
	simPF.doInitFromNodesAndTerminals(true);
	simPF.addLogger(loggerPF);
	simPF.run();

	// ----- Dynamic simulation ------
	String simNameEMT = simName + "_EMT";
	Logger::setLogDir("logs/"+simNameEMT);

	// Nodes
	auto BUS_gas_EMT = SimNode<Real>::make("BUS_gas", PhaseType::ABC);
	auto BUS_a_EMT = SimNode<Real>::make("BUS_a", PhaseType::ABC);
	auto BUS_a_load_EMT = SimNode<Real>::make("BUS_a_load", PhaseType::ABC);
	auto BUS_b_EMT = SimNode<Real>::make("BUS_b", PhaseType::ABC);
	auto BUS_b_load_EMT = SimNode<Real>::make("BUS_b_load", PhaseType::ABC);
	auto BUS_psha_EMT = SimNode<Real>::make("BUS_psha", PhaseType::ABC);

	// Components
	//Synchronous generator 1
	auto GEN_gas_EMT = EMT::Ph3::SynchronGeneratorTrStab::make("GEN_gas", Logger::Level::debug);
	// Xpd is given in p.u of generator base at transfomer primary side and should be transformed to network side
	GEN_gas_EMT->setStandardParametersPU(generic_model_B_A.nomPower_G1, generic_model_B_A.nomPhPhVoltRMS_G1, generic_model_B_A.nomFreq_G1, generic_model_B_A.Xpd_G1*std::pow(generic_model_B_A.t1_ratio,2), cmdInertia_G1*generic_model_B_A.H_G1, generic_model_B_A.Rs_G1, cmdDamping_G1*generic_model_B_A.D_G1);
	// Get actual active and reactive power of generator's Terminal from Powerflow solution
	Complex initApparentPower_G1= GEN_gas_PF->getApparentPower();
	GEN_gas_EMT->setInitialValues(initApparentPower_G1, generic_model_B_A.initMechPower_G1);

	// Trafo
	auto trafo = EMT::Ph3::Transformer::make("trafo_gas", "trafo_gas", Logger::Level::debug, true);
	trafo->setParameters( generic_model_B_A.nomPhPhVoltRMS_G1, generic_model_B_A.Vnom, generic_model_B_A.nomPower_G1,
                     (generic_model_B_A.nomPhPhVoltRMS_G1/generic_model_B_A.Vnom)*(1 + (2.5/100)*0), 5.0*30*0, Math::singlePhaseParameterToThreePhase(1.82078864*2),
                     Math::singlePhaseParameterToThreePhase(0.15518646*2));

	//transformer->setParameters(generic_model_B_A.nomPhPhVoltRMS_G1 /*nomVoltageEnd1*/, generic_model_B_A.Vnom /*nomVoltageEnd2*/, generic_model_B_A.nomPower_G1 /*ratedPower*/, (generic_model_B_A.nomPhPhVoltRMS_G1/generic_model_B_A.Vnom)*(1 + (2.5/100)*0)/*ratioAbs*/, 5.0*30*0 /*ratioPhase*/, /*2*1.9129660649*/ 1.82078864*2 /*resistance*/, /*2*0.163042774914*/ 0.15518646*2 /*inductance*/);
    //Real baseVolt = voltageNode1 >= voltageNode2 ? voltageNode1 : voltageNode2;
    //transformer->setBaseVoltage(generic_model_B_A.Vnom);

	// Cable
	auto cable_EMT = EMT::Ph3::PiLine::make("cable", Logger::Level::debug);
	cable_EMT->setParameters(Math::singlePhaseParameterToThreePhase(generic_model_B_A.cableResistance), 
	                      Math::singlePhaseParameterToThreePhase(generic_model_B_A.cableInductance), 
					      Math::singlePhaseParameterToThreePhase(generic_model_B_A.cableCapacitance),
						  Math::singlePhaseParameterToThreePhase(generic_model_B_A.cableConductance));

	// Line3
	auto line3_EMT = EMT::Ph3::PiLine::make("line_3", Logger::Level::debug);
	line3_EMT->setParameters(Math::singlePhaseParameterToThreePhase(generic_model_B_A.lineResistance3), 
	                      Math::singlePhaseParameterToThreePhase(generic_model_B_A.lineInductance3), 
					      Math::singlePhaseParameterToThreePhase(generic_model_B_A.lineCapacitance3),
						  Math::singlePhaseParameterToThreePhase(generic_model_B_A.lineConductance3));


	//Load
	//auto shunt_SR_acb_EMT = EMT::Ph3::RXLoad::make("Load", Logger::Level::debug);  // here the active power of the shunt needs to be written
	//shunt_SR_acb_EMT->setParameters(CPS::Math::singlePhasePowerToThreePhase(generic_model_B_A.activePower_L), CPS::Math::singlePhasePowerToThreePhase(generic_model_B_A.reactivePower_L), generic_model_B_A.Vnom);

	//auto shunt_SR_bcb_EMT = EMT::Ph3::RXLoad::make("Load", Logger::Level::debug);  // here the active power of the shunt needs to be written
	//shunt_SR_bcb_EMT->setParameters(CPS::Math::singlePhasePowerToThreePhase(generic_model_B_A.activePower_L), CPS::Math::singlePhasePowerToThreePhase(generic_model_B_A.reactivePower_L), generic_model_B_A.Vnom);

	auto load_bus_a_EMT = EMT::Ph3::RXLoad::make("load_bus_a", Logger::Level::debug);
	load_bus_a_EMT->setParameters(CPS::Math::singlePhasePowerToThreePhase(generic_model_B_A.activePower_L_bus_A), CPS::Math::singlePhasePowerToThreePhase(generic_model_B_A.reactivePower_L_bus_A), generic_model_B_A.Vnom);

	auto load_bus_b_EMT = EMT::Ph3::RXLoad::make("load_bus_b", Logger::Level::debug);
	load_bus_b_EMT->setParameters(CPS::Math::singlePhasePowerToThreePhase(generic_model_B_A.activePower_L_bus_B), CPS::Math::singlePhasePowerToThreePhase(generic_model_B_A.reactivePower_L_bus_B), generic_model_B_A.Vnom);


	// //Switch

	auto breaker_a_EMT = CPS::EMT::Ph3::Switch::make("breaker_a_EMT", Logger::Level::debug);
	breaker_a_EMT->setParameters(Math::singlePhaseParameterToThreePhase(SwitchOpen), 
							Math::singlePhaseParameterToThreePhase(SwitchClosed));
	breaker_a_EMT->closeSwitch();

	auto breaker_b_EMT = CPS::EMT::Ph3::Switch::make("breaker_b_EMT", Logger::Level::debug);
	breaker_b_EMT->setParameters(Math::singlePhaseParameterToThreePhase(SwitchOpen), 
							Math::singlePhaseParameterToThreePhase(SwitchClosed));
	breaker_b_EMT->closeSwitch();

	

	// Topology
	GEN_gas_EMT->connect({ BUS_gas_EMT });
	load_bus_a_EMT->connect({ BUS_a_load_EMT });
	load_bus_b_EMT->connect({ BUS_b_load_EMT });
	cable_EMT->connect({ BUS_a_EMT, BUS_b_EMT });
	line3_EMT->connect({ BUS_a_EMT, BUS_psha_EMT });
	trafo->connect({BUS_gas_EMT, BUS_b_EMT});
	// faultEMT->connect({EMT::SimNode::GND , n1EMT }); //terminal of generator 1
	breaker_a_EMT->connect({BUS_a_load_EMT, BUS_a_EMT }); 
	breaker_b_EMT->connect({BUS_b_load_EMT, BUS_b_EMT }); 
	// faultEMT->connect({EMT::SimNode::GND , n3EMT }); //Load bus
	auto systemEMT = SystemTopology(50,
			SystemNodeList{BUS_gas_EMT, BUS_a_EMT, BUS_b_EMT, BUS_a_load_EMT, BUS_b_load_EMT},
			SystemComponentList{GEN_gas_EMT, cable_EMT, line3_EMT, trafo, breaker_a_EMT, breaker_b_EMT, BUS_a_load_EMT, load_bus_a_EMT, load_bus_b_EMT});

	// Initialization of dynamic topology
	systemEMT.initWithPowerflow(systemPF, Domain::EMT);

	// Logging
	auto loggerEMT = DataLogger::make(simNameEMT);
	loggerEMT->logAttribute("BUS_gas_EMT_v", BUS_gas_EMT->attribute("v"));
	loggerEMT->logAttribute("BUS_a_EMT_v", BUS_a_EMT->attribute("v"));
	loggerEMT->logAttribute("BUS_b_EMT_b", BUS_b_EMT->attribute("v"));
	loggerEMT->logAttribute("cable_EMT_v", cable_EMT->attribute("v_intf"));
	loggerEMT->logAttribute("cable_EMT_i", cable_EMT->attribute("i_intf"));
	loggerEMT->logAttribute("line3_EMT_v", line3_EMT->attribute("v_intf"));
	loggerEMT->logAttribute("line3_EMT_i", line3_EMT->attribute("i_intf"));
	loggerEMT->logAttribute("GEN_gas_EMT_ep_mag", GEN_gas_EMT->attribute("Ep_mag"));
	loggerEMT->logAttribute("GEN_gas_EMT_v", GEN_gas_EMT->attribute("v_intf"));
	loggerEMT->logAttribute("GEN_gas_EMT_i", GEN_gas_EMT->attribute("i_intf"));
	loggerEMT->logAttribute("GEN_gas_EMT_wr", GEN_gas_EMT->attribute("w_r"));
	loggerEMT->logAttribute("GEN_gas_EMT_delta", GEN_gas_EMT->attribute("delta_r"));	
	loggerEMT->logAttribute("load_bus_a_EMT_v", load_bus_a_EMT->attribute("v_intf"));
	loggerEMT->logAttribute("load_bus_a_EMT_i", load_bus_a_EMT->attribute("i_intf"));
	loggerEMT->logAttribute("load_bus_b_EMT_v", load_bus_b_EMT->attribute("v_intf"));
	loggerEMT->logAttribute("load_bus_b_EMT_i", load_bus_b_EMT->attribute("i_intf"));
	loggerEMT->logAttribute("P_mech1", GEN_gas_EMT->attribute("P_mech"));
	loggerEMT->logAttribute("P_elec1", GEN_gas_EMT->attribute("P_elec"));

	Simulation simEMT(simNameEMT, Logger::Level::debug);
	simEMT.setSystem(systemEMT);
	simEMT.setTimeStep(timeStep);
	simEMT.setFinalTime(finalTime);
	simEMT.setDomain(Domain::EMT);
	simEMT.addLogger(loggerEMT);
	simEMT.setSolverType(Solver::Type::MNA);
	simEMT.doInitFromNodesAndTerminals(true);
	//simEMT.setDirectLinearSolverImplementation(DPsim::DirectLinearSolverImpl::SparseLU);
	
	if (useVarResSwitch == true) {
		simEMT.doSystemMatrixRecomputation(true);
	}

		// Events
	if (startFaultEvent){
		auto sw1 = SwitchEvent3Ph::make(startTimeFault, breaker_a_EMT, false);
		auto sw2 = SwitchEvent3Ph::make(startTimeFault, breaker_b_EMT, false);
		simEMT.addEvent(sw1);
		simEMT.addEvent(sw2);
	}
/*
	if(endFaultEvent){

		auto sw3 = SwitchEvent3Ph::make(endTimeFault, breaker_a_EMT, true);
		auto sw4 = SwitchEvent3Ph::make(endTimeFault, breaker_b_EMT, true);
		simEMT.addEvent(sw3);
		simEMT.addEvent(sw4);
	
	}
*/

	simEMT.run();

}

int main(int argc, char* argv[]) {	
		
/*
	//Simultion parameters
	String simName="scenario_B_step_A";
	Real finalTime = 30;
	Real timeStep = 0.001;
	
	
	scenario_B_step_A(simName, timeStep, finalTime);
*/

//Simultion parameters
	String simName="scenario_B_step_A";
	Real finalTime = 1;
	Real timeStep = 0.001;
	Bool startFaultEvent=true;
	Bool endFaultEvent=true;
	Bool useVarResSwitch=false;
	Real startTimeFault=0.5;
	Real endTimeFault=100;
	Real cmdInertia_G1= 1.0;
	Real cmdDamping_G1=1.0;


	CommandLineArgs args(argc, argv);
	if (argc > 1) {
		timeStep = args.timeStep;
		finalTime = args.duration;
		if (args.name != "dpsim")
			simName = args.name;
		if (args.options.find("SCALEINERTIA_G1") != args.options.end())
			cmdInertia_G1 = args.getOptionReal("SCALEINERTIA_G1");
		if (args.options.find("SCALEDAMPING_G1") != args.options.end())
			cmdDamping_G1 = args.getOptionReal("SCALEDAMPING_G1");
		if (args.options.find("STARTTIMEFAULT") != args.options.end())
			startTimeFault = args.getOptionReal("STARTTIMEFAULT");
		if (args.options.find("ENDTIMEFAULT") != args.options.end())
			endTimeFault = args.getOptionReal("ENDTIMEFAULT");
		// if (args.options.find("USEVARRESSWITCH") != args.options.end())
		// 	useVarResSwitch = args.options["USEVARRESSWITCH"];	
		// if (args.options.find("FAULTRESISTANCE") != args.options.end())
		// 	SwitchClosed = args.options["FAULTRESISTANCE"];	
	}
	
	scenario_B_step_A(simName, timeStep, finalTime, startFaultEvent, endFaultEvent, startTimeFault, endTimeFault, useVarResSwitch, cmdInertia_G1, cmdDamping_G1);

}