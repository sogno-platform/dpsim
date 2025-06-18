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
using namespace CIM::Examples::Grids::generic_model_C_A;

ScenarioConfig generic_model_C_A;

//Switch to trigger fault at generator terminal
Real SwitchOpen = 1e12;
Real SwitchClosed = 1e-12;
//void scenario_C_step_A(String simName, Real timeStep, Real finalTime){
void scenario_C_step_A(String simName, Real timeStep, Real finalTime, Bool startFaultEvent, Bool endFaultEvent, Real startTimeFault, Real endTimeFault, Bool useVarResSwitch, Real cmdInertia_G1, Real cmdDamping_G1) {
	// ----- POWERFLOW FOR INITIALIZATION -----
	Real timeStepPF = finalTime;
	Real finalTimePF = finalTime+timeStepPF;
	String simNamePF = simName + "_PF";
	Logger::setLogDir("logs/" + simNamePF);

	// Components // EDITED ARMIN
	auto BUS_gas_PF = SimNode<Complex>::make("BUS_gas", PhaseType::Single);
	auto BUS_b_PF = SimNode<Complex>::make("BUS_b", PhaseType::Single);
	

	//Synchronous generator 1 // EDITED ARMIN
	auto GEN_gas_PF = SP::Ph1::SynchronGenerator::make("GEN_gas", Logger::Level::debug);
	// setPointVoltage is defined as the voltage at the transfomer primary side and should be transformed to network side
	GEN_gas_PF->setParameters(generic_model_C_A.nomPower_G1, generic_model_C_A.nomPhPhVoltRMS_G1, generic_model_C_A.initActivePower_G1, 10.5e3, PowerflowBusType::VD);
	GEN_gas_PF->setBaseVoltage(10.5e3);

	auto extnetPF = SP::Ph1::NetworkInjection::make("Slack", Logger::Level::debug);
	extnetPF->setParameters(generic_model_C_A.nomPhPhVoltRMS_G1/*scenario.systemNominalVoltage*/);
	extnetPF->setBaseVoltage(generic_model_C_A.nomPhPhVoltRMS_G1/*scenario.systemNominalVoltage*/);
	extnetPF->modifyPowerFlowBusType(PowerflowBusType::VD);

	auto transformer = std::make_shared<SP::Ph1::Transformer>("trafo_gas", Logger::Level::debug);
    transformer->setParameters(generic_model_C_A.nomPhPhVoltRMS_G1  /*nomVoltageEnd1*/, generic_model_C_A.Vnom /*nomVoltageEnd2*/, generic_model_C_A.nomPower_G1 /*ratedPower*/, 
				(generic_model_C_A.nomPhPhVoltRMS_G1/generic_model_C_A.Vnom)/*ratioAbs*/, 5.0*30*0 /*ratioPhase*/, /*2*1.9129660649*/ /*3.64157728*/1.82078864*2 /*resistance*/, 
	/*2*0.163042774914*/ 0.310372659/*0.15518646*2*/ /*inductance*/);
    //Real baseVolt = voltageNode1 >= voltageNode2 ? voltageNode1 : voltageNode2;
    transformer->setBaseVoltage(generic_model_C_A.Vnom);
	

	auto dummy_load_bus_b = SP::Ph1::Load::make("dummy_load_bus_b", Logger::Level::debug);
	dummy_load_bus_b->setParameters(15e6, 5e6, 220e3);

	// shunt
	auto shunt_SR_bcb_PF = SP::Ph1::Shunt::make("shunt_SR_acb", Logger::Level::debug);
	shunt_SR_bcb_PF->setParameters(generic_model_C_A.shuntConduntanceB /*conduntance*/, generic_model_C_A.shuntSusceptanceB /*susceptance*/);
	shunt_SR_bcb_PF->setBaseVoltage(generic_model_C_A.Vnom); // 2.0659e-07 - 2.0659e-05i


	// Topology
	GEN_gas_PF->connect({ BUS_gas_PF });
	//extnetPF->connect({ BUS_gas_PF });
	transformer->connect({ BUS_gas_PF, BUS_b_PF});
	//dummy_load_bus_b->connect({ BUS_b_PF });
	dummy_load_bus_b->connect({ BUS_b_PF });


	auto systemPF = SystemTopology(50, // das ist freq??
			SystemNodeList{BUS_gas_PF, BUS_b_PF},
			SystemComponentList{GEN_gas_PF, transformer, dummy_load_bus_b});

	// Logging
	auto loggerPF = DataLogger::make(simNamePF);
	loggerPF->logAttribute("V_BUS_b_PF", BUS_b_PF->attribute("v"));
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
	auto BUS_b_EMT = SimNode<Real>::make("BUS_b", PhaseType::ABC);
	//auto BUS_l2_EMT = SimNode<Real>::make("BUS_l2", PhaseType::ABC);
	//auto BUS_dummy_EMT = SimNode<Real>::make("dummy_bus", PhaseType::ABC);
	// Components

	//Synchronous generator 1
	auto GEN_gas_EMT = EMT::Ph3::SynchronGeneratorTrStab::make("GEN_gas", Logger::Level::debug);
	// Xpd is given in p.u of generator base at transfomer primary side and should be transformed to network side
	GEN_gas_EMT->setStandardParametersPU(generic_model_C_A.nomPower_G1, generic_model_C_A.nomPhPhVoltRMS_G1, generic_model_C_A.nomFreq_G1, 
				 generic_model_C_A.Xpd_G1*std::pow(generic_model_C_A.t1_ratio,2), cmdInertia_G1*generic_model_C_A.H_G1, generic_model_C_A.Rs_G1, cmdDamping_G1*generic_model_C_A.D_G1);
	// Get actual active and reactive power of generator's Terminal from Powerflow solution
	Complex initApparentPower_G1= GEN_gas_PF->getApparentPower();
	GEN_gas_EMT->setInitialValues(initApparentPower_G1, generic_model_C_A.initMechPower_G1);

	// Trafo
	auto trafo = EMT::Ph3::Transformer::make("trafo_gas", "trafo_gas", Logger::Level::debug, true);
	trafo->setParameters( generic_model_C_A.nomPhPhVoltRMS_G1, generic_model_C_A.Vnom, generic_model_C_A.nomPower_G1,
                     (generic_model_C_A.nomPhPhVoltRMS_G1/generic_model_C_A.Vnom)*(1 + (2.5/100)*0), 5.0*30*0, Math::singlePhaseParameterToThreePhase(3.64157728),
                     Math::singlePhaseParameterToThreePhase(0.15518646*2));

	//auto trafo_2 = EMT::Ph3::Transformer::make("trafo_load", "trafo_load", Logger::Level::debug, true);
	//trafo_2->setParameters(220e3 /*high voltage side*/, 10e3 /*low voltage side*/, 50e6,
    //                 (220e3/10e3), 5.0*30*0, Math::singlePhaseParameterToThreePhase(3.64157728),
    //                 Math::singlePhaseParameterToThreePhase(0.15518646*2));

	//transformer->setParameters(generic_model_C_A.nomPhPhVoltRMS_G1 /*nomVoltageEnd1*/, generic_model_C_A.Vnom /*nomVoltageEnd2*/, generic_model_C_A.nomPower_G1 /*ratedPower*/, (generic_model_C_A.nomPhPhVoltRMS_G1/generic_model_C_A.Vnom)*(1 + (2.5/100)*0)/*ratioAbs*/, 5.0*30*0 /*ratioPhase*/, /*2*1.9129660649*/ 1.82078864*2 /*resistance*/, /*2*0.163042774914*/ 0.15518646*2 /*inductance*/);
    //Real baseVolt = voltageNode1 >= voltageNode2 ? voltageNode1 : voltageNode2;
    //transformer->setBaseVoltage(generic_model_C_A.Vnom);
/*
	auto breaker_b_EMT = CPS::EMT::Ph3::Switch::make("breaker_b_EMT", Logger::Level::debug);
	breaker_b_EMT->setParameters(Math::singlePhaseParameterToThreePhase(SwitchOpen), 
							Math::singlePhaseParameterToThreePhase(SwitchClosed));
	breaker_b_EMT->openSwitch();
*/

	auto dummy_load_bus_b_EMT = EMT::Ph3::RXLoad::make("dummy_load_bus_b", Logger::Level::debug);
	dummy_load_bus_b_EMT->setParameters(CPS::Math::singlePhasePowerToThreePhase(15e6), 
	CPS::Math::singlePhasePowerToThreePhase(5e6), generic_model_C_A.Vnom);

	//auto load_bus_l2_EMT = EMT::Ph3::RXLoad::make("load_bus_l2", Logger::Level::debug);
	//load_bus_l2_EMT->setParameters(CPS::Math::singlePhasePowerToThreePhase(5.05631), CPS::Math::singlePhasePowerToThreePhase(3.98532), 10e3);

	

	// Topology
	GEN_gas_EMT->connect({ BUS_gas_EMT });
	trafo->connect({BUS_gas_EMT, BUS_b_EMT});
	dummy_load_bus_b_EMT->connect({ BUS_b_EMT });
	//breaker_b_EMT->connect({BUS_b_EMT, BUS_dummy_EMT});
	//trafo_2->connect({BUS_dummy_EMT, BUS_l2_EMT});
	//load_bus_l2_EMT->connect({ BUS_l2_EMT });
	

	auto systemEMT = SystemTopology(50,
			SystemNodeList{BUS_gas_EMT, BUS_b_EMT/*, BUS_l2_EMT*/},
			SystemComponentList{GEN_gas_EMT, trafo, dummy_load_bus_b_EMT/*, trafo_2, breaker_b_EMT, load_bus_l2_EMT*/});

	// Initialization of dynamic topology
	systemEMT.initWithPowerflow(systemPF, Domain::EMT);

	// Logging
	auto loggerEMT = DataLogger::make(simNameEMT);
	loggerEMT->logAttribute("BUS_gas_EMT_v", BUS_gas_EMT->attribute("v"));
	loggerEMT->logAttribute("BUS_b_EMT_v", BUS_b_EMT->attribute("v"));
	loggerEMT->logAttribute("GEN_gas_EMT_ep_mag", GEN_gas_EMT->attribute("Ep_mag"));
	loggerEMT->logAttribute("GEN_gas_EMT_v", GEN_gas_EMT->attribute("v_intf"));
	loggerEMT->logAttribute("GEN_gas_EMT_i", GEN_gas_EMT->attribute("i_intf"));
	loggerEMT->logAttribute("GEN_gas_EMT_wr", GEN_gas_EMT->attribute("w_r"));
	loggerEMT->logAttribute("GEN_gas_EMT_delta", GEN_gas_EMT->attribute("delta_r"));	
	loggerEMT->logAttribute("P_mech1", GEN_gas_EMT->attribute("P_mech"));
	loggerEMT->logAttribute("P_elec1", GEN_gas_EMT->attribute("P_elec"));
	

	/*  staro
	Simulation simEMT(simNameEMT, Logger::Level::debug);
	simEMT.setSystem(systemEMT);
	simEMT.setTimeStep(timeStep);
	simEMT.setFinalTime(finalTime);
	simEMT.setDomain(Domain::EMT);
	simEMT.addLogger(loggerEMT);
	simEMT.setSolverType(Solver::Type::MNA);
	simEMT.doInitFromNodesAndTerminals(true);
	//simEMT.setDirectLinearSolverImplementation(DPsim::DirectLinearSolverImpl::SparseLU);
	*/

	Simulation simEMT(simNameEMT, Logger::Level::debug);
  	simEMT.doInitFromNodesAndTerminals(true);
  	simEMT.setSystem(systemEMT);
  	simEMT.setTimeStep(timeStep);
  	simEMT.setFinalTime(finalTime);
  	simEMT.setDomain(Domain::EMT);
  	simEMT.addLogger(loggerEMT);

	if (useVarResSwitch == true) {
		simEMT.doSystemMatrixRecomputation(true);
	}
/*
	if (startFaultEvent){
		auto sw1 = SwitchEvent3Ph::make(startTimeFault, breaker_b_EMT, true);
		simEMT.addEvent(sw1);
	}*/


	simEMT.run();

}

int main(int argc, char* argv[]) {	
		
/*
	//Simultion parameters
	String simName="scenario_C_step_A";
	Real finalTime = 30;
	Real timeStep = 0.001;
	
	scenario_C_step_A(simName, timeStep, finalTime);
*/

//Simultion parameters
	String simName="scenario_C_step_A";
	Real finalTime = 0.7;
	Real timeStep = 0.001;
	Bool startFaultEvent=true;
	Bool endFaultEvent=true;
	Bool useVarResSwitch=false;
	Real startTimeFault=5.5;
	Real endTimeFault=100;
	Real cmdInertia_G1= 1.0;
	Real cmdDamping_G1= 1.0;


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
	
	scenario_C_step_A(simName, timeStep, finalTime, startFaultEvent, endFaultEvent, startTimeFault, endTimeFault, useVarResSwitch, cmdInertia_G1, cmdDamping_G1);

}