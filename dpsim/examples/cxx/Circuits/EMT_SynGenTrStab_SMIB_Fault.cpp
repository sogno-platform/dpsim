/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <DPsim.h>
#include "../Examples.h"

using namespace DPsim;
using namespace CPS;
using namespace CIM::Examples::Grids::SMIB;

ScenarioConfig smib;

//Switch to trigger fault at generator terminal
Real SwitchOpen = 1e6;
Real SwitchClosed = 0.1;

void EMT_1ph_SynGenTrStab_Fault(String simName, Real timeStep, Real finalTime, bool startFaultEvent, bool endFaultEvent, Real startTimeFault, Real endTimeFault, Real cmdInertia, Real cmdDamping) {
	// ----- POWERFLOW FOR INITIALIZATION -----
	Real timeStepPF = finalTime;
	Real finalTimePF = finalTime+timeStepPF;
	String simNamePF = simName + "_PF";
	Logger::setLogDir("logs/" + simNamePF);

	// Components
	auto n1PF = SimNode<Complex>::make("n1", PhaseType::Single);
	auto n2PF = SimNode<Complex>::make("n2", PhaseType::Single);

	//Synchronous generator ideal model
	auto genPF = SP::Ph1::SynchronGenerator::make("Generator", Logger::Level::debug);
	// setPointVoltage is defined as the voltage at the transfomer primary side and should be transformed to network side
	genPF->setParameters(smib.nomPower, smib.nomPhPhVoltRMS, smib.initActivePower, smib.setPointVoltage*smib.t_ratio, PowerflowBusType::PV);
	genPF->setBaseVoltage(smib.Vnom);
	genPF->modifyPowerFlowBusType(PowerflowBusType::PV);

	//Grid bus as Slack
	auto extnetPF = SP::Ph1::NetworkInjection::make("Slack", Logger::Level::debug);
	extnetPF->setParameters(smib.Vnom);
	extnetPF->setBaseVoltage(smib.Vnom);
	extnetPF->modifyPowerFlowBusType(PowerflowBusType::VD);

	//Line
	auto linePF = SP::Ph1::PiLine::make("PiLine", Logger::Level::debug);
	linePF->setParameters(smib.lineResistance, smib.lineInductance, smib.lineCapacitance, smib.lineConductance);
	linePF->setBaseVoltage(smib.Vnom);

	//Switch
	auto faultPF = CPS::SP::Ph1::Switch::make("Br_fault", Logger::Level::debug);
	faultPF->setParameters(SwitchOpen, SwitchClosed);
	faultPF->open();

	// Topology
	genPF->connect({ n1PF });
	faultPF->connect({SP::SimNode::GND , n1PF });
	linePF->connect({ n1PF, n2PF });
	extnetPF->connect({ n2PF });
	auto systemPF = SystemTopology(60,
			SystemNodeList{n1PF, n2PF},
			SystemComponentList{genPF, linePF, extnetPF, faultPF});

	// Logging
	auto loggerPF = DataLogger::make(simNamePF);
	loggerPF->logAttribute("v1", n1PF->attribute("v"));
	loggerPF->logAttribute("v2", n2PF->attribute("v"));

	// Simulation
	Simulation simPF(simNamePF, Logger::Level::debug);
	simPF.setSystem(systemPF);
	simPF.setTimeStep(timeStepPF);
	simPF.setFinalTime(finalTimePF);
	simPF.setDomain(Domain::SP);
	simPF.setSolverType(Solver::Type::NRP);
	simPF.setSolverAndComponentBehaviour(Solver::Behaviour::Initialization);
	simPF.doInitFromNodesAndTerminals(false);
	simPF.addLogger(loggerPF);
	simPF.run();

	// ----- Dynamic simulation ------
	String simNameEMT = simName + "_EMT";
	Logger::setLogDir("logs/"+simNameEMT);

	// Nodes
	auto n1EMT = SimNode<Real>::make("n1", PhaseType::ABC);
	auto n2EMT = SimNode<Real>::make("n2", PhaseType::ABC);

	// Components
	auto genEMT = EMT::Ph3::SynchronGeneratorTrStab::make("SynGen", Logger::Level::debug);
	// Xpd is given in p.u of generator base at transfomer primary side and should be transformed to network side
	genEMT->setStandardParametersPU(smib.nomPower, smib.nomPhPhVoltRMS, smib.nomFreq, smib.Xpd*std::pow(smib.t_ratio,2), cmdInertia*smib.H, smib.Rs, cmdDamping*smib.D );
	// Get actual active and reactive power of generator's Terminal from Powerflow solution
	Complex initApparentPower= genPF->getApparentPower();
	genEMT->setInitialValues(initApparentPower, smib.initMechPower);

	//Grid bus as Slack
	auto extnetEMT = EMT::Ph3::NetworkInjection::make("Slack", Logger::Level::debug);
	extnetEMT->setParameters(CPS::Math::singlePhaseVariableToThreePhase(smib.Vnom), 60); //frequency must be set to 60 otherwise the subvoltage source will set it to 50 per default

	// Line
	auto lineEMT = EMT::Ph3::PiLine::make("PiLine", Logger::Level::debug);
	lineEMT->setParameters(Math::singlePhaseParameterToThreePhase(smib.lineResistance),
	                      Math::singlePhaseParameterToThreePhase(smib.lineInductance),
					      Math::singlePhaseParameterToThreePhase(smib.lineCapacitance),
						  Math::singlePhaseParameterToThreePhase(smib.lineConductance));

	// //Switch
	// auto faultEMT = CPS::EMT::Ph3::Switch::make("Br_fault", Logger::Level::debug);
	// faultEMT->setParameters(Math::singlePhaseParameterToThreePhase(SwitchOpen),
	// 					 	   Math::singlePhaseParameterToThreePhase(SwitchClosed));
	// faultEMT->openSwitch();

	// Variable resistance Switch
	auto faultEMT = EMT::Ph3::varResSwitch::make("Br_fault", Logger::Level::debug);
	faultEMT->setParameters(Math::singlePhaseParameterToThreePhase(SwitchOpen),
						    Math::singlePhaseParameterToThreePhase(SwitchClosed));
	faultEMT->setInitParameters(timeStep);
	faultEMT->openSwitch();

	// Topology
	genEMT->connect({ n1EMT });
	faultEMT->connect({EMT::SimNode::GND , n1EMT });
	lineEMT->connect({ n1EMT, n2EMT });
	extnetEMT->connect({ n2EMT });
	auto systemEMT = SystemTopology(60,
			SystemNodeList{n1EMT, n2EMT},
			SystemComponentList{genEMT, lineEMT, extnetEMT, faultEMT});

	// Initialization of dynamic topology
	systemEMT.initWithPowerflow(systemPF);


	// Logging
	auto loggerEMT = DataLogger::make(simNameEMT);
	loggerEMT->logAttribute("v1", n1EMT->attribute("v"));
	loggerEMT->logAttribute("v2", n2EMT->attribute("v"));
	//gen
	loggerEMT->logAttribute("Ep", genEMT->attribute("Ep"));
	loggerEMT->logAttribute("v_gen", genEMT->attribute("v_intf"));
	loggerEMT->logAttribute("i_gen", genEMT->attribute("i_intf"));
	loggerEMT->logAttribute("wr_gen", genEMT->attribute("w_r"));
	loggerEMT->logAttribute("delta_r_gen", genEMT->attribute("delta_r"));
	loggerEMT->logAttribute("P_elec", genEMT->attribute("P_elec"));
	loggerEMT->logAttribute("P_mech", genEMT->attribute("P_mech"));
	//Switch
	loggerEMT->logAttribute("i_fault", faultEMT->attribute("i_intf"));
	//line
	loggerEMT->logAttribute("v_line", lineEMT->attribute("v_intf"));
	loggerEMT->logAttribute("i_line", lineEMT->attribute("i_intf"));
	//slack
	loggerEMT->logAttribute("v_slack", extnetEMT->attribute("v_intf"));
	loggerEMT->logAttribute("i_slack", extnetEMT->attribute("i_intf"));



	Simulation simEMT(simNameEMT, Logger::Level::debug);
	simEMT.setSystem(systemEMT);
	simEMT.setTimeStep(timeStep);
	simEMT.setFinalTime(finalTime);
	simEMT.setDomain(Domain::EMT);
	simEMT.addLogger(loggerEMT);
	simEMT.doSystemMatrixRecomputation(true); //for varres switch
	simEMT.setDirectLinearSolverImplementation(DPsim::DirectLinearSolverImpl::SparseLU);

	// Events
	if (startFaultEvent){
		auto sw1 = SwitchEvent3Ph::make(startTimeFault, faultEMT, true);

		simEMT.addEvent(sw1);
	}

	if(endFaultEvent){

		auto sw2 = SwitchEvent3Ph::make(endTimeFault, faultEMT, false);
		simEMT.addEvent(sw2);

	}

	simEMT.run();
}

int main(int argc, char* argv[]) {


	//Simultion parameters
	String simName="EMT_SynGenTrStab_SMIB_Fault";
	Real finalTime = 30;
	Real timeStep = 0.001;
	Bool startFaultEvent=true;
	Bool endFaultEvent=true;
	Real startTimeFault=10;
	Real endTimeFault=10.2;
	Real cmdInertia= 1.0;
	Real cmdDamping=1.0;

	CommandLineArgs args(argc, argv);
	if (argc > 1) {
		timeStep = args.timeStep;
		finalTime = args.duration;
		if (args.name != "dpsim")
			simName = args.name;
		if (args.options.find("SCALEINERTIA") != args.options.end())
			cmdInertia = args.getOptionReal("SCALEINERTIA");
		if (args.options.find("SCALEDAMPING") != args.options.end())
			cmdDamping = args.getOptionReal("SCALEDAMPING");
		if (args.options.find("STARTTIMEFAULT") != args.options.end())
			startTimeFault = args.getOptionReal("STARTTIMEFAULT");
		if (args.options.find("ENDTIMEFAULT") != args.options.end())
			endTimeFault = args.getOptionReal("ENDTIMEFAULT");
	}

	EMT_1ph_SynGenTrStab_Fault(simName, timeStep, finalTime, startFaultEvent, endFaultEvent, startTimeFault, endTimeFault, cmdInertia, cmdDamping);
}
