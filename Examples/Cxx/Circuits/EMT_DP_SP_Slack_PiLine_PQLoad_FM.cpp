/* Copyright 2017-2020 Institute for Automation of Complex Power Systems,
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
	
String simName = "EMT_DP_SP_Slack_PiLine_PQLoad_FM";

// Component parameters
Real Vnom = 20e3;
Real pLoadNom = 100e3;
Real qLoadNom = 50e3;
Real lineResistance = 0.05;
Real lineInductance = 0.1;
Real lineCapacitance = 0.1e-6;

// Simulation parameters
Real timeStep = 0.001;
Real finalTime = 1.0;

void powerFlow(SystemTopology& systemPF) {
	// ----- POWERFLOW FOR INITIALIZATION -----
	Real timeStepPF = finalTime;
	Real finalTimePF = finalTime+timeStepPF;
	String simNamePF = simName + "_PF";
	Logger::setLogDir("logs/" + simNamePF);

	// Components
	auto n1PF = SimNode<Complex>::make("n1", PhaseType::Single);
	auto n2PF = SimNode<Complex>::make("n2", PhaseType::Single);

	auto extnetPF = SP::Ph1::NetworkInjection::make("Slack", Logger::Level::debug);
	extnetPF->setParameters(Vnom);
	extnetPF->setBaseVoltage(Vnom);
	extnetPF->modifyPowerFlowBusType(PowerflowBusType::VD);

	auto linePF = SP::Ph1::PiLine::make("PiLine", Logger::Level::debug);
	linePF->setParameters(lineResistance, lineInductance, lineCapacitance);
	linePF->setBaseVoltage(Vnom);

	auto loadPF = SP::Ph1::Shunt::make("Load", Logger::Level::debug);
	loadPF->setParameters(pLoadNom / std::pow(Vnom, 2), - qLoadNom / std::pow(Vnom, 2));
	loadPF->setBaseVoltage(Vnom);

	// Topology
	extnetPF->connect({ n1PF });
	linePF->connect({ n1PF, n2PF });
	loadPF->connect({ n2PF });	
	systemPF = SystemTopology(50,
			SystemNodeList{n1PF, n2PF},
			SystemComponentList{extnetPF, linePF, loadPF});

	// Logging
	auto loggerPF = DataLogger::make(simNamePF);
	loggerPF->addAttribute("v1", n1PF->attribute("v"));
	loggerPF->addAttribute("v2", n2PF->attribute("v"));

	// Simulation
	Simulation simPF(simNamePF, Logger::Level::debug);
	simPF.setSystem(systemPF);
	simPF.setTimeStep(timeStepPF);
	simPF.setFinalTime(finalTimePF);
	simPF.setDomain(Domain::SP);
	simPF.setSolverType(Solver::Type::NRP);
	simPF.doInitFromNodesAndTerminals(false);
	simPF.addLogger(loggerPF);
	simPF.run();
}

// ----- DP SIMULATION -----
void simulateDP(SystemTopology& systemPF, String waveform) {
	Real timeStepDP = timeStep;
	Real finalTimeDP = finalTime+timeStepDP;
	String simNameDP = simName + "_DP";
	Logger::setLogDir("logs/" + simNameDP);

	// Components
	auto n1DP = SimNode<Complex>::make("n1", PhaseType::Single);
	auto n2DP = SimNode<Complex>::make("n2", PhaseType::Single);

	auto extnetDP = DP::Ph1::NetworkInjection::make("Slack", Logger::Level::debug);
	if(waveform == "cosineFM")
		extnetDP->setParameters(Complex(Vnom,0), 1.25, 1.25, -1.25);
	else
		extnetDP->setParameters(Complex(Vnom,0), 0.0, -6.25, 5.0, 0.4);

	auto lineDP = DP::Ph1::PiLine::make("PiLine", Logger::Level::debug);
	lineDP->setParameters(lineResistance, lineInductance, lineCapacitance);

	auto loadDP = DP::Ph1::RXLoad::make("Load", Logger::Level::debug);	
	loadDP->setParameters(pLoadNom, qLoadNom, Vnom);

	// Topology
	extnetDP->connect({ n1DP });
	lineDP->connect({ n1DP, n2DP });
	loadDP->connect({ n2DP });	
	auto systemDP = SystemTopology(50,
			SystemNodeList{n1DP, n2DP},
			SystemComponentList{extnetDP, lineDP, loadDP});

	// Initialization of dynamic topology
	CIM::Reader reader(simNameDP, Logger::Level::debug);
	reader.initDynamicSystemTopologyWithPowerflow(systemPF, systemDP);			

	// Logging
	auto loggerDP = DataLogger::make(simNameDP);
	loggerDP->addAttribute("v1", n1DP->attribute("v"));
	loggerDP->addAttribute("v2", n2DP->attribute("v"));
	loggerDP->addAttribute("isrc", extnetDP->attribute("i_intf"));
	loggerDP->addAttribute("i12", lineDP->attribute("i_intf"));
	loggerDP->addAttribute("irx", loadDP->attribute("i_intf"));
	loggerDP->addAttribute("f_src", extnetDP->attribute("f_src"));

	// Simulation
	Simulation sim(simNameDP, Logger::Level::debug);
	sim.setSystem(systemDP);
	sim.setTimeStep(timeStepDP);
	sim.setFinalTime(finalTimeDP);
	sim.setDomain(Domain::DP);
	
	sim.addLogger(loggerDP);
	sim.run();
}

// ----- SP SIMULATION -----
void simulateSP(SystemTopology& systemPF, String waveform) {
	
	Real timeStepSP = timeStep;
	Real finalTimeSP = finalTime+timeStepSP;
	String simNameSP = simName+"_SP";
	Logger::setLogDir("logs/" + simNameSP);

	// Components
	auto n1SP = SimNode<Complex>::make("n1", PhaseType::Single);
	auto n2SP = SimNode<Complex>::make("n2", PhaseType::Single);

	auto extnetSP = SP::Ph1::NetworkInjection::make("Slack", Logger::Level::debug);
	if(waveform == "cosineFM")
		extnetSP->setParameters(Complex(Vnom,0), 1.25, 1.25, -1.25);
	else
		extnetSP->setParameters(Complex(Vnom,0), 0.0, -6.25, 5.0, 0.4);

	auto lineSP = SP::Ph1::PiLine::make("PiLine", Logger::Level::debug);
	lineSP->setParameters(lineResistance, lineInductance, lineCapacitance);

	auto loadSP = SP::Ph1::Load::make("Load", Logger::Level::debug);	
	loadSP->setParameters(pLoadNom, qLoadNom, Vnom);

	// Topology
	extnetSP->connect({ n1SP });
	lineSP->connect({ n1SP, n2SP });
	loadSP->connect({ n2SP });	
	auto systemSP = SystemTopology(50,
			SystemNodeList{n1SP, n2SP},
			SystemComponentList{extnetSP, lineSP, loadSP});

	// Initialization of dynamic topology with values from powerflow
	CIM::Reader reader(simNameSP, Logger::Level::debug);
	reader.initDynamicSystemTopologyWithPowerflow(systemPF, systemSP);			

	// Logging
	auto loggerSP = DataLogger::make(simNameSP);
	loggerSP->addAttribute("v1", n1SP->attribute("v"));
	loggerSP->addAttribute("v2", n2SP->attribute("v"));
	loggerSP->addAttribute("i12", lineSP->attribute("i_intf"));
	loggerSP->addAttribute("f_src", extnetSP->attribute("f_src"));

	// Simulation
	Simulation sim(simNameSP, Logger::Level::debug);
	sim.setSystem(systemSP);
	sim.setTimeStep(timeStepSP);
	sim.setFinalTime(finalTimeSP);
	sim.setDomain(Domain::SP);
	
	sim.addLogger(loggerSP);
	sim.run();
}

// ----- EMT SIMULATION -----
void simulateEMT(SystemTopology& systemPF, String waveform) {
	String simName = "EMT_DP_SP_Slack_PiLine_PQLoad_FM";

	Real timeStepEMT = timeStep;
	Real finalTimeEMT = finalTime+timeStepEMT;
	String simNameEMT = simName + "_EMT";
	Logger::setLogDir("logs/" + simNameEMT);

	// Components
	auto n1EMT = SimNode<Real>::make("n1", PhaseType::ABC);
	auto n2EMT = SimNode<Real>::make("n2", PhaseType::ABC);

	auto extnetEMT = EMT::Ph3::NetworkInjection::make("Slack", Logger::Level::debug);
	if(waveform == "cosineFM")
		extnetEMT->setParameters(CPS::Math::singlePhaseVariableToThreePhase(Vnom), 1.25, 1.25, 48.75);
	else
		extnetEMT->setParameters(CPS::Math::singlePhaseVariableToThreePhase(Vnom), 48.75, -6.25, 5.0, 0.4);

	auto lineEMT = EMT::Ph3::PiLine::make("PiLine", Logger::Level::debug);
	lineEMT->setParameters(CPS::Math::singlePhaseParameterToThreePhase(lineResistance), CPS::Math::singlePhaseParameterToThreePhase(lineInductance), CPS::Math::singlePhaseParameterToThreePhase(lineCapacitance));

	auto loadEMT = EMT::Ph3::RXLoad::make("Load", Logger::Level::debug);
	loadEMT->setParameters(CPS::Math::singlePhasePowerToThreePhase(pLoadNom), CPS::Math::singlePhasePowerToThreePhase(qLoadNom), Vnom);

	// Topology
	extnetEMT->connect({ n1EMT });
	lineEMT->connect({ n1EMT, n2EMT });
	loadEMT->connect({ n2EMT });	
	auto systemEMT = SystemTopology(50,
			SystemNodeList{n1EMT, n2EMT},
			SystemComponentList{extnetEMT, lineEMT, loadEMT});

	// Initialization of dynamic topology
	CIM::Reader reader(simNameEMT, Logger::Level::debug);
	reader.initDynamicSystemTopologyWithPowerflow(systemPF, systemEMT);			

	// Logging
	auto loggerEMT = DataLogger::make(simNameEMT);
	loggerEMT->addAttribute("v1", n1EMT->attribute("v"));
	loggerEMT->addAttribute("v2", n2EMT->attribute("v"));
	loggerEMT->addAttribute("i12", lineEMT->attribute("i_intf"));
	loggerEMT->addAttribute("irx", loadEMT->attribute("i_intf"));
	loggerEMT->addAttribute("isrc", extnetEMT->attribute("i_intf"));
	loggerEMT->addAttribute("f_src", extnetEMT->attribute("f_src"));

	// load step sized in absolute terms
	//std::shared_ptr<SwitchEvent3Ph> loadStepEvent = CIM::Examples::createEventAddPowerConsumption3Ph("n2", 0.1-timeStepEMT, 100e3, systemEMT, Domain::EMT, loggerEMT);

	// Simulation
	Simulation sim(simNameEMT, Logger::Level::debug);
	sim.setSystem(systemEMT);
	sim.setTimeStep(timeStepEMT);
	sim.setFinalTime(finalTimeEMT);
	sim.setDomain(Domain::EMT);
	sim.addLogger(loggerEMT);
	//sim.addEvent(loadStepEvent);
	sim.run();
}

int main(int argc, char* argv[]) {
	CommandLineArgs args(argc, argv);
	if (argc > 1) {
		timeStep = args.timeStep;
		finalTime = args.duration;
	
		if (args.name != "dpsim")
			simName = args.name;
	}

	SystemTopology systemPF;
	powerFlow(systemPF);
	simulateDP(systemPF, "cosineFM");
	simulateDP(systemPF, "linearRamp");
	simulateSP(systemPF, "cosineFM");
	simulateSP(systemPF, "linearRamp");
	simulateEMT(systemPF, "cosineFM");
	simulateEMT(systemPF, "linearRamp");
}