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

int main(int argc, char* argv[]) {
	String simName = "EMT_Slack_PiLine_PQLoad_with_PF_Init";
	
	// Parameters
	Real Vnom = 20e3;
	Real pLoadNom = 100e3;
	Real qLoadNom = 50e3;
	Real lineResistance = 0.05;
	Real lineInductance = 0.1;
	Real lineCapacitance = 0.1e-6;
	
	// Simulation parameters
	Real timeStep = 0.001;
	Real finalTime = 1.0;
	CommandLineArgs args(argc, argv);
	if (argc > 1) {
		timeStep = args.timeStep;
		finalTime = args.duration;
		
		if (args.name != "dpsim")
			simName = args.name;
	}

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
	auto systemPF = SystemTopology(50,
			SystemNodeList{n1PF, n2PF},
			SystemComponentList{extnetPF, linePF, loadPF});

	// Logging
	auto loggerPF = DataLogger::make(simNamePF);
	loggerPF->addAttribute("v1", n1PF->attribute("v"));
	loggerPF->addAttribute("v2", n2PF->attribute("v"));
	loggerPF->addAttribute("i12", linePF->attribute("i_intf"));

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

	// ----- DYNAMIC SIMULATION -----
	Real timeStepEMT = timeStep;
	Real finalTimeEMT = finalTime+timeStepEMT;
	String simNameEMT = simName + "_EMT";
	Logger::setLogDir("logs/" + simNameEMT);

	// Components
	auto n1EMT = SimNode<Real>::make("n1", PhaseType::ABC);
	auto n2EMT = SimNode<Real>::make("n2", PhaseType::ABC);

	auto extnetEMT = EMT::Ph3::NetworkInjection::make("Slack", Logger::Level::debug);
	extnetEMT->setParameters(CPS::Math::singlePhaseVariableToThreePhase(Vnom), 50);

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
	loggerEMT->addAttribute("isrc", extnetEMT->attribute("i_intf"));
	loggerEMT->addAttribute("i12", lineEMT->attribute("i_intf"));
	loggerEMT->addAttribute("irx", loadEMT->attribute("i_intf"));
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
