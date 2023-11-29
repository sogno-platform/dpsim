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

int main(int argc, char* argv[]) {
	String simName = "DP_Slack_PiLine_PQLoad_with_PF_Init";

	// Component parameters
	Real Vnom = 20e3;
	Real pLoadNom = 100e3;
	Real qLoadNom = 50e3;
	Real lineResistance = 0.05;
	Real lineInductance = 0.1;
	Real lineCapacitance = 0.1e-6;

	// Simulation parameters
	Real timeStep = 0.001;
	Real finalTime = 2.0;
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

	// ----- DYNAMIC SIMULATION -----
	Real timeStepDP = timeStep;
	Real finalTimeDP = finalTime+timeStepDP;
	String simNameDP = simName + "_DP";
	Logger::setLogDir("logs/" + simNameDP);

	// Components
	auto n1DP = SimNode<Complex>::make("n1", PhaseType::Single);
	auto n2DP = SimNode<Complex>::make("n2", PhaseType::Single);

	auto extnetDP = DP::Ph1::NetworkInjection::make("Slack", Logger::Level::debug);
	extnetDP->setParameters(Complex(Vnom,0));

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
	systemDP.initWithPowerflow(systemPF, Domain::DP);

	// Logging
	auto loggerDP = DataLogger::make(simNameDP);
	loggerDP->logAttribute("v1", n1DP->attribute("v"));
	loggerDP->logAttribute("v2", n2DP->attribute("v"));
	loggerDP->logAttribute("isrc", extnetDP->attribute("i_intf"));
	loggerDP->logAttribute("i12", lineDP->attribute("i_intf"));
	loggerDP->logAttribute("irx", loadDP->attribute("i_intf"));
	loggerDP->logAttribute("f_src", extnetDP->attribute("f_src"));

	// load step sized in absolute terms
	//std::shared_ptr<SwitchEvent> loadStepEvent = CIM::Examples::Events::createEventAddPowerConsumption("n2", 0.1-timeStepDP, 100e3, systemDP, Domain::DP, loggerDP);

	// Simulation
	Simulation sim(simNameDP, Logger::Level::debug);
	sim.setSystem(systemDP);
	sim.setTimeStep(timeStepDP);
	sim.setFinalTime(finalTimeDP);
	sim.setDomain(Domain::DP);
	sim.addLogger(loggerDP);
	//sim.addEvent(loadStepEvent);
	sim.run();

}
