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
	String simName = "SP_Slack_PiLine_PQLoad_with_PF_Init";

	// Component parameters
	Real Vnom = 20e3;
	Real pLoadNom = 100e3;
	Real qLoadNom = 50e3;
	Real lineResistance = 0.05;
	Real lineInductance = 0.1;
	Real lineCapacitance = 0.1e-6;
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
	String simNamePF = simName+"_PF";
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

	// ----- SP SIMULATION -----
	Real timeStepSP = timeStep;
	Real finalTimeSP = finalTime+timeStepSP;
	String simNameSP = simName+"_SP";
	Logger::setLogDir("logs/" + simNameSP);

	// Components
	auto n1SP = SimNode<Complex>::make("n1", PhaseType::Single);
	auto n2SP = SimNode<Complex>::make("n2", PhaseType::Single);

	auto extnetSP = SP::Ph1::NetworkInjection::make("Slack", Logger::Level::debug);
	extnetSP->setParameters(Complex(Vnom, 0));

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
