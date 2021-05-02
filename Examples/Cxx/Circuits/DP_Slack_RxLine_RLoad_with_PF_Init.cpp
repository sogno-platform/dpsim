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
	String simName = "DP_Slack_RxLine_RLoad";
	
	// Component parameters
	Real Vnom = 20e3;
	Real pLoadNom = 100e3;
	//Real loadResistance = 4e3;
	Real lineResistance = 0.05;
	Real lineInductance = 0.1;
	
	// Simulation parameters
	Real timeStep = 0.001;
	Real finalTime = 10.0;
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

	auto linePF = SP::Ph1::PiLine::make("RxLine", Logger::Level::debug);
	linePF->setParameters(lineResistance, lineInductance);
	linePF->setBaseVoltage(Vnom);

	auto loadPF = SP::Ph1::Resistor::make("RLoad", Logger::Level::debug);
	loadPF->setParameters(std::pow(Vnom, 2) / pLoadNom);
	loadPF->setBaseVoltage(Vnom);

	// Topology
	extnetPF->connect({ n1PF });
	linePF->connect({ n2PF, n1PF });
	loadPF->connect({ n2PF, SimNode<Complex>::GND });	
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

	// ----- DYNAMIC SIMULATION -----
	Real timeStepDP = timeStep;
	Real finalTimeDP = finalTime+timeStepDP;
	String simNameDP = simName + "_DP";
	Logger::setLogDir("logs/" + simNameDP);

	// Components
	auto n1DP = SimNode<Complex>::make("n1", PhaseType::Single);
	auto n2DP = SimNode<Complex>::make("n2", PhaseType::Single);

	auto extnetDP = DP::Ph1::NetworkInjection::make("Slack", Logger::Level::debug);
	// extnetDP->setParameters(Complex(Vnom,0), 0, -6.25, 0.05, 0.4);
	extnetDP->setParameters(Complex(Vnom,0), 1.25, 1.25, -1.25);
	
	auto lineDP = DP::Ph1::PiLine::make("RxLine", Logger::Level::debug);
	lineDP->setParameters(lineResistance, lineInductance);

	auto loadDP = DP::Ph1::Resistor::make("RLoad", Logger::Level::debug);	
	loadDP->setParameters(std::pow(Vnom, 2) / pLoadNom);

	// Topology
	extnetDP->connect({ n1DP });
	lineDP->connect({ n2DP, n1DP });
	loadDP->connect({ n2DP, SimNode<Complex>::GND });	
	auto systemDP = SystemTopology(50,
			SystemNodeList{n1DP, n2DP},
			SystemComponentList{extnetDP, lineDP, loadDP});

	CIM::Reader reader(simNameDP, Logger::Level::debug);
	reader.initDynamicSystemTopologyWithPowerflow(systemPF, systemDP);		

	// Logging
	auto loggerDP = DataLogger::make(simNameDP);
	loggerDP->addAttribute("v1", n1DP->attribute("v"));
	loggerDP->addAttribute("v2", n2DP->attribute("v"));
	loggerDP->addAttribute("isrc", extnetDP->attribute("i_intf"));
	loggerDP->addAttribute("i12", lineDP->attribute("i_intf"));
	loggerDP->addAttribute("iL", loadDP->attribute("i_intf"));
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
