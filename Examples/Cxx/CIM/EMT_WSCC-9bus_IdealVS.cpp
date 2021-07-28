/* Copyright 2017-2020 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <iostream>
#include <list>

#include <DPsim.h>

using namespace DPsim;
using namespace CPS::EMT;

int main(int argc, char *argv[]) {

	Real timeStep;
	Real finalTime;

	// Find CIM files
	std::list<fs::path> filenames;
	CommandLineArgs args(argc, argv);
	if (argc <= 1) {
		filenames = Utils::findFiles({
			"WSCC-09_RX_DI.xml",
			"WSCC-09_RX_EQ.xml",
			"WSCC-09_RX_SV.xml",
			"WSCC-09_RX_TP.xml"
		}, "build/_deps/cim-data-src/WSCC-09/WSCC-09_RX", "CIMPATH");
		timeStep = 1e-6;
		finalTime = 10e-6;
	}
	else {
		filenames = args.positionalPaths();
		timeStep = args.timeStep;
		finalTime = args.duration;
	}

	String simName = "EMT_WSCC-9bus_IdealVS";
	Logger::setLogDir("logs/"+simName);

	CPS::CIM::Reader reader(simName, Logger::Level::debug, Logger::Level::debug);
	SystemTopology sys = reader.loadCIM(60, filenames, Domain::EMT, PhaseType::ABC, CPS::GeneratorType::IdealVoltageSource);

	// Logging
	auto logger = DataLogger::make(simName);
	logger->addAttribute("v1", sys.node<SimNode>("BUS1")->attribute("v"));
	logger->addAttribute("v2", sys.node<SimNode>("BUS2")->attribute("v"));
	logger->addAttribute("v3", sys.node<SimNode>("BUS3")->attribute("v"));
	logger->addAttribute("v4", sys.node<SimNode>("BUS4")->attribute("v"));
	logger->addAttribute("v5", sys.node<SimNode>("BUS5")->attribute("v"));
	logger->addAttribute("v6", sys.node<SimNode>("BUS6")->attribute("v"));
	logger->addAttribute("v7", sys.node<SimNode>("BUS7")->attribute("v"));
	logger->addAttribute("v8", sys.node<SimNode>("BUS8")->attribute("v"));
	logger->addAttribute("v9", sys.node<SimNode>("BUS9")->attribute("v"));

	// log generator's current
	for (auto comp : sys.mComponents) {
		if (std::dynamic_pointer_cast<CPS::EMT::Ph3::SynchronGeneratorIdeal>(comp))
			logger->addAttribute(comp->name() + ".I", comp->attribute("i_intf"));
	}

	Simulation sim(simName, Logger::Level::info);
	sim.setSystem(sys);
	sim.setDomain(Domain::EMT);
	sim.setTimeStep(timeStep);
	sim.setFinalTime(finalTime);
	sim.addLogger(logger);
	sim.run();

	return 0;
}
