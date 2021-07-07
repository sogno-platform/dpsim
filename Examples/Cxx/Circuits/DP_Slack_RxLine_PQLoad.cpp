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
	String simName = "DP_Slack_RxLine_PQLoad";
	
	// Component parameters
	Real Vnom = 20e3;
	Real pLoadNom = 100e3;
	Real qLoadNom = 50e3;
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
		if (args.options.find("qLoad") != args.options.end())
			qLoadNom = args.options["qLoad"];
	}

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

	auto lineDP = DP::Ph1::RxLine::make("RxLine", Logger::Level::debug);
	lineDP->setParameters(lineResistance, lineInductance);

	auto loadDP = DP::Ph1::RXLoad::make("Load", Logger::Level::debug);	
	loadDP->setParameters(pLoadNom, qLoadNom, Vnom);

	// Topology
	extnetDP->connect({ n1DP });
	lineDP->connect({ n1DP, n2DP });
	loadDP->connect({ n2DP });	
	auto systemDP = SystemTopology(50,
			SystemNodeList{n1DP, n2DP},
			SystemComponentList{extnetDP, lineDP, loadDP});

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
