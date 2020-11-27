/* Copyright 2017-2020 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include "cps/CIM/Reader.h"
#include <DPsim.h>

using namespace std;
using namespace DPsim;
using namespace CPS;
using namespace CPS::CIM;


/*
 * This example runs the powerflow for the CIGRE MV benchmark system (neglecting the tap changers of the transformers)
 */
int main(int argc, char** argv){

	// Find CIM files
	std::list<fs::path> filenames;
	if (argc <= 1) {
		filenames = DPsim::Utils::findFiles({
		"case300.xml"
		}, "build/_deps/cim-data-src/Matpower_cases", "CIMPATH");
	}
	else {
		filenames = std::list<fs::path>(argv + 1, argv + argc);
	}

	String simName = "case300";
	CPS::Real system_freq = 50;

    CIM::Reader reader(simName, Logger::Level::info, Logger::Level::off);
    SystemTopology system = reader.loadCIM(system_freq, filenames, CPS::Domain::SP);

	auto logger = DPsim::DataLogger::make(simName);
	for (auto node : system.mNodes)
	{
		logger->addAttribute(node->name() + ".V", node->attribute("v"));
	}

	Simulation sim(simName, system, 1, 1, Domain::SP, Solver::Type::NRP, Logger::Level::debug, true);

	sim.addLogger(logger);
	sim.run();

	return 0;
}

