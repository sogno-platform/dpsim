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
#include <dpsim/Utils.h>

using namespace DPsim;

int main(int argc, char *argv[]) {
	CommandLineArgs args(argc, argv);

	String simName = "dpsim";

	CIMReader reader(simName);
	SystemTopology sys = reader.loadCIM(args.sysFreq, args.positionalPaths(), args.solver.domain);

	Simulation sim(simName, sys, args.timeStep, args.duration, args.solver.domain, args.solver.type);

	// ToDO: add DataLoggers

	sim.run();

	return 0;
}
