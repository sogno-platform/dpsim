/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include "dpsim-models/CIM/Reader.h"
#include <DPsim.h>
#include "../Examples.h"

using namespace std;
using namespace DPsim;
using namespace CPS;
using namespace CPS::CIM;

int main(int argc, char** argv){

	// Find CIM files
	std::list<fs::path> filenames;
	if (argc <= 1) {
		filenames = DPsim::Utils::findFiles({
			"Rootnet_FULL_NE_28J17h_DI.xml",
			"Rootnet_FULL_NE_28J17h_EQ.xml",
			"Rootnet_FULL_NE_28J17h_SV.xml",
			"Rootnet_FULL_NE_28J17h_TP.xml"
		}, "dpsim/Examples/CIM/grid-data/CIGRE_MV/NEPLAN/CIGRE_MV_no_tapchanger_noLoad1_LeftFeeder_With_LoadFlow_Results", "CIMPATH");
	}
	else {
		filenames = std::list<fs::path>(argv + 1, argv + argc);
	}

	// Simulation parameters
	Real timeStep = 1;
	Real finalTime = 2;
	String simName = "PF_CIGRE_MV_withDG";
	Examples::Grids::CIGREMV::ScenarioConfig scenario;
	Logger::setLogDir("logs/" + simName);

	// read original network topology
    CIM::Reader reader(Logger::Level::debug, Logger::Level::off, Logger::Level::debug);
    SystemTopology system = reader.loadCIM(scenario.systemFrequency, filenames, Domain::SP);
	Examples::Grids::CIGREMV::addInvertersToCIGREMV(system, scenario, Domain::SP);

    auto loggerPF = DPsim::DataLogger::make(simName);
    for (auto node : system.mNodes)
    {
        loggerPF->logAttribute(node->name() + ".V", node->attribute("v"));
    }
    Simulation simPF(simName, Logger::Level::debug);
	simPF.setSystem(system);
	simPF.setTimeStep(timeStep);
	simPF.setFinalTime(finalTime);
	simPF.setDomain(Domain::SP);
	simPF.setSolverType(Solver::Type::NRP);
	simPF.doInitFromNodesAndTerminals(true);
    simPF.addLogger(loggerPF);
    simPF.run();
}
