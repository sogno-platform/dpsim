/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include "dpsim-models/CIM/Reader.h"
#include <DPsim.h>
#include <dpsim-models/CSVReader.h>

using namespace std;
using namespace DPsim;
using namespace CPS;
using namespace CPS::CIM;

int main(int argc, char** argv){

	// Simulation parameters
	String simName = "EMT_CIGRE_MV_withoutDG";
	Real systemFrequency = 50;
	std::list<fs::path> filenames;
	Real timeStep;
	Real finalTime;

	// Set remaining simulation parameters using default values or command line infos
	CommandLineArgs args(argc, argv);
	if (argc <= 1) {
		filenames = DPsim::Utils::findFiles({
			"Rootnet_FULL_NE_28J17h_DI.xml",
			"Rootnet_FULL_NE_28J17h_EQ.xml",
			"Rootnet_FULL_NE_28J17h_SV.xml",
			"Rootnet_FULL_NE_28J17h_TP.xml"
		}, "dpsim/Examples/CIM/grid-data/CIGRE_MV/NEPLAN/CIGRE_MV_no_tapchanger_noLoad1_LeftFeeder_With_LoadFlow_Results", "CIMPATH");
		timeStep = 1e-3;
		finalTime = 0.1;
	}
	else {
		filenames = args.positionalPaths();
		timeStep = args.timeStep;
		finalTime = args.duration;
	}

	// ----- POWERFLOW FOR INITIALIZATION -----
	String simNamePF = simName + "_Powerflow";
	Logger::setLogDir("logs/" + simNamePF);
    CIM::Reader reader(simNamePF, Logger::Level::debug, Logger::Level::debug);
    SystemTopology systemPF = reader.loadCIM(systemFrequency, filenames, CPS::Domain::SP);

    auto loggerPF = DPsim::DataLogger::make(simNamePF);
    for (auto node : systemPF.mNodes)
    {
        loggerPF->logAttribute(node->name() + ".V", node->attribute("v"));
    }
    Simulation simPF(simNamePF, Logger::Level::debug);
	simPF.setSystem(systemPF);
	simPF.setTimeStep(1);
	simPF.setFinalTime(2);
	simPF.setDomain(Domain::SP);
	simPF.setSolverType(Solver::Type::NRP);
	simPF.setSolverAndComponentBehaviour(Solver::Behaviour::Initialization);
	simPF.doInitFromNodesAndTerminals(true);
	simPF.doSteadyStateInit(false);
    simPF.addLogger(loggerPF);
    simPF.run();

	// ----- DYNAMIC SIMULATION -----
	Logger::setLogDir("logs/" + simName);
	CIM::Reader reader2(simName, Logger::Level::debug, Logger::Level::debug);
    SystemTopology systemEMT = reader2.loadCIM(systemFrequency, filenames, CPS::Domain::EMT, PhaseType::ABC);
	systemEMT.initWithPowerflow(systemPF, CPS::Domain::EMT);

	auto logger = DPsim::DataLogger::make(simName);

	// log node voltages
	for (auto node : systemEMT.mNodes)
		logger->logAttribute(node->name() + ".V", node->attribute("v"));

	// log line currents
	for (auto comp : systemEMT.mComponents) {
		if (dynamic_pointer_cast<CPS::EMT::Ph3::PiLine>(comp))
			logger->logAttribute(comp->name() + ".I", comp->attribute("i_intf"));
	}

	// log load currents
	for (auto comp : systemEMT.mComponents) {
		if (dynamic_pointer_cast<CPS::EMT::Ph3::RXLoad>(comp))
			logger->logAttribute(comp->name() + ".I", comp->attribute("i_intf"));
	}

	Simulation sim(simName, Logger::Level::debug);
	sim.setSystem(systemEMT);
	sim.setTimeStep(timeStep);
	sim.setFinalTime(finalTime);
	sim.setDomain(Domain::EMT);
	sim.setSolverType(Solver::Type::MNA);
	sim.doInitFromNodesAndTerminals(true);
	sim.doSteadyStateInit(false);
	sim.addLogger(logger);

	sim.run();
}
