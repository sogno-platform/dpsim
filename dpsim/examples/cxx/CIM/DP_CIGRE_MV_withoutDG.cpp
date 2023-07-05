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
	String simName = "DP_CIGRE_MV_withoutDG";
	Real systemFrequency = 50;
	std::list<fs::path> filenames;
	Real timeStep;
	Real finalTime;
	Bool steadyStateInit;

	// Set remaining simulation parameters using default values or command line infos
	CommandLineArgs args(argc, argv);
	if (argc <= 1) {
		filenames = DPsim::Utils::findFiles({
			"Rootnet_FULL_NE_28J17h_DI.xml",
			"Rootnet_FULL_NE_28J17h_EQ.xml",
			"Rootnet_FULL_NE_28J17h_SV.xml",
			"Rootnet_FULL_NE_28J17h_TP.xml"
		}, "dpsim/Examples/CIM/grid-data/CIGRE_MV/NEPLAN/CIGRE_MV_no_tapchanger_noLoad1_LeftFeeder_With_LoadFlow_Results", "CIMPATH");
		timeStep = 0.1e-3;
		finalTime = 1;
		steadyStateInit = false;
	}
	else {
		filenames = args.positionalPaths();
		timeStep = args.timeStep;
		finalTime = args.duration;
		steadyStateInit = args.steadyInit;
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

	// set solver parameters
	auto solverParameters = std::make_shared<SolverParametersMNA>();
	solverParameters->setSolverAndComponentBehaviour(Solver::Behaviour::Initialization);
	solverParameters->setInitFromNodesAndTerminals(true);

	//
    Simulation simPF(simNamePF, Logger::Level::debug);
	simPF.setSystem(systemPF);
	simPF.setTimeStep(1);
	simPF.setFinalTime(2);
	simPF.setSolverParameters(Domain::SP, Solver::Type::NRP, solverParameters);
    simPF.addLogger(loggerPF);
    simPF.run();

	// ----- DYNAMIC SIMULATION -----
	Logger::setLogDir("logs/" + simName);
	CIM::Reader reader2(simName, Logger::Level::debug, Logger::Level::debug);
    SystemTopology systemDP = reader2.loadCIM(systemFrequency, filenames, CPS::Domain::DP);
	systemDP.initWithPowerflow(systemPF);

	auto logger = DPsim::DataLogger::make(simName);

	// log node voltages
	for (auto node : systemDP.mNodes)
	{
		logger->logAttribute(node->name() + ".V", node->attribute("v"));
	}

	// log line currents
	for (auto comp : systemDP.mComponents) {
		if (dynamic_pointer_cast<CPS::DP::Ph1::PiLine>(comp))
			logger->logAttribute(comp->name() + ".I", comp->attribute("i_intf"));
	}

	// log load currents
	for (auto comp : systemDP.mComponents) {
		if (dynamic_pointer_cast<CPS::DP::Ph1::RXLoad>(comp))
			logger->logAttribute(comp->name() + ".I", comp->attribute("i_intf"));
	}

	// set solver parameters
	auto solverParameterDP = std::make_shared<SolverParametersMNA>();
	solverParameterDP->setInitFromNodesAndTerminals(true);
	solverParameterDP->setDirectLinearSolverImplementation(CPS::DirectLinearSolverImpl::SparseLU);
	solverParameterDP->doSystemMatrixRecomputation(false);
	solverParameterDP->doSteadyStateInit(steadyStateInit);

	//
	Simulation sim(simName, Logger::Level::debug);
	sim.setSystem(systemDP);
	sim.setSimulationParameters(timeStep, finalTime);
	sim.setSolverParameters(Domain::DP, Solver::Type::MNA, solverParameterSP);
	sim.addLogger(logger);

	sim.run();
}
