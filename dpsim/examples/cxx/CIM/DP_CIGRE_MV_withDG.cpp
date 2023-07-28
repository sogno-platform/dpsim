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

	// Simulation parameters
	String simName = "DP_CIGRE_MV_withDG";
	Examples::Grids::CIGREMV::ScenarioConfig scenario;
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
	// read original network topology
	String simNamePF = simName + "_Powerflow";
	Logger::setLogDir("logs/" + simNamePF);
    CIM::Reader reader(Logger::Level::debug, Logger::Level::off, Logger::Level::debug);
    SystemTopology systemPF = reader.loadCIM(scenario.systemFrequency, filenames, Domain::SP);
	Examples::Grids::CIGREMV::addInvertersToCIGREMV(systemPF, scenario, Domain::SP);

	// define logging
    auto loggerPF = CPS::DataLogger::make(simNamePF);
    for (auto node : systemPF.mNodes)
    {
        loggerPF->logAttribute(node->name() + ".V", node->attribute("v"));
    }

	// run powerflow
    Simulation simPF(simNamePF, Logger::Level::debug);
	simPF.setSystem(systemPF);
	simPF.setTimeStep(1);
	simPF.setFinalTime(2);
	simPF.setDomain(Domain::SP);
	simPF.setSolverType(Solver::Type::NRP);
	simPF.setSolverAndComponentBehaviour(Solver::Behaviour::Initialization);
	simPF.doInitFromNodesAndTerminals(true);
    simPF.addLogger(loggerPF);
    simPF.run();


	// ----- DYNAMIC SIMULATION -----
	Logger::setLogDir("logs/" + simName);
	CIM::Reader reader2(Logger::Level::debug, Logger::Level::off, Logger::Level::debug);
    SystemTopology systemDP = reader2.loadCIM(scenario.systemFrequency, filenames, CPS::Domain::DP);
	Examples::Grids::CIGREMV::addInvertersToCIGREMV(systemDP, scenario, Domain::DP);
	systemDP.initWithPowerflow(systemPF);

	auto logger = CPS::DataLogger::make(simName);

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

	// log output of PV connected at N11
	String pv_name = "pv_N11";
	auto pv = systemDP.component<CPS::SimPowerComp<Complex>>(pv_name);
	Examples::Grids::CIGREMV::logPVAttributes(logger, pv);

	Simulation sim(simName, Logger::Level::debug);
	sim.setSystem(systemDP);
	sim.setTimeStep(timeStep);
	sim.setFinalTime(finalTime);
	sim.setDomain(Domain::DP);
	sim.setSolverType(Solver::Type::MNA);
	sim.doInitFromNodesAndTerminals(true);

	sim.doSteadyStateInit(steadyStateInit);
	sim.addLogger(logger);
	sim.run();

}
