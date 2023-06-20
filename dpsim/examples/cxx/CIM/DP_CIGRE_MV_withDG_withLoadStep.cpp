/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <DPsim.h>
#include <dpsim-models/CSVReader.h>
#include "../Examples.h"

using namespace std;
using namespace DPsim;
using namespace CPS;
using namespace CPS::CIM;

int main(int argc, char** argv){

	// Simulation parameters
	Examples::Grids::CIGREMV::ScenarioConfig scenario;
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
		timeStep = 0.1e-3;
		finalTime = 1;
	}
	else {
		filenames = args.positionalPaths();
		timeStep = args.timeStep;
		finalTime = args.duration;
	}

	// ----- POWERFLOW FOR INITIALIZATION -----
	// read original network topology
	//String simName = "DP_CIGRE_MV_withDG_withLoadStep";
	String simName = "DP_CIGRE_MV_withDG_withLoadStep";
	String simNamePF = simName + "_Powerflow";
	Logger::setLogDir("logs/" + simNamePF);
    CIM::Reader reader(simNamePF, Logger::Level::debug, Logger::Level::debug);
    SystemTopology systemPF = reader.loadCIM(scenario.systemFrequency, filenames, Domain::SP);
	Examples::Grids::CIGREMV::addInvertersToCIGREMV(systemPF, scenario, Domain::SP);

	// define logging
    auto loggerPF = DPsim::DataLogger::make(simNamePF);
    for (auto node : systemPF.mNodes) {
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
	CIM::Reader reader2(simName, Logger::Level::info, Logger::Level::debug);
    SystemTopology systemDP = reader2.loadCIM(scenario.systemFrequency, filenames, CPS::Domain::DP);
	Examples::Grids::CIGREMV::addInvertersToCIGREMV(systemDP, scenario, Domain::DP);
	systemDP.initWithPowerflow(systemPF);

	auto logger = DPsim::DataLogger::make(simName);

	// log node voltages
	for (auto node : systemDP.mNodes)
		logger->logAttribute(node->name() + ".V", node->attribute("v"));

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

	// // load step sized relative to nominal load at N11
	// std::shared_ptr<SwitchEvent> loadStepEvent = Examples::Events::createEventAddPowerConsumption("N11", 2-timeStep, 5*systemPF.component<CPS::SP::Ph1::Load>("LOAD-H-11")->attributeTyped<CPS::Real>("P")->get(), systemDP, Domain::DP, logger);

	// load step sized in absolute terms
	std::shared_ptr<SwitchEvent> loadStepEvent = Examples::Events::createEventAddPowerConsumption("N11", 2-timeStep, 1500.0e3, systemDP, Domain::DP, logger);

	Simulation sim(simName, Logger::Level::debug);
	sim.setSystem(systemDP);
	sim.setTimeStep(timeStep);
	sim.setFinalTime(finalTime);
	sim.setDomain(Domain::DP);
	sim.setSolverType(Solver::Type::MNA);
	sim.doInitFromNodesAndTerminals(true);
	sim.doSteadyStateInit(false);
	sim.addLogger(logger);
	sim.addEvent(loadStepEvent);

	sim.run();
}
