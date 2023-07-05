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
using namespace CPS::SP;
using namespace CPS::CIM;


int main(int argc, char *argv[]) {

	// Simulation parameters
	String simName = "SP_WSCC9bus_SGReducedOrderVBR";
	Real timeStep = 10e-6;
	Real finalTime = 0.1;
	String sgType = "4";
	Bool withFault = true;
	Real startTimeFault = 0.2;
	Real endTimeFault = 0.3;
	String faultBusName= "BUS5";
	Real inertiaScalingFactor = 1.0;
	String logDirectory = "logs";

	// Find CIM files
	std::list<fs::path> filenames;
	CommandLineArgs args(argc, argv);
	if (argc <= 1) {
		filenames = Utils::findFiles({
			"WSCC-09_Dyn_Full_DI.xml",
			"WSCC-09_Dyn_Full_EQ.xml",
			"WSCC-09_Dyn_Full_SV.xml",
			"WSCC-09_Dyn_Full_TP.xml"
		}, "WSCC-09_Dyn_Full", "CIMPATH");
	}
	else {
		filenames = args.positionalPaths();
		timeStep = args.timeStep;
		finalTime = args.duration;

		if (args.name != "dpsim")
			simName = args.name;

		if (args.options.find("sgType") != args.options.end())
			sgType = args.getOptionString("sgType");

		if (args.options.find("withFault") != args.options.end())
			withFault = args.getOptionBool("withFault");

		if (args.options.find("startTimeFault") != args.options.end())
			startTimeFault = args.getOptionReal("startTimeFault");

		if (args.options.find("endTimeFault") != args.options.end())
			endTimeFault = args.getOptionReal("endTimeFault");

		if (args.options.find("faultBus") != args.options.end())
			faultBusName = args.getOptionString("faultBus");

		if (args.options.find("inertiaScalingFactor") != args.options.end())
			inertiaScalingFactor = args.getOptionReal("inertiaScalingFactor");

		if (args.options.find("logDirectory") != args.options.end())
			logDirectory = args.getOptionString("logDirectory");
	}

	// Configure logging
	Logger::Level logLevel = Logger::Level::info;

	// apply downsampling for simulation step sizes lower than 10us
	Real logDownSampling;
	if (timeStep < 10e-6)
		logDownSampling = floor((10e-6) / timeStep);
	else
		logDownSampling = 1.0;

	// ----- POWERFLOW FOR INITIALIZATION -----
	// read original network topology
	String simNamePF = simName + "_PF";
	Logger::setLogDir(logDirectory + "/" + simNamePF);
    CPS::CIM::Reader reader(simNamePF, logLevel, logLevel);
    SystemTopology systemPF = reader.loadCIM(60, filenames, Domain::SP, PhaseType::Single, CPS::GeneratorType::PVNode);
	systemPF.component<CPS::SP::Ph1::SynchronGenerator>("GEN1")->modifyPowerFlowBusType(CPS::PowerflowBusType::VD);

	// define logging
    auto loggerPF = DPsim::DataLogger::make(simNamePF);
    for (auto node : systemPF.mNodes)
        loggerPF->logAttribute(node->name() + ".V", node->attribute("v"));

	// run powerflow
    Simulation simPF(simNamePF, logLevel);
	simPF.setSystem(systemPF);
	simPF.setTimeStep(finalTime);
	simPF.setFinalTime(2*finalTime);
	simPF.setDomain(Domain::SP);
	simPF.setSolverType(Solver::Type::NRP);
	simPF.setSolverAndComponentBehaviour(Solver::Behaviour::Initialization);
	simPF.doInitFromNodesAndTerminals(true);
    simPF.addLogger(loggerPF);
    simPF.run();

	// ----- DYNAMIC SIMULATION -----
	Logger::setLogDir(logDirectory + "/" + simName);

	CPS::CIM::Reader reader2(simName, logLevel, logLevel);
	SystemTopology sys;
	if (sgType=="3")
		sys = reader2.loadCIM(60, filenames, Domain::SP, PhaseType::Single, CPS::GeneratorType::SG3OrderVBR);
	else if (sgType=="4")
		sys = reader2.loadCIM(60, filenames, Domain::SP, PhaseType::Single, CPS::GeneratorType::SG4OrderVBR);
	else if (sgType=="6b")
		sys = reader2.loadCIM(60, filenames, Domain::SP, PhaseType::Single, CPS::GeneratorType::SG6bOrderVBR);
	else
		throw CPS::SystemError("Unsupported reduced-order SG type!");

	// Optionally extend topology with switch
	auto faultSP = Ph1::Switch::make("Fault", logLevel);
	if (withFault) {
		faultSP->setParameters(1e12,0.02*529);
		faultSP->connect({ SimNode::GND, sys.node<SimNode>(faultBusName) });
		faultSP->open();
		sys.addComponent(faultSP);
	}

	sys.initWithPowerflow(systemPF);
	for (auto comp : sys.mComponents) {
		if (auto genReducedOrder = std::dynamic_pointer_cast<CPS::Base::ReducedOrderSynchronGenerator<Complex>>(comp)) {
			auto genPF = systemPF.component<CPS::SP::Ph1::SynchronGenerator>(comp->name());
			genReducedOrder->terminal(0)->setPower(-genPF->getApparentPower());
			genReducedOrder->scaleInertiaConstant(inertiaScalingFactor);
		}
	}

	// Logging
	// log node voltage
	auto logger = DataLogger::make(simName, true, logDownSampling);
		for (auto node : sys.mNodes)
			logger->logAttribute(node->name() + ".V", node->attribute("v"));

	// log generator vars
	for (auto comp : sys.mComponents) {
		if (auto genReducedOrder = std::dynamic_pointer_cast<CPS::Base::ReducedOrderSynchronGenerator<Complex>>(comp)){
			logger->logAttribute(genReducedOrder->name() + ".Tm", genReducedOrder->attribute("Tm"));
			logger->logAttribute(genReducedOrder->name() + ".Te", genReducedOrder->attribute("Te"));
			logger->logAttribute(genReducedOrder->name() + ".omega", genReducedOrder->attribute("w_r"));
			logger->logAttribute(genReducedOrder->name() + ".delta", genReducedOrder->attribute("delta"));
		}
	}

	Simulation sim(simName, logLevel);
	sim.setSystem(sys);
	sim.setDomain(Domain::SP);
	sim.setSolverType(Solver::Type::MNA);
	sim.setTimeStep(timeStep);
	sim.setFinalTime(finalTime);
	sim.doSystemMatrixRecomputation(true);
	sim.setDirectLinearSolverImplementation(CPS::DirectLinearSolverImpl::SparseLU);
	sim.addLogger(logger);

	// Optionally add switch event
	if (withFault) {
		auto faultEvent1 = SwitchEvent::make(startTimeFault, faultSP, true);
		auto faultEvent2 = SwitchEvent::make(endTimeFault, faultSP, false);
		sim.addEvent(faultEvent1);
		sim.addEvent(faultEvent2);
	}

	sim.run();

	return 0;
}
