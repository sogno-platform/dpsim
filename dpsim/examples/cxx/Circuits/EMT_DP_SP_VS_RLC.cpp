/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <DPsim.h>

using namespace DPsim;
using namespace CPS;

void voltageSourceResistorEMT3ph() {
	Real timeStep = 0.00005;
	Real finalTime = 1;
	String simName = "EMT_VoltageSource_Resistor";
	Logger::setLogDir("logs/"+simName);

	// Nodes
	auto n1 = SimNode<Real>::make("n1", PhaseType::ABC);

	// Components
	auto vs = EMT::Ph3::VoltageSource::make("vs1");
	vs->setParameters(CPS::Math::singlePhaseVariableToThreePhase(CPS::Math::polar(100000, 0)), 50);

	auto res = EMT::Ph3::Resistor::make("R1", Logger::Level::debug);
	res->setParameters(CPS::Math::singlePhaseParameterToThreePhase(100));

	// Topology
	vs->connect({ SimNode<Real>::GND, n1 });
	res->connect({n1, SimNode<Real>::GND});

	auto sys = SystemTopology(50,
		SystemNodeList{n1},
		SystemComponentList{vs, res});

	// Logging
	auto logger = DataLogger::make(simName);
	logger->logAttribute("v1", n1->attribute("v"));
	logger->logAttribute("iR", res->attribute("i_intf"));

	// Simulation
	Simulation sim(simName);
	sim.setSystem(sys);
	sim.setSimulationParameters(timeStep, finalTime);
	sim.setDomain(Domain::EMT);
	sim.addLogger(logger);
	sim.run();
}

void voltageSourceResistorDP1ph() {
	Real timeStep = 0.00005;
	Real finalTime = 1;
	String simName = "DP_VoltageSource_Resistor";
	Logger::setLogDir("logs/"+simName);

	// Nodes
	auto n1 = SimNode<Complex>::make("n1");

	// Components
	auto vs = DP::Ph1::VoltageSource::make("vs1");
	vs->setParameters(CPS::Math::polar(100000, 0));

	auto res = DP::Ph1::Resistor::make("R1", Logger::Level::debug);
	res->setParameters(100);

	// Topology
	vs->connect({ SimNode<Complex>::GND, n1 });
	res->connect({n1, SimNode<Complex>::GND});

	auto sys = SystemTopology(50,
		SystemNodeList{n1},
		SystemComponentList{vs, res});

	// Logging
	auto logger = DataLogger::make(simName);
	logger->logAttribute("v1", n1->attribute("v"));
	logger->logAttribute("iR", res->attribute("i_intf"));

	// Simulation
	Simulation sim(simName);
	sim.setSystem(sys);
	sim.setSimulationParameters(timeStep, finalTime);
	sim.setDomain(Domain::DP);
	sim.addLogger(logger);
	sim.run();
}

void voltageSourceResistorSP1ph() {
	Real timeStep = 0.00005;
	Real finalTime = 1;
	String simName = "SP_VoltageSource_Resistor";
	Logger::setLogDir("logs/"+simName);

	// Nodes
	auto n1 = SimNode<Complex>::make("n1");

	// Components
	auto vs = SP::Ph1::VoltageSource::make("vs1");
	vs->setParameters(CPS::Math::polar(100000, 0));

	auto res = SP::Ph1::Resistor::make("R1", Logger::Level::debug);
	res->setParameters(100);

	// Topology
	vs->connect({ SimNode<Complex>::GND, n1 });
	res->connect({n1, SimNode<Complex>::GND});

	auto sys = SystemTopology(50,
		SystemNodeList{n1},
		SystemComponentList{vs, res});

	// Logging
	auto logger = DataLogger::make(simName);
	logger->logAttribute("v1", n1->attribute("v"));
	logger->logAttribute("iR", res->attribute("i_intf"));

	// Simulation
	Simulation sim(simName);
	sim.setSystem(sys);
	sim.setSimulationParameters(timeStep, finalTime);
	sim.setDomain(Domain::SP);
	sim.addLogger(logger);
	sim.run();
}

void voltageSourceInductorEMT3ph() {
	Real timeStep = 0.00005;
	Real finalTime = 1;
	String simName = "EMT_VoltageSource_Inductor";
	Logger::setLogDir("logs/"+simName);

	// Nodes
	auto n1 = SimNode<Real>::make("n1", PhaseType::ABC);
	auto n2 = SimNode<Real>::make("n2", PhaseType::ABC);

	// Components
	auto vs = EMT::Ph3::VoltageSource::make("vs1", Logger::Level::debug);
	vs->setParameters(CPS::Math::singlePhaseVariableToThreePhase(CPS::Math::polar(100000, 0)), 50);

	auto res = EMT::Ph3::Resistor::make("R1", Logger::Level::debug);
	res->setParameters(CPS::Math::singlePhaseParameterToThreePhase(5));

	auto ind = EMT::Ph3::Inductor::make("L1", Logger::Level::debug);
	ind->setParameters(CPS::Math::singlePhaseParameterToThreePhase(0.5));

	// Topology
	vs->connect({ SimNode<Real>::GND, n1 });
	res->connect({n1, n2});
	ind->connect({n2, SimNode<Real>::GND});

	auto sys = SystemTopology(50,
		SystemNodeList{n1, n2},
		SystemComponentList{vs, res, ind});

	// Logging
	auto logger = DataLogger::make(simName);
	logger->logAttribute("v1", n1->attribute("v"));
	logger->logAttribute("v2", n2->attribute("v"));
	logger->logAttribute("iR", res->attribute("i_intf"));
	logger->logAttribute("iL", ind->attribute("i_intf"));

	// Simulation
	Simulation sim(simName);
	sim.setSystem(sys);
	sim.setSimulationParameters(timeStep, finalTime);
	sim.setDomain(Domain::EMT);
	sim.addLogger(logger);
	sim.run();
}

void voltageSourceInductorDP1ph() {
	Real timeStep = 0.00005;
	Real finalTime = 1;
	String simName = "DP_VoltageSource_Inductor";
	Logger::setLogDir("logs/"+simName);

	// Nodes
	auto n1 = SimNode<Complex>::make("n1");
	auto n2 = SimNode<Complex>::make("n2");

	// Components
	auto vs = DP::Ph1::VoltageSource::make("vs1", Logger::Level::debug);
	vs->setParameters(CPS::Math::polar(100000, 0));

	auto res = DP::Ph1::Resistor::make("R1", Logger::Level::debug);
	res->setParameters(5);

	auto ind = DP::Ph1::Inductor::make("L1", Logger::Level::debug);
	ind->setParameters(0.5);

	// Topology
	vs->connect({ SimNode<Complex>::GND, n1 });
	res->connect({n1, n2});
	ind->connect({n2, SimNode<Complex>::GND});

	auto sys = SystemTopology(50,
		SystemNodeList{n1, n2},
		SystemComponentList{vs, res, ind});

	// Logging
	auto logger = DataLogger::make(simName);
	logger->logAttribute("v1", n1->attribute("v"));
	logger->logAttribute("v2", n2->attribute("v"));
	logger->logAttribute("iR", res->attribute("i_intf"));
	logger->logAttribute("iL", ind->attribute("i_intf"));

	// Simulation
	Simulation sim(simName);
	sim.setSystem(sys);
	sim.setSimulationParameters(timeStep, finalTime);
	sim.setDomain(Domain::DP);
	sim.addLogger(logger);
	sim.run();
}

void voltageSourceInductorSP1ph() {
	Real timeStep = 0.00005;
	Real finalTime = 1;
	String simName = "SP_VoltageSource_Inductor";
	Logger::setLogDir("logs/"+simName);

	// Nodes
	auto n1 = SimNode<Complex>::make("n1");
	auto n2 = SimNode<Complex>::make("n2");

	// Components
	auto vs = SP::Ph1::VoltageSource::make("vs1", Logger::Level::debug);
	vs->setParameters(CPS::Math::polar(100000, 0));

	auto res = SP::Ph1::Resistor::make("R1", Logger::Level::debug);
	res->setParameters(5);

	auto ind = SP::Ph1::Inductor::make("L1", Logger::Level::debug);
	ind->setParameters(0.5);

	// Topology
	vs->connect({ SimNode<Complex>::GND, n1 });
	res->connect({n1, n2});
	ind->connect({n2, SimNode<Complex>::GND});

	auto sys = SystemTopology(50,
		SystemNodeList{n1, n2},
		SystemComponentList{vs, res, ind});

	// Logging
	auto logger = DataLogger::make(simName);
	logger->logAttribute("v1", n1->attribute("v"));
	logger->logAttribute("v2", n2->attribute("v"));
	logger->logAttribute("iR", res->attribute("i_intf"));
	logger->logAttribute("iL", ind->attribute("i_intf"));

	// Simulation
	Simulation sim(simName);
	sim.setSystem(sys);
	sim.setSimulationParameters(timeStep, finalTime);
	sim.setDomain(Domain::SP);
	sim.addLogger(logger);
	sim.run();
}

void voltageSourceCapacitorEMT3ph() {
	Real timeStep = 0.00005;
	Real finalTime = 1;
	String simName = "EMT_VoltageSource_Capacitor";
	Logger::setLogDir("logs/"+simName);

	// Nodes
	auto n1 = SimNode<Real>::make("n1", PhaseType::ABC);
	auto n2 = SimNode<Real>::make("n2", PhaseType::ABC);

	// Components
	auto vs = EMT::Ph3::VoltageSource::make("vs1", Logger::Level::debug);
	vs->setParameters(CPS::Math::singlePhaseVariableToThreePhase(CPS::Math::polar(100000, 0)), 50);

	auto res = EMT::Ph3::Resistor::make("R1", Logger::Level::debug);
	res->setParameters(CPS::Math::singlePhaseParameterToThreePhase(5));

	auto cap = EMT::Ph3::Capacitor::make("C1", Logger::Level::debug);
	cap->setParameters(CPS::Math::singlePhaseParameterToThreePhase(10e-3));

	// Topology
	vs->connect({ SimNode<Real>::GND, n1 });
	res->connect({n1, n2});
	cap->connect({n2, SimNode<Real>::GND});

	auto sys = SystemTopology(50,
		SystemNodeList{n1, n2},
		SystemComponentList{vs, res, cap});

	// Logging
	auto logger = DataLogger::make(simName);
	logger->logAttribute("v1", n1->attribute("v"));
	logger->logAttribute("v2", n2->attribute("v"));
	logger->logAttribute("iR", res->attribute("i_intf"));
	logger->logAttribute("iC", cap->attribute("i_intf"));

	// Simulation
	Simulation sim(simName);
	sim.setSystem(sys);
	sim.setSimulationParameters(timeStep, finalTime);
	sim.setDomain(Domain::EMT);
	sim.addLogger(logger);
	sim.run();
}

void voltageSourceCapacitorDP1ph() {
	Real timeStep = 0.00005;
	Real finalTime = 1;
	String simName = "DP_VoltageSource_Capacitor";
	Logger::setLogDir("logs/"+simName);

	// Nodes
	auto n1 = SimNode<Complex>::make("n1");
	auto n2 = SimNode<Complex>::make("n2");

	// Components
	auto vs = DP::Ph1::VoltageSource::make("vs1", Logger::Level::debug);
	vs->setParameters(CPS::Math::polar(100000, 0));

	auto res = DP::Ph1::Resistor::make("R1", Logger::Level::debug);
	res->setParameters(5);

	auto cap = DP::Ph1::Capacitor::make("C1", Logger::Level::debug);
	cap->setParameters(10e-3);

	// Topology
	vs->connect({ SimNode<Complex>::GND, n1 });
	res->connect({n1, n2});
	cap->connect({n2, SimNode<Complex>::GND});

	auto sys = SystemTopology(50,
		SystemNodeList{n1, n2},
		SystemComponentList{vs, res, cap});

	// Logging
	auto logger = DataLogger::make(simName);
	logger->logAttribute("v1", n1->attribute("v"));
	logger->logAttribute("v2", n2->attribute("v"));
	logger->logAttribute("iR", res->attribute("i_intf"));
	logger->logAttribute("iC", cap->attribute("i_intf"));

	// Simulation
	Simulation sim(simName);
	sim.setSystem(sys);
	sim.setSimulationParameters(timeStep, finalTime);
	sim.setDomain(Domain::DP);
	sim.addLogger(logger);
	sim.run();
}

void voltageSourceCapacitorSP1ph() {
	Real timeStep = 0.00005;
	Real finalTime = 1;
	String simName = "SP_VoltageSource_Capacitor";
	Logger::setLogDir("logs/"+simName);

	// Nodes
	auto n1 = SimNode<Complex>::make("n1");
	auto n2 = SimNode<Complex>::make("n2");

	// Components
	auto vs = SP::Ph1::VoltageSource::make("vs1", Logger::Level::debug);
	vs->setParameters(CPS::Math::polar(100000, 0));

	auto res = SP::Ph1::Resistor::make("R1", Logger::Level::debug);
	res->setParameters(5);

	auto cap = SP::Ph1::Capacitor::make("C1", Logger::Level::debug);
	cap->setParameters(10e-3);

	// Topology
	vs->connect({ SimNode<Complex>::GND, n1 });
	res->connect({n1, n2});
	cap->connect({n2, SimNode<Complex>::GND});

	auto sys = SystemTopology(50,
		SystemNodeList{n1, n2},
		SystemComponentList{vs, res, cap});

	// Logging
	auto logger = DataLogger::make(simName);
	logger->logAttribute("v1", n1->attribute("v"));
	logger->logAttribute("v2", n2->attribute("v"));
	logger->logAttribute("iR", res->attribute("i_intf"));
	logger->logAttribute("iC", cap->attribute("i_intf"));

	// Simulation
	Simulation sim(simName);
	sim.setSystem(sys);
	sim.setSimulationParameters(timeStep, finalTime);
	sim.setDomain(Domain::SP);
	sim.addLogger(logger);
	sim.run();
}

int main(int argc, char* argv[]) {
	voltageSourceResistorEMT3ph();
	voltageSourceResistorDP1ph();
	voltageSourceResistorSP1ph();

	voltageSourceInductorEMT3ph();
	voltageSourceInductorDP1ph();
	voltageSourceInductorSP1ph();

	voltageSourceCapacitorEMT3ph();
	voltageSourceCapacitorDP1ph();
	voltageSourceCapacitorSP1ph();
}
