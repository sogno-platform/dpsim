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

void simElementsSP1ph() {
	Real timeStep = 0.00005;
	Real finalTime = 1;
	String simName = "SP_Slack_Elements";
	Logger::setLogDir("logs/"+simName);

	// Nodes
	auto n1 = SimNode<Complex>::SimNode::make("n1");

	// Components
	auto vs = SP::Ph1::VoltageSource::make("v_1");
	vs->setParameters(CPS::Math::polar(100000, 0));

	auto load = SP::Ph1::Resistor::make("R_load");
	load->setParameters(10000);

	// Topology
	vs->connect({ SimNode<Complex>::GND, n1 });
	load->connect({ n1, SimNode<Complex>::GND });

	auto sys = SystemTopology(50,
		SystemNodeList{n1},
		SystemComponentList{vs, load});

	// Logging
	auto logger = DataLogger::make(simName);
	logger->logAttribute("v1", n1->attribute("v"));
	logger->logAttribute("i1", vs->attribute("i_intf"));

	Simulation sim(simName);
	sim.setSystem(sys);
	sim.setSimulationParameters(timeStep, finalTime);
	sim.addLogger(logger);

	sim.run();
}

void simComponentSP1ph() {
	Real timeStep = 0.00005;
	Real finalTime = 1;
	String simName = "SP_Slack_Component";
	Logger::setLogDir("logs/"+simName);

	// Nodes
	auto n1 = SimNode<Complex>::make("n1");

	// Components
	auto sl = SP::Ph1::NetworkInjection::make("v_1");
	sl->setParameters(CPS::Math::polar(100000, 0));

	auto load = SP::Ph1::Resistor::make("R_load");
	load->setParameters(10000);

	// Topology
	sl->connect({ n1 });
	load->connect({ n1, SimNode<Complex>::GND });

	auto sys = SystemTopology(50,
		SystemNodeList{n1},
		SystemComponentList{sl, load});

	// Logging
	auto logger = DataLogger::make(simName);
	logger->logAttribute("v1", n1->attribute("v"));
	logger->logAttribute("i1", sl->attribute("i_intf"));

	Simulation sim(simName);
	sim.setSystem(sys);
	sim.setTimeStep(timeStep);
	sim.setFinalTime(finalTime);
	sim.addLogger(logger);

	sim.run();
}

void simElementsDP1ph() {
	Real timeStep = 0.00005;
	Real finalTime = 1;
	String simName = "DP_Slack_Elements";
	Logger::setLogDir("logs/"+simName);

	// Nodes
	auto n1 = SimNode<Complex>::SimNode::make("n1");

	// Components
	auto vs = DP::Ph1::VoltageSource::make("v_1");
	vs->setParameters(CPS::Math::polar(100000, 0));

	auto load = DP::Ph1::Resistor::make("R_load");
	load->setParameters(10000);

	// Topology
	vs->connect({ SimNode<Complex>::GND, n1 });
	load->connect({ n1, SimNode<Complex>::GND });

	auto sys = SystemTopology(50,
		SystemNodeList{n1},
		SystemComponentList{vs, load});

	// Logging
	auto logger = DataLogger::make(simName);
	logger->logAttribute("v1", n1->attribute("v"));
	logger->logAttribute("i1", vs->attribute("i_intf"));

	Simulation sim(simName);
	sim.setSystem(sys);
	sim.setSimulationParameters(timeStep, finalTime);
	sim.addLogger(logger);

	sim.run();
}

void simComponentDP1ph() {
	Real timeStep = 0.00005;
	Real finalTime = 1;
	String simName = "DP_Slack_Component";
	Logger::setLogDir("logs/"+simName);

	// Nodes
	auto n1 = SimNode<Complex>::make("n1");

	// Components
	auto sl = DP::Ph1::NetworkInjection::make("v_1");
	sl->setParameters(CPS::Math::polar(100000, 0));

	auto load = DP::Ph1::Resistor::make("R_load");
	load->setParameters(10000);

	// Topology
	sl->connect({ n1 });
	load->connect({ n1, SimNode<Complex>::GND });

	auto sys = SystemTopology(50,
		SystemNodeList{n1},
		SystemComponentList{sl, load});

	// Logging
	auto logger = DataLogger::make(simName);
	logger->logAttribute("v1", n1->attribute("v"));
	logger->logAttribute("i1", sl->attribute("i_intf"));

	Simulation sim(simName);
	sim.setSystem(sys);
	sim.setSimulationParameters(timeStep, finalTime);
	sim.addLogger(logger);

	sim.run();
}

void simElementsEMT3ph() {
	Real timeStep = 0.00005;
	Real finalTime = 1;
	String simName = "EMT_Slack_Elements";
	Logger::setLogDir("logs/"+simName);

	// Nodes
	auto n1 = SimNode<Real>::make("n1", PhaseType::ABC);

	// Components
	auto vs = EMT::Ph3::VoltageSource::make("vs1");
	vs->setParameters(CPS::Math::singlePhaseVariableToThreePhase(CPS::Math::polar(100000, 0)), 50);

	auto load = EMT::Ph3::Resistor::make("Rload");
	load->setParameters(CPS::Math::singlePhaseParameterToThreePhase(10000));

	// Topology
	vs->connect({ SimNode<Real>::GND, n1 });
	load->connect({n1, SimNode<Real>::GND});

	auto sys = SystemTopology(50,
		SystemNodeList{n1},
		SystemComponentList{vs, load});

	// Logging
	auto logger = DataLogger::make(simName);
	logger->logAttribute("v1", n1->attribute("v"));
	logger->logAttribute("i1", vs->attribute("i_intf"));

	// Simulation
	Simulation sim(simName);
	sim.setSystem(sys);
	sim.setSimulationParameters(timeStep, finalTime);
	sim.setDomain(Domain::EMT);
	sim.addLogger(logger);
	sim.run();
}

void simComponentEMT3ph() {
	Real timeStep = 0.00005;
	Real finalTime = 1;
	String simName = "EMT_Slack_Component";
	Logger::setLogDir("logs/"+simName);

	// Nodes
	auto n1 = SimNode<Real>::make("n1", PhaseType::ABC);

	// Components
	auto vs = EMT::Ph3::NetworkInjection::make("vs1", Logger::Level::debug);
	vs->setParameters(CPS::Math::singlePhaseVariableToThreePhase(CPS::Math::polar(100000, 0)), 50);

	auto load = EMT::Ph3::Resistor::make("Rload");
	load->setParameters(CPS::Math::singlePhaseParameterToThreePhase(10000));

	// Topology
	vs->connect({ n1 });
	load->connect({n1, SimNode<Real>::GND});

	auto sys = SystemTopology(50,
		SystemNodeList{n1},
		SystemComponentList{vs, load});

	// Logging
	auto logger = DataLogger::make(simName);
	logger->logAttribute("v1", n1->attribute("v"));
	logger->logAttribute("i1", vs->attribute("i_intf"));

	// Simulation
	Simulation sim(simName);
	sim.setSystem(sys);
	sim.setSimulationParameters(timeStep, finalTime);
	sim.setDomain(Domain::EMT);
	sim.addLogger(logger);
	sim.run();
}

int main(int argc, char* argv[]) {
	simElementsSP1ph();
	simComponentSP1ph();

	simElementsDP1ph();
    simComponentDP1ph();

	simElementsEMT3ph();
	simComponentEMT3ph();
}
