/* Copyright 2017-2020 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <DPsim.h>

using namespace DPsim;
using namespace CPS;

void vsEMT3ph() {
	Real timeStep = 0.00005;
	Real finalTime = 0.1;
	String simName = "EMT_VoltageSource";
	Logger::setLogDir("logs/"+simName);

	// Nodes
	auto n1 = SimNode<Real>::make("n1", PhaseType::ABC);

	// Components
	auto vs = EMT::Ph3::VoltageSource::make("vs1", Logger::Level::debug);
	vs->setParameters(CPS::Math::singlePhaseVariableToThreePhase(CPS::Math::polar(100000, 0)), 50);

	auto res = EMT::Ph3::Resistor::make("R1");
	res->setParameters(CPS::Math::singlePhaseParameterToThreePhase(100));

	// Topology
	vs->connect({ SimNode<Real>::GND, n1 });
	res->connect({n1, SimNode<Real>::GND});

	auto sys = SystemTopology(50,
		SystemNodeList{n1},
		SystemComponentList{vs, res});

	// Logging
	auto logger = DataLogger::make(simName);
	logger->addAttribute("v", vs->attribute("v_intf"));
	logger->addAttribute("i", vs->attribute("i_intf"));

	// Simulation
	Simulation sim(simName);
	sim.setSystem(sys);
	sim.setTimeStep(timeStep);
	sim.setFinalTime(finalTime);
	sim.setDomain(Domain::EMT);
	sim.addLogger(logger);
	sim.run();
}

void vsControlledEMT3ph() {
	Real timeStep = 0.00005;
	Real finalTime = 0.1;
	String simName = "EMT_VoltageSource_Controlled";
	Logger::setLogDir("logs/"+simName);

	// Nodes
	auto n1 = SimNode<Real>::make("n1", PhaseType::ABC);

	// Components
	auto vs = EMT::Ph3::ControlledVoltageSource::make("vs1", Logger::Level::debug);

	auto res = EMT::Ph3::Resistor::make("R1");
	res->setParameters(CPS::Math::singlePhaseParameterToThreePhase(100));

	// Topology
	vs->connect({ SimNode<Real>::GND, n1 });
	res->connect({n1, SimNode<Real>::GND});

	auto sys = SystemTopology(50,
		SystemNodeList{n1},
		SystemComponentList{vs, res});

	// Logging
	auto logger = DataLogger::make(simName);
	logger->addAttribute("v", vs->attribute("v_intf"));
	logger->addAttribute("i", vs->attribute("i_intf"));

	// Simulation
	Simulation sim(simName);
	sim.setSystem(sys);
	sim.setTimeStep(timeStep);
	sim.setFinalTime(finalTime);
	sim.setDomain(Domain::EMT);
	sim.addLogger(logger);
	sim.run();
}

int main(int argc, char* argv[]) {
	vsEMT3ph();
	vsControlledEMT3ph();
}
