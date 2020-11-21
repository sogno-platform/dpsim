/* Copyright 2017-2020 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 * DPsim
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
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
