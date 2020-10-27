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
using namespace CPS::CIM;

void voltageSourceResistorEMT3ph() {
	Real timeStep = 0.00005;
	Real finalTime = 1;
	String simName = "EMT_VoltageSource_Resistor";
	Logger::setLogDir("logs/"+simName);

	// Nodes
	auto n1 = SimNode<Real>::make("n1", PhaseType::ABC);

	// Components
	auto vs = EMT::Ph3::VoltageSource::make("vs1");
	vs->setParameters(CPS::Math::polar(100000, -PI/2.), 50);

	auto res = EMT::Ph3::Resistor::make("R1");
	res->setParameters(Reader::singlePhaseParameterToThreePhase(100));

	// Topology
	vs->connect({ SimNode<Real>::GND, n1 });
	res->connect({n1, SimNode<Real>::GND});

	auto sys = SystemTopology(50,
		SystemNodeList{n1},
		SystemComponentList{vs, res});

	// Logging
	auto logger = DataLogger::make(simName);
	logger->addAttribute("v1", n1->attribute("v"));
	logger->addAttribute("iR", res->attribute("i_intf"));

	// Simulation
	Simulation sim(simName);
	sim.setSystem(sys);
	sim.setTimeStep(timeStep);
	sim.setFinalTime(finalTime);
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
	vs->setParameters(CPS::Math::polar(100000, -PI/2.));

	auto res = DP::Ph1::Resistor::make("R1");
	res->setParameters(100);

	// Topology
	vs->connect({ SimNode<Complex>::GND, n1 });
	res->connect({n1, SimNode<Complex>::GND});

	auto sys = SystemTopology(50,
		SystemNodeList{n1},
		SystemComponentList{vs, res});

	// Logging
	auto logger = DataLogger::make(simName);
	logger->addAttribute("v1", n1->attribute("v"));
	logger->addAttribute("iR", res->attribute("i_intf"));

	// Simulation
	Simulation sim(simName);
	sim.setSystem(sys);
	sim.setTimeStep(timeStep);
	sim.setFinalTime(finalTime);
	sim.setDomain(Domain::DP);
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
	vs->setParameters(CPS::Math::polar(100000, 0), 50);

	auto res = EMT::Ph3::Resistor::make("R1");
	res->setParameters(Reader::singlePhaseParameterToThreePhase(5));

	auto ind = EMT::Ph3::Inductor::make("L1", Logger::Level::debug);
	ind->setParameters(Reader::singlePhaseParameterToThreePhase(0.5));

	// Topology
	vs->connect({ SimNode<Real>::GND, n1 });
	res->connect({n1, n2});
	ind->connect({n2, SimNode<Real>::GND});

	auto sys = SystemTopology(50,
		SystemNodeList{n1, n2},
		SystemComponentList{vs, res, ind});

	// Logging
	auto logger = DataLogger::make(simName);
	logger->addAttribute("v1", n1->attribute("v"));
	logger->addAttribute("v2", n2->attribute("v"));
	logger->addAttribute("iR", res->attribute("i_intf"));
	logger->addAttribute("iL", ind->attribute("i_intf"));

	// Simulation
	Simulation sim(simName);
	sim.setSystem(sys);
	sim.setTimeStep(timeStep);
	sim.setFinalTime(finalTime);
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

	auto res = DP::Ph1::Resistor::make("R1");
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
	logger->addAttribute("v1", n1->attribute("v"));
	logger->addAttribute("v2", n2->attribute("v"));
	logger->addAttribute("iR", res->attribute("i_intf"));
	logger->addAttribute("iL", ind->attribute("i_intf"));

	// Simulation
	Simulation sim(simName);
	sim.setSystem(sys);
	sim.setTimeStep(timeStep);
	sim.setFinalTime(finalTime);
	sim.setDomain(Domain::DP);
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
	vs->setParameters(CPS::Math::polar(100000, 0), 50);

	auto res = EMT::Ph3::Resistor::make("R1");
	res->setParameters(Reader::singlePhaseParameterToThreePhase(5));

	auto cap = EMT::Ph3::Capacitor::make("C1", Logger::Level::debug);
	cap->setParameters(Reader::singlePhaseParameterToThreePhase(10e-3));

	// Topology
	vs->connect({ SimNode<Real>::GND, n1 });
	res->connect({n1, n2});
	cap->connect({n2, SimNode<Real>::GND});

	auto sys = SystemTopology(50,
		SystemNodeList{n1, n2},
		SystemComponentList{vs, res, cap});

	// Logging
	auto logger = DataLogger::make(simName);
	logger->addAttribute("v1", n1->attribute("v"));
	logger->addAttribute("v2", n2->attribute("v"));
	logger->addAttribute("iR", res->attribute("i_intf"));
	logger->addAttribute("iC", cap->attribute("i_intf"));

	// Simulation
	Simulation sim(simName);
	sim.setSystem(sys);
	sim.setTimeStep(timeStep);
	sim.setFinalTime(finalTime);
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

	auto res = DP::Ph1::Resistor::make("R1");
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
	logger->addAttribute("v1", n1->attribute("v"));
	logger->addAttribute("v2", n2->attribute("v"));
	logger->addAttribute("iR", res->attribute("i_intf"));
	logger->addAttribute("iC", cap->attribute("i_intf"));

	// Simulation
	Simulation sim(simName);
	sim.setSystem(sys);
	sim.setTimeStep(timeStep);
	sim.setFinalTime(finalTime);
	sim.setDomain(Domain::DP);
	sim.addLogger(logger);
	sim.run();
}

int main(int argc, char* argv[]) {
	voltageSourceResistorEMT3ph();
	voltageSourceResistorDP1ph();

	voltageSourceInductorEMT3ph();
	voltageSourceInductorDP1ph();

	voltageSourceCapacitorEMT3ph();
	voltageSourceCapacitorDP1ph();
}
