/**
 * @file
 * @author Markus Mirz <mmirz@eonerc.rwth-aachen.de>
 * @copyright 2017-2019, Institute for Automation of Complex Power Systems, EONERC
 *
 * DPsim
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *********************************************************************************/

#include <DPsim.h>

using namespace DPsim;
using namespace CPS::DP;
using namespace CPS::DP::Ph1;

void DP_VS_CS_R4() {
	Real timeStep = 0.0001;
	Real finalTime = 0.1;
	String simName = "DP_VS_CS_R4";
	Logger::setLogDir("logs/"+simName);

	// Nodes
	auto n1 = Node::make("n1");
	auto n2 = Node::make("n2");
	auto n3 = Node::make("n3");

	// Components
	auto vs = VoltageSource::make("vs");
	vs->setParameters(10);
	auto r1 = Resistor::make("r_1");
	r1->setParameters(1);
	auto r2 = Resistor::make("r_2", Logger::Level::debug);
	r2->setParameters(1);
	auto r3 = Resistor::make("r_3");
	r3->setParameters(10);
	auto r4 = Resistor::make("r_4");
	r4->setParameters(5);
	auto cs = CurrentSource::make("cs");
	cs->setParameters(1);

	// Topology
	vs->connect(Node::List{ Node::GND, n1 });
	r1->connect(Node::List{ n1, n2 });
	r2->connect(Node::List{ n2, Node::GND });
	r3->connect(Node::List{ n2, n3 });
	r4->connect(Node::List{ n3, Node::GND });
	cs->connect(Node::List{ Node::GND, n3 });

	// Define system topology
	auto sys = SystemTopology(50,
		SystemNodeList{n1, n2, n3},
		SystemComponentList{vs, r1, r2, r3, r4, cs});

	// Logging
	auto logger = DataLogger::make(simName);
	logger->addAttribute("v1", n1->attribute("v"));
	logger->addAttribute("v2", n2->attribute("v"));
	logger->addAttribute("v3", n3->attribute("v"));
	logger->addAttribute("i12", r1->attribute("i_intf"));
	logger->addAttribute("i23", r3->attribute("i_intf"));

	Simulation sim(simName, sys, timeStep, finalTime);
	sim.setSystem(sys);
	sim.setTearingComponents(sys.mTearComponents);
	sim.setTimeStep(timeStep);
	sim.setFinalTime(finalTime);
	sim.addLogger(logger);

	sim.run();
}

void DP_VS_CS_R4_Diakoptics() {
	Real timeStep = 0.0001;
	Real finalTime = 0.1;
	String simName = "DP_VS_CS_R4_Diakoptics";
	Logger::setLogDir("logs/"+simName);

	// Nodes
	auto n1 = Node::make("n1");
	auto n2 = Node::make("n2");
	auto n3 = Node::make("n3");

	// Components
	auto vs = VoltageSource::make("vs");
	vs->setParameters(10);
	auto r1 = Resistor::make("r_1");
	r1->setParameters(1);
	auto r2 = Resistor::make("r_2", Logger::Level::debug);
	r2->setParameters(1);
	auto r3 = Resistor::make("r_3");
	r3->setParameters(10);
	auto r4 = Resistor::make("r_4");
	r4->setParameters(5);
	auto cs = CurrentSource::make("cs");
	cs->setParameters(1);

	// Topology
	vs->connect(Node::List{ Node::GND, n1 });
	r1->connect(Node::List{ n1, n2 });
	r2->connect(Node::List{ n2, Node::GND });
	r3->connect(Node::List{ n2, n3 });
	r4->connect(Node::List{ n3, Node::GND });
	cs->connect(Node::List{ Node::GND, n3 });

	// Define system topology
	auto sys = SystemTopology(50,
		SystemNodeList{n1, n2, n3},
		SystemComponentList{vs, r1, r2, r4, cs});
	sys.addTearComponent(r3);

	// Logging
	auto logger = DataLogger::make(simName);
	logger->addAttribute("v1", n1->attribute("v"));
	logger->addAttribute("v2", n2->attribute("v"));
	logger->addAttribute("v3", n3->attribute("v"));
	logger->addAttribute("i12", r1->attribute("i_intf"));
	logger->addAttribute("i23", r3->attribute("i_intf"));

	Simulation sim(simName, Logger::Level::info);
	sim.setSystem(sys);
	sim.setTearingComponents(sys.mTearComponents);
	sim.setTimeStep(timeStep);
	sim.setFinalTime(finalTime);
	sim.addLogger(logger);

	sim.run();
}

void DP_VS_R2L3() {
	Real timeStep = 0.0001;
	Real finalTime = 0.1;
	String simName = "DP_VS_R2L3";
	Logger::setLogDir("logs/"+simName);

	// Nodes
	auto n1 = Node::make("n1");
	auto n2 = Node::make("n2");
	auto n3 = Node::make("n3");
	auto n4 = Node::make("n4");

	// Components
	auto vs = VoltageSource::make("vs");
	vs->setParameters(10);
	auto r1 = Resistor::make("r_1");
	r1->setParameters(1);
	auto l1 = Inductor::make("l_1");
	l1->setParameters(0.02);
	auto l2 = Inductor::make("l_2");
	l2->setParameters(0.1);
	auto l3 = Inductor::make("l_3");
	l3->setParameters(0.05);
	auto r2 = Resistor::make("r_2");
	r2->setParameters(2);

	// Topology
	vs->connect(Node::List{ Node::GND, n1 });
	r1->connect(Node::List{ n1, n2 });
	l1->connect(Node::List{ n2, n3 });
	l2->connect(Node::List{ n3, Node::GND });
	l3->connect(Node::List{ n3, n4 });
	r2->connect(Node::List{ n4, Node::GND });

	auto sys = SystemTopology(50,
		SystemNodeList{n1, n2, n3, n4},
		SystemComponentList{vs, r1, l1, l2, l3, r2});

	// Logging
	auto logger = DataLogger::make(simName);
	logger->addAttribute("v1", n1->attribute("v"));
	logger->addAttribute("v2", n2->attribute("v"));
	logger->addAttribute("v3", n3->attribute("v"));
	logger->addAttribute("v4", n4->attribute("v"));
	logger->addAttribute("i12", r1->attribute("i_intf"));
	logger->addAttribute("i34", l3->attribute("i_intf"));

	Simulation sim(simName, sys, timeStep, finalTime);
	sim.setSystem(sys);
	sim.setTimeStep(timeStep);
	sim.setFinalTime(finalTime);
	sim.addLogger(logger);

	sim.run();
}

void DP_VS_R2L3_Diakoptics() {
	Real timeStep = 0.0001;
	Real finalTime = 0.1;
	String simName = "DP_VS_R2L3_Diakoptics";
	Logger::setLogDir("logs/"+simName);

	// Nodes
	auto n1 = Node::make("n1");
	auto n2 = Node::make("n2");
	auto n3 = Node::make("n3");
	auto n4 = Node::make("n4");

	// Components
	auto vs = VoltageSource::make("vs");
	vs->setParameters(10);
	auto r1 = Resistor::make("r_1");
	r1->setParameters(1);
	auto l1 = Inductor::make("l_1");
	l1->setParameters(0.02);
	auto l2 = Inductor::make("l_2");
	l2->setParameters(0.1);
	auto l3 = Inductor::make("l_3");
	l3->setParameters(0.05);
	auto r2 = Resistor::make("r_2");
	r2->setParameters(2);

	// Topology
	vs->connect(Node::List{ Node::GND, n1 });
	r1->connect(Node::List{ n1, n2 });
	l1->connect(Node::List{ n2, n3 });
	l2->connect(Node::List{ n3, Node::GND });
	l3->connect(Node::List{ n3, n4 });
	r2->connect(Node::List{ n4, Node::GND });

	auto sys = SystemTopology(50,
		SystemNodeList{n1, n2, n3, n4},
		SystemComponentList{vs, r1, l2, l3, r2});
	sys.addTearComponent(l1);

	// Logging
	auto logger = DataLogger::make(simName);
	logger->addAttribute("v1", n1->attribute("v"));
	logger->addAttribute("v2", n2->attribute("v"));
	logger->addAttribute("v3", n3->attribute("v"));
	logger->addAttribute("v4", n4->attribute("v"));
	logger->addAttribute("i12", r1->attribute("i_intf"));
	logger->addAttribute("i34", l3->attribute("i_intf"));

	Simulation sim(simName, sys, timeStep, finalTime);
	sim.setSystem(sys);
	sim.setTearingComponents(sys.mTearComponents);
	sim.setTimeStep(timeStep);
	sim.setFinalTime(finalTime);
	sim.addLogger(logger);

	sim.run();
}

int main(int argc, char* argv[]) {
	DP_VS_CS_R4();
	DP_VS_CS_R4_Diakoptics();
	DP_VS_R2L3();
	DP_VS_R2L3_Diakoptics();
}
