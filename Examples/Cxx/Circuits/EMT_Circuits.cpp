/**
 * @file
 * @author Markus Mirz <mmirz@eonerc.rwth-aachen.de>
 * @copyright 2017-2018, Institute for Automation of Complex Power Systems, EONERC
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
using namespace CPS::EMT;
using namespace CPS::EMT::Ph1;

void EMT_VS_R1();
void EMT_CS_R1();
void EMT_CS_R2CL();
void EMT_VS_CS_R4_AC();
void EMT_VS_CS_R4_DC();
void EMT_VS_R2L3();
void EMT_VS_RC1();

int main(int argc, char* argv[]) {
	EMT_VS_R1();
	EMT_CS_R1();
	EMT_CS_R2CL();
	EMT_VS_CS_R4_AC();
	EMT_VS_CS_R4_DC();
	EMT_VS_R2L3();
	EMT_VS_RC1();
}

void EMT_CS_R1() {
	Real timeStep = 0.0001;
	Real finalTime = 0.1;
	String simName = "EMT_CS_R1";
	Logger::setLogDir("logs/"+simName);

	// Nodes
	auto n1 = Node::make("n1");

	// Components
	auto cs = CurrentSource::make("cs");
	cs->setParameters(Complex(10, 0), 50);
	cs->connect(Node::List{ Node::GND, n1 });
	auto r1 = Resistor::make("r_1");
	r1->setParameters(1);
	r1->connect({ Node::GND, n1 });

	auto sys = SystemTopology(50, SystemNodeList{n1}, SystemComponentList{cs, r1});

	// Logging
	auto logger = DataLogger::make(simName);
	logger->addAttribute("v1", n1->attribute("v"));

	Simulation sim(simName, sys, timeStep, finalTime, Domain::EMT);
	sim.addLogger(logger);

	sim.run();
}

void EMT_VS_R1() {
	Real timeStep = 0.0001;
	Real finalTime = 0.1;
	String simName = "EMT_VS_R1";
	Logger::setLogDir("logs/"+simName);

	// Nodes
	auto n1 = Node::make("n1");

	// Components
	auto vs = VoltageSource::make("v_1");
	vs->setParameters(Complex(10, 0), 50);
	auto r = Resistor::make("r_1");
	r->setParameters(1);

	// Topology
	vs->connect({Node::GND, n1});
	r->connect({n1, Node::GND});

	auto sys = SystemTopology(50, SystemNodeList{n1}, SystemComponentList{vs, r});

	// Logging
	auto logger = DataLogger::make(simName);
	logger->addAttribute("v1", n1->attribute("v"));

	Simulation sim(simName, sys, timeStep, finalTime, Domain::EMT);
	sim.addLogger(logger);

	sim.run();
}

void EMT_CS_R2CL() {
	Real timeStep = 0.0001;
	Real finalTime = 0.1;
	String simName = "EMT_CS_R2CL";
	Logger::setLogDir("logs/"+simName);

	// Nodes
	auto n1 = Node::make("n1");
	auto n2 = Node::make("n2");

	// Components
	auto cs = CurrentSource::make("cs");
	cs->setParameters(10, 50);
	cs->connect(Node::List{ Node::GND, n1 });
	auto r1 = Resistor::make("r_1");
	r1->setParameters(1);
	r1->connect(Node::List{ n1, Node::GND });
	auto c1 = Capacitor::make("c_1");
	c1->setParameters(0.001);
	c1->connect(Node::List{ n1, n2 });
	auto l1 = Inductor::make("l_1");
	l1->setParameters(0.001);
	l1->connect(Node::List{ n2, Node::GND });
	auto r2 = Resistor::make("r_2");
	r2->setParameters(1);
	r2->connect(Node::List{ n2, Node::GND });

	auto sys = SystemTopology(50, SystemNodeList{n1, n2}, SystemComponentList{cs, r1, c1, l1, r2});

	// Logging
	auto logger = DataLogger::make(simName);
	logger->addAttribute("v1", n1->attribute("v"));
	logger->addAttribute("v2", n2->attribute("v"));
	logger->addAttribute("i12", cs->attribute("i_intf"));
	logger->addAttribute("i34", c1->attribute("i_intf"));

	Simulation sim(simName, sys, timeStep, finalTime, Domain::EMT);
	sim.addLogger(logger);

	sim.run();
}

void EMT_VS_CS_R4_AC() {
	Real timeStep = 0.0001;
	Real finalTime = 0.1;
	String simName = "EMT_VS_CS_R4_AC";
	Logger::setLogDir("logs/"+simName);

	// Nodes
	auto n1 = Node::make("n1");
	auto n2 = Node::make("n2");
	auto n3 = Node::make("n3");

	// Components
	auto vs = VoltageSource::make("vs");
	vs->setParameters(10, 50);
	auto r1 = Resistor::make("r_1");
	r1->setParameters(1);
	auto r2 = Resistor::make("r_2");
	r2->setParameters(1);
	auto r3 = Resistor::make("r_3");
	r3->setParameters(10);
	auto r4 = Resistor::make("r_4");
	r4->setParameters(5);
	auto cs = CurrentSource::make("cs");
	cs->setParameters(1, 50);

	// Topology
	vs->connect(Node::List{ Node::GND, n1 });
	r1->connect(Node::List{ n1, n2 });
	r2->connect(Node::List{ n2, Node::GND });
	r3->connect(Node::List{ n2, n3 });
	r4->connect(Node::List{ n3, Node::GND });
	cs->connect(Node::List{ Node::GND, n3 });

	auto sys = SystemTopology(50, SystemNodeList{n1, n2, n3}, SystemComponentList{vs, r1, r2, r3, r4, cs});

	// Logging
	auto logger = DataLogger::make(simName);
	logger->addAttribute("v1", n1->attribute("v"));
	logger->addAttribute("v2", n2->attribute("v"));
	logger->addAttribute("v3", n3->attribute("v"));
	logger->addAttribute("i12", r1->attribute("i_intf"));
	logger->addAttribute("i23", r3->attribute("i_intf"));

	Simulation sim(simName, sys, timeStep, finalTime, Domain::EMT);
	sim.addLogger(logger);

	sim.run();
}

void EMT_VS_CS_R4_DC() {
	Real timeStep = 0.001;
	Real finalTime = 0.1;
	String simName = "EMT_VS_CS_R4_DC";
	Logger::setLogDir("logs/"+simName);

	// Nodes
	auto n1 = Node::make("n1");
	auto n2 = Node::make("n2");
	auto n3 = Node::make("n3");

	// Components
	auto vs = VoltageSource::make("vs");
	vs->setParameters(10);
	auto r1 = Resistor::make("r_1", Logger::Level::debug);
	r1->setParameters(1);
	auto r2 = Resistor::make("r_2");
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

	auto sys = SystemTopology(50, SystemNodeList{n1, n2, n3}, SystemComponentList{vs, r1, r2, r3, r4, cs});

	// Logging
	auto logger = DataLogger::make(simName);
	logger->addAttribute("v1", n1->attribute("v"));
	logger->addAttribute("v2", n2->attribute("v"));
	logger->addAttribute("v3", n3->attribute("v"));

	Simulation sim(simName, sys, timeStep, finalTime, Domain::EMT);
	sim.addLogger(logger);

	sim.run();
}

void EMT_VS_R2L3() {
	Real timeStep = 0.00001;
	Real finalTime = 0.1;
	String simName = "EMT_VS_R2L3";
	Logger::setLogDir("logs/"+simName);

	// Nodes
	auto n1 = Node::make("n1");
	auto n2 = Node::make("n2");
	auto n3 = Node::make("n3");
	auto n4 = Node::make("n4");

	// Components
	auto vs = VoltageSource::make("vs", Logger::Level::debug);
	vs->setParameters(10, 50);
	auto r1 = Resistor::make("r_1", Logger::Level::debug);
	r1->setParameters(1);
	auto l1 = Inductor::make("l_1", Logger::Level::debug);
	l1->setParameters(0.02);
	auto l2 = Inductor::make("l_2", Logger::Level::debug);
	l2->setParameters(0.1);
	auto l3 = Inductor::make("l_3", Logger::Level::debug);
	l3->setParameters(0.05);
	auto r2 = Resistor::make("r_2", Logger::Level::debug);
	r2->setParameters(2);

	// Topology
	vs->connect(Node::List{ Node::GND, n1 });
	r1->connect(Node::List{ n1, n2 });
	l1->connect(Node::List{ n2, n3 });
	l2->connect(Node::List{ n3, Node::GND });
	l3->connect(Node::List{ n3, n4 });
	r2->connect(Node::List{ n4, Node::GND });

	auto sys = SystemTopology(50, SystemNodeList{n1, n2, n3, n4}, SystemComponentList{vs, r1, l1, l2, l3, r2});

	// Logging
	auto logger = DataLogger::make(simName);
	logger->addAttribute("v1", n1->attribute("v"));
	logger->addAttribute("v2", n2->attribute("v"));
	logger->addAttribute("v3", n3->attribute("v"));
	logger->addAttribute("v4", n4->attribute("v"));
	logger->addAttribute("i12", r1->attribute("i_intf"));
	logger->addAttribute("i34", l3->attribute("i_intf"));

	Simulation sim(simName, sys, timeStep, finalTime, Domain::EMT);
	sim.addLogger(logger);

	sim.run();
}

void EMT_VS_RC1() {
	Real timeStep = 0.0001;
	Real finalTime = 0.1;
	String simName = "EMT_VS_RC1";
	Logger::setLogDir("logs/"+simName);

	// Nodes
	auto n1 = Node::make("n1");
	auto n2 = Node::make("n2");

	// Components
	auto vs = VoltageSource::make("vs");
	vs->setParameters(Complex(10, 0), 50);
	auto r1 = Resistor::make("r_1");
	r1->setParameters(1);
	auto c1 = Capacitor::make("c_1");
	c1->setParameters(0.001);

	// Topology
	vs->connect({ Node::GND, n1 });
	r1->connect({ n1, n2 });
	c1->connect({ n2, Node::GND });

	auto sys = SystemTopology(50, SystemNodeList{n1, n2}, SystemComponentList{vs, r1, c1});

	// Logging
	auto logger = DataLogger::make(simName);
	logger->addAttribute("v1", n1->attribute("v"));
	logger->addAttribute("v2", n2->attribute("v"));
	logger->addAttribute("i12", r1->attribute("i_intf"));

	Simulation sim(simName, sys, timeStep, finalTime, Domain::EMT);
	sim.addLogger(logger);

	sim.run();
}
