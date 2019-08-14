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

void DP_CS_R1();
void DP_VS_R1();
void DP_CS_R2CL();
void DP_VS_CS_R4();
void DP_VS_R2L3();
void DP_VS_RC1();
void DP_VS_RL2();

int main(int argc, char* argv[]) {
	DP_CS_R1();
	DP_VS_R1();
	DP_CS_R2CL();
	DP_VS_CS_R4();
	DP_VS_R2L3();
	DP_VS_RC1();
	DP_VS_RL2();

	return 0;
}

void DP_CS_R1() {
	Real timeStep = 0.0001;
	Real finalTime = 0.1;
	String simName = "DP_CS_R1";
	Logger::setLogDir("logs/"+simName);

	// Nodes
	auto n1 = Node::make("n1");

	// Components
	auto cs = CurrentSource::make("cs");
	cs->setParameters(Complex(10, 0));
	auto r1 = Resistor::make("r_1");
	r1->setParameters(1);

	// Topology
	cs->connect({ Node::GND, n1 });
	r1->connect({ Node::GND, n1 });

	auto sys = SystemTopology(50, SystemNodeList{n1}, SystemComponentList{cs, r1});

	// Logging
	auto logger = DataLogger::make(simName);
	logger->addAttribute("v1", n1->attribute("v"));
	logger->addAttribute("i10", r1->attribute("i_intf"));

	Simulation sim(simName, sys, timeStep, finalTime, Domain::DP, Solver::Type::MNA, Logger::Level::debug);
	sim.addLogger(logger);

	sim.run();
}

void DP_VS_R1() {
	Real timeStep = 0.0001;
	Real finalTime = 0.1;
	String simName = "DP_VS_R1";
	Logger::setLogDir("logs/"+simName);

	// Nodes
	auto n1 = Node::make("n1");

	// Components
	auto vs = VoltageSource::make("v_1");
	vs->setParameters(Complex(10, 0));
	auto r = Resistor::make("r_1");
	r->setParameters(1);

	// Topology
	vs->connect({Node::GND, n1});
	r->connect({n1, Node::GND});

	auto sys = SystemTopology(50, SystemNodeList{n1}, SystemComponentList{vs, r});

	// Logging
	auto logger = DataLogger::make(simName);
	logger->addAttribute("v1", n1->attribute("v"));

	Simulation sim(simName, sys, timeStep, finalTime);
	sim.addLogger(logger);

	sim.run();
}

void DP_CS_R2CL() {
	Real timeStep = 0.0001;
	Real finalTime = 0.1;
	String simName = "DP_CS_R2CL";
	Logger::setLogDir("logs/"+simName);

	// Nodes
	auto n1 = Node::make("n1");
	auto n2 = Node::make("n2");

	// Components
	auto cs = CurrentSource::make("cs");
	cs->setParameters(10);
	auto r1 = Resistor::make("r_1");
	r1->setParameters(1);
	auto c1 = Capacitor::make("c_1");
	c1->setParameters(0.001);
	auto l1 = Inductor::make("l_1");
	l1->setParameters(0.001);
	auto r2 = Resistor::make("r_2");
	r2->setParameters(1);

	// Topology
	cs->connect({ Node::GND, n1 });
	r1->connect({ n1, Node::GND });
	c1->connect({ n1, n2 });
	l1->connect({ n2, Node::GND });
	r2->connect({ n2, Node::GND });

	auto sys = SystemTopology(50, SystemNodeList{n1, n2}, SystemComponentList{cs, r1, c1, l1, r2});

	// Logging
	auto logger = DataLogger::make(simName);
	logger->addAttribute("v1", n1->attribute("v"));
	logger->addAttribute("v2", n2->attribute("v"));
	logger->addAttribute("i12", cs->attribute("i_intf"));
	logger->addAttribute("i34", c1->attribute("i_intf"));

	Simulation sim(simName, sys, timeStep, finalTime);
	sim.addLogger(logger);

	sim.run();
}

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
	auto sys = SystemTopology(50, SystemNodeList{n1, n2, n3}, SystemComponentList{vs, r1, r2, r3, r4, cs});

	// Logging
	auto logger = DataLogger::make(simName);
	logger->addAttribute("v1", n1->attribute("v"));
	logger->addAttribute("v2", n2->attribute("v"));
	logger->addAttribute("v3", n3->attribute("v"));
	logger->addAttribute("i12", r1->attribute("i_intf"));
	logger->addAttribute("i23", r3->attribute("i_intf"));

	Simulation sim(simName, sys, timeStep, finalTime);
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

	auto sys = SystemTopology(50, SystemNodeList{n1, n2, n3, n4}, SystemComponentList{vs, r1, l1, l2, l3, r2});

	// Logging
	auto logger = DataLogger::make(simName);
	logger->addAttribute("v1", n1->attribute("v"));
	logger->addAttribute("v2", n2->attribute("v"));
	logger->addAttribute("v3", n3->attribute("v"));
	logger->addAttribute("v4", n4->attribute("v"));
	logger->addAttribute("i12", r1->attribute("i_intf"));
	logger->addAttribute("i34", l3->attribute("i_intf"));

	Simulation sim(simName, sys, timeStep, finalTime);
	sim.addLogger(logger);

	sim.run();
}

void DP_VS_RC1() {
	Real timeStep = 0.0001;
	Real finalTime = 0.1;
	String simName = "DP_VS_RC1";
	Logger::setLogDir("logs/"+simName);

	// Nodes
	auto n1 = Node::make("n1");
	auto n2 = Node::make("n2");

	// Components
	auto vs = VoltageSource::make("vs", Logger::Level::debug);
	vs->setParameters(Complex(10, 0));
	auto r1 = Resistor::make("r_1", Logger::Level::debug);
	r1->setParameters(1);
	auto c1 = Capacitor::make("c_1", Logger::Level::debug);
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

	Simulation sim(simName, sys, timeStep, finalTime);
	sim.addLogger(logger);

	sim.run();
}

void DP_VS_RL2() {
	Real timeStep = 0.0001;
	Real finalTime = 0.1;
	String simName = "DP_VS_RL2";
	Logger::setLogDir("logs/"+simName);

	// Nodes
	auto n1 = Node::make("n1");
	auto n2 = Node::make("n2");
	auto n3 = Node::make("n3");

	// Components
	auto vs = VoltageSource::make("v_s");
	vs->setParameters(Complex(1000, 0));
	auto rl = Resistor::make("r_line");
	rl->setParameters(1);
	auto ll = Inductor::make("l_line");
	ll->setParameters(0.01);
	auto rL = Resistor::make("r_load");
	rL->setParameters(100);

	// Topology
	vs->connect({ Node::GND, n1 });
	rl->connect({ n1, n2 });
	ll->connect({ n2, n3 });
	rL->connect({ Node::GND, n3 });

	// Define system topology
	auto sys = SystemTopology(50, SystemNodeList{n1, n2, n3}, SystemComponentList{vs, rl, ll, rL});

	// Logger
	auto logger = DataLogger::make(simName);
	logger->addAttribute("v1", n1->attribute("v"));
	logger->addAttribute("v2", n2->attribute("v"));
	logger->addAttribute("i12", rL->attribute("i_intf"));

	Simulation sim(simName, sys, timeStep, finalTime, Domain::DP, Solver::Type::MNA, Logger::Level::debug);
	sim.addLogger(logger);

	sim.run();
}
