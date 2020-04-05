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
using namespace CPS::DP;

void DP_CS_R1() {
	Real timeStep = 0.0001;
	Real finalTime = 0.1;
	String simName = "DP_CS_R1";
	Logger::setLogDir("logs/"+simName);

	// Nodes
	auto n1 = SimNode::make("n1");

	// Components
	auto cs = Ph1::CurrentSource::make("cs");
	cs->setParameters(Complex(10, 0));
	auto r1 = Ph1::Resistor::make("r_1");
	r1->setParameters(1);

	// Topology
	cs->connect({ SimNode::GND, n1 });
	r1->connect({ SimNode::GND, n1 });

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
	auto n1 = SimNode::make("n1");

	// Components
	auto vs = Ph1::VoltageSource::make("v_1");
	vs->setParameters(Complex(10, 0));
	auto r = Ph1::Resistor::make("r_1");
	r->setParameters(1);

	// Topology
	vs->connect({SimNode::GND, n1});
	r->connect({n1, SimNode::GND});

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
	auto n1 = SimNode::make("n1");
	auto n2 = SimNode::make("n2");

	// Components
	auto cs = Ph1::CurrentSource::make("cs");
	cs->setParameters(10);
	auto r1 = Ph1::Resistor::make("r_1");
	r1->setParameters(1);
	auto c1 = Ph1::Capacitor::make("c_1");
	c1->setParameters(0.001);
	auto l1 = Ph1::Inductor::make("l_1");
	l1->setParameters(0.001);
	auto r2 = Ph1::Resistor::make("r_2");
	r2->setParameters(1);

	// Topology
	cs->connect({ SimNode::GND, n1 });
	r1->connect({ n1, SimNode::GND });
	c1->connect({ n1, n2 });
	l1->connect({ n2, SimNode::GND });
	r2->connect({ n2, SimNode::GND });

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
	auto n1 = SimNode::make("n1");
	auto n2 = SimNode::make("n2");
	auto n3 = SimNode::make("n3");

	// Components
	auto vs = Ph1::VoltageSource::make("vs");
	vs->setParameters(10);
	auto r1 = Ph1::Resistor::make("r_1");
	r1->setParameters(1);
	auto r2 = Ph1::Resistor::make("r_2", Logger::Level::debug);
	r2->setParameters(1);
	auto r3 = Ph1::Resistor::make("r_3");
	r3->setParameters(10);
	auto r4 = Ph1::Resistor::make("r_4");
	r4->setParameters(5);
	auto cs = Ph1::CurrentSource::make("cs");
	cs->setParameters(1);

	// Topology
	vs->connect(SimNode::List{ SimNode::GND, n1 });
	r1->connect(SimNode::List{ n1, n2 });
	r2->connect(SimNode::List{ n2, SimNode::GND });
	r3->connect(SimNode::List{ n2, n3 });
	r4->connect(SimNode::List{ n3, SimNode::GND });
	cs->connect(SimNode::List{ SimNode::GND, n3 });

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
	auto n1 = SimNode::make("n1");
	auto n2 = SimNode::make("n2");
	auto n3 = SimNode::make("n3");
	auto n4 = SimNode::make("n4");

	// Components
	auto vs = Ph1::VoltageSource::make("vs");
	vs->setParameters(10);
	auto r1 = Ph1::Resistor::make("r_1");
	r1->setParameters(1);
	auto l1 = Ph1::Inductor::make("l_1");
	l1->setParameters(0.02);
	auto l2 = Ph1::Inductor::make("l_2");
	l2->setParameters(0.1);
	auto l3 = Ph1::Inductor::make("l_3");
	l3->setParameters(0.05);
	auto r2 = Ph1::Resistor::make("r_2");
	r2->setParameters(2);

	// Topology
	vs->connect(SimNode::List{ SimNode::GND, n1 });
	r1->connect(SimNode::List{ n1, n2 });
	l1->connect(SimNode::List{ n2, n3 });
	l2->connect(SimNode::List{ n3, SimNode::GND });
	l3->connect(SimNode::List{ n3, n4 });
	r2->connect(SimNode::List{ n4, SimNode::GND });

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
	auto n1 = SimNode::make("n1");
	auto n2 = SimNode::make("n2");

	// Components
	auto vs = Ph1::VoltageSource::make("vs", Logger::Level::debug);
	vs->setParameters(Complex(10, 0));
	auto r1 = Ph1::Resistor::make("r_1", Logger::Level::debug);
	r1->setParameters(1);
	auto c1 = Ph1::Capacitor::make("c_1", Logger::Level::debug);
	c1->setParameters(0.001);

	// Topology
	vs->connect({ SimNode::GND, n1 });
	r1->connect({ n1, n2 });
	c1->connect({ n2, SimNode::GND });

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
	auto n1 = SimNode::make("n1");
	auto n2 = SimNode::make("n2");
	auto n3 = SimNode::make("n3");

	// Components
	auto vs = Ph1::VoltageSource::make("v_s");
	vs->setParameters(Complex(1000, 0));
	auto rl = Ph1::Resistor::make("r_line");
	rl->setParameters(1);
	auto ll = Ph1::Inductor::make("l_line");
	ll->setParameters(0.01);
	auto rL = Ph1::Resistor::make("r_load");
	rL->setParameters(100);

	// Topology
	vs->connect({ SimNode::GND, n1 });
	rl->connect({ n1, n2 });
	ll->connect({ n2, n3 });
	rL->connect({ SimNode::GND, n3 });

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

void DP_Ph3_VS_R2L3() {
	// Define simulation scenario
	Real timeStep = 0.0001;
	Real finalTime = 0.1;
	String simName = "DP_Ph3_VS_R2L3";
	Logger::setLogDir("logs/" + simName);

	// Nodes
	auto n1 = SimNode::make("n1", PhaseType::ABC);
	auto n2 = SimNode::make("n2", PhaseType::ABC);
	auto n3 = SimNode::make("n3", PhaseType::ABC);
	auto n4 = SimNode::make("n4", PhaseType::ABC);

	// Components
	auto vs = Ph3::VoltageSource::make("vs");
	vs->setParameters(10);
	auto r1 = Ph3::Resistor::make("r_1");
	Matrix r1_param = Matrix::Zero(3, 3);
	r1_param <<
		1., 0, 0,
		0, 1., 0,
		0, 0, 1.;
	r1->setParameters(r1_param);
	auto l1 = Ph3::Inductor::make("l_1");
	Matrix l_param = Matrix::Zero(3, 3);
	l_param <<
		1., 0, 0,
		0, 1., 0,
		0, 0, 1.;
	l1->setParameters(0.02 * l_param);
	auto l2 = Ph3::Inductor::make("l_2");
	l2->setParameters(0.1 * l_param);
	auto l3 = Ph3::Inductor::make("l_3");
	l3->setParameters(0.05 * l_param);
	auto r2 = Ph3::Resistor::make("r_2");
	r2->setParameters(2 * r1_param);

	// Topology
	vs->connect(SimNode::List{ SimNode::GND, n1 });
	r1->connect(SimNode::List{ n1, n2 });
	l1->connect(SimNode::List{ n2, n3 });
	l2->connect(SimNode::List{ n3, SimNode::GND });
	l3->connect(SimNode::List{ n3, n4 });
	r2->connect(SimNode::List{ n4, SimNode::GND });

	auto sys = SystemTopology(50, SystemNodeList{n1, n2, n3, n4}, SystemComponentList{vs, r1, l1, l2, l3, r2});

	// Logging
	auto logger = DataLogger::make(simName);
	logger->addAttribute("v1", n1->attribute("v"));
	logger->addAttribute("v2", n2->attribute("v"));
	logger->addAttribute("v3", n3->attribute("v"));
	logger->addAttribute("v4", n4->attribute("v"));
	logger->addAttribute("i12", r1->attribute("i_intf"));
	logger->addAttribute("i34", l3->attribute("i_intf"));


	Simulation sim(simName, Logger::Level::info);
	sim.setSystem(sys);
	sim.addLogger(logger);
	sim.setDomain(Domain::DP);
	sim.setTimeStep(timeStep);
	sim.setFinalTime(finalTime);
	sim.run();
}

void DP_Ph3_VS_RC1() {
	// Define simulation scenario
	Real timeStep = 0.0001;
	Real finalTime = 0.1;
	String simName = "DP_Ph3_VS_RC1";
	Logger::setLogDir("logs/" + simName);

	// Nodes
	auto n1 = SimNode::make("n1", PhaseType::ABC);
	auto n2 = SimNode::make("n2", PhaseType::ABC);

	// Components
	auto vs = Ph3::VoltageSource::make("vs");
	vs->setParameters(Complex(10, 0));
	auto r1 = Ph3::Resistor::make("r_1");
	Matrix r1_param = Matrix::Zero(3, 3);
	r1_param <<
		1., 0, 0,
		0, 1., 0,
		0, 0, 1.;
	r1->setParameters(r1_param);
	auto c1 = Ph3::Capacitor::make("c_1");
	Matrix c_param = Matrix::Zero(3, 3);
	c_param <<
		0.001, 0, 0,
		0, 0.001, 0,
		0, 0, 0.001;
	c1->setParameters(c_param);

	// Topology
	vs->connect({ SimNode::GND, n1 });
	r1->connect({ n1, n2 });
	c1->connect({ n2, SimNode::GND });

	auto sys = SystemTopology(50, SystemNodeList{n1, n2}, SystemComponentList{vs, r1, c1});

	// Logging
	auto logger = DataLogger::make(simName);
	logger->addAttribute("v1", n1->attribute("v"));
	logger->addAttribute("v2", n2->attribute("v"));
	logger->addAttribute("i12", r1->attribute("i_intf"));

	Simulation sim(simName, Logger::Level::info);
	sim.setSystem(sys);
	sim.addLogger(logger);
	sim.setDomain(Domain::DP);
	sim.setTimeStep(timeStep);
	sim.setFinalTime(finalTime);
	sim.run();
}

int main(int argc, char* argv[]) {
	DP_CS_R1();
	DP_VS_R1();
	DP_CS_R2CL();
	DP_VS_CS_R4();
	DP_VS_R2L3();
	DP_VS_RC1();
	DP_VS_RL2();

	DP_Ph3_VS_R2L3();
	DP_Ph3_VS_RC1();
}
