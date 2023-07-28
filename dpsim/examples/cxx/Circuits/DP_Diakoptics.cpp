/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
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
	auto n1 = SimNode::make("n1");
	auto n2 = SimNode::make("n2");
	auto n3 = SimNode::make("n3");

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
	vs->connect(SimNode::List{ SimNode::GND, n1 });
	r1->connect(SimNode::List{ n1, n2 });
	r2->connect(SimNode::List{ n2, SimNode::GND });
	r3->connect(SimNode::List{ n2, n3 });
	r4->connect(SimNode::List{ n3, SimNode::GND });
	cs->connect(SimNode::List{ SimNode::GND, n3 });

	// Define system topology
	auto sys = SystemTopology(50,
		SystemNodeList{n1, n2, n3},
		SystemComponentList{vs, r1, r2, r3, r4, cs});

	// Logging
	auto logger = CPS::DataLogger::make(simName);
	logger->logAttribute("v1", n1->attribute("v"));
	logger->logAttribute("v2", n2->attribute("v"));
	logger->logAttribute("v3", n3->attribute("v"));
	logger->logAttribute("i12", r1->attribute("i_intf"));
	logger->logAttribute("i23", r3->attribute("i_intf"));

	Simulation sim(simName);
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
	auto n1 = SimNode::make("n1");
	auto n2 = SimNode::make("n2");
	auto n3 = SimNode::make("n3");

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
	vs->connect(SimNode::List{ SimNode::GND, n1 });
	r1->connect(SimNode::List{ n1, n2 });
	r2->connect(SimNode::List{ n2, SimNode::GND });
	r3->connect(SimNode::List{ n2, n3 });
	r4->connect(SimNode::List{ n3, SimNode::GND });
	cs->connect(SimNode::List{ SimNode::GND, n3 });

	// Define system topology
	auto sys = SystemTopology(50,
		SystemNodeList{n1, n2, n3},
		SystemComponentList{vs, r1, r2, r4, cs});
	sys.addTearComponent(r3);

	// Logging
	auto logger = CPS::DataLogger::make(simName);
	logger->logAttribute("v1", n1->attribute("v"));
	logger->logAttribute("v2", n2->attribute("v"));
	logger->logAttribute("v3", n3->attribute("v"));
	logger->logAttribute("i12", r1->attribute("i_intf"));
	logger->logAttribute("i23", r3->attribute("i_intf"));

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
	auto n1 = SimNode::make("n1");
	auto n2 = SimNode::make("n2");
	auto n3 = SimNode::make("n3");
	auto n4 = SimNode::make("n4");

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
	vs->connect(SimNode::List{ SimNode::GND, n1 });
	r1->connect(SimNode::List{ n1, n2 });
	l1->connect(SimNode::List{ n2, n3 });
	l2->connect(SimNode::List{ n3, SimNode::GND });
	l3->connect(SimNode::List{ n3, n4 });
	r2->connect(SimNode::List{ n4, SimNode::GND });

	auto sys = SystemTopology(50,
		SystemNodeList{n1, n2, n3, n4},
		SystemComponentList{vs, r1, l1, l2, l3, r2});

	// Logging
	auto logger = CPS::DataLogger::make(simName);
	logger->logAttribute("v1", n1->attribute("v"));
	logger->logAttribute("v2", n2->attribute("v"));
	logger->logAttribute("v3", n3->attribute("v"));
	logger->logAttribute("v4", n4->attribute("v"));
	logger->logAttribute("i12", r1->attribute("i_intf"));
	logger->logAttribute("i34", l3->attribute("i_intf"));

	Simulation sim(simName);
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
	auto n1 = SimNode::make("n1");
	auto n2 = SimNode::make("n2");
	auto n3 = SimNode::make("n3");
	auto n4 = SimNode::make("n4");

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
	vs->connect(SimNode::List{ SimNode::GND, n1 });
	r1->connect(SimNode::List{ n1, n2 });
	l1->connect(SimNode::List{ n2, n3 });
	l2->connect(SimNode::List{ n3, SimNode::GND });
	l3->connect(SimNode::List{ n3, n4 });
	r2->connect(SimNode::List{ n4, SimNode::GND });

	auto sys = SystemTopology(50,
		SystemNodeList{n1, n2, n3, n4},
		SystemComponentList{vs, r1, l2, l3, r2});
	sys.addTearComponent(l1);

	// Logging
	auto logger = CPS::DataLogger::make(simName);
	logger->logAttribute("v1", n1->attribute("v"));
	logger->logAttribute("v2", n2->attribute("v"));
	logger->logAttribute("v3", n3->attribute("v"));
	logger->logAttribute("v4", n4->attribute("v"));
	logger->logAttribute("i12", r1->attribute("i_intf"));
	logger->logAttribute("i34", l3->attribute("i_intf"));

	Simulation sim(simName);
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
