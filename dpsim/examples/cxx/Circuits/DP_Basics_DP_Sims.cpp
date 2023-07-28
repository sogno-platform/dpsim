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

void DP_VS_RL_f60_largeTs();
void DP_VS_RL_f60_vlargeTs();
void DP_VS_RL_f60();
void DP_VS_RL_f500_largeTs();
void DP_VS_RL_f500_ph500();
void DP_VS_RL_f500();

int main(int argc, char* argv[]) {
	DP_VS_RL_f60_largeTs();
	DP_VS_RL_f60_vlargeTs();
	DP_VS_RL_f60();
	DP_VS_RL_f500_largeTs();
	DP_VS_RL_f500_ph500();
	DP_VS_RL_f500();
}

void DP_VS_RL_f60_largeTs() {
	Real timeStep = 0.01;
	Real finalTime = 0.2;
	String simName = "DP_VS_RL_f60_largeTs";
	Logger::setLogDir("logs/"+simName);

	// Nodes
	auto n1 = SimNode::make("n1");
	auto n2 = SimNode::make("n2");
	auto n3 = SimNode::make("n3");

	// Components
	auto vs = VoltageSource::make("vs");
	vs->setParameters(Complex(230, 0), 10);
	auto rline = Resistor::make("r_line");
	rline->setParameters(1);
	auto lline = Inductor::make("l_line");
	lline->setParameters(0.02);
	auto rload = Resistor::make("r_load");
	rload->setParameters(10);

	// Connections
	vs->connect({ SimNode::GND, n1 });
	rline->connect({ n1, n2 });
	lline->connect({ n2, n3 });
	rload->connect({ n3, SimNode::GND });

	// Define system topology
	auto sys = SystemTopology(50,
		SystemNodeList{n1, n2, n3},
		SystemComponentList{vs, rline, lline, rload});

	// Logger
	auto logger = CPS::DataLogger::make(simName);
	logger->logAttribute("v1", n1->attribute("v"));
	logger->logAttribute("v2", n2->attribute("v"));
	logger->logAttribute("v3", n3->attribute("v"));
	logger->logAttribute("i_line", rline->attribute("i_intf"));

	Simulation sim(simName, Logger::Level::info);
	sim.setSystem(sys);
	sim.setTimeStep(timeStep);
	sim.setFinalTime(finalTime);
	sim.initialize();

	sim.addLogger(logger);

	sim.run();
}

void DP_VS_RL_f60_vlargeTs() {
	Real timeStep = 0.05;
	Real finalTime = 0.2;
	String simName = "DP_VS_RL_f60_vlargeTs";
	Logger::setLogDir("logs/"+simName);

	// Nodes
	auto n1 = SimNode::make("n1");
	auto n2 = SimNode::make("n2");
	auto n3 = SimNode::make("n3");

	// Components
	auto vs = VoltageSource::make("vs");
	vs->setParameters(Complex(230, 0), 10);
	auto rline = Resistor::make("r_line");
	rline->setParameters(1);
	auto lline = Inductor::make("l_line");
	lline->setParameters(0.02);
	auto rload = Resistor::make("r_load");
	rload->setParameters(10);

	// Connections
	vs->connect({ SimNode::GND, n1 });
	rline->connect({ n1, n2 });
	lline->connect({ n2, n3 });
	rload->connect({ n3, SimNode::GND });

	// Define system topology
	auto sys = SystemTopology(50,
		SystemNodeList{n1, n2, n3},
		SystemComponentList{vs, rline, lline, rload});

	// Logger
	auto logger = CPS::DataLogger::make(simName);
	logger->logAttribute("v1", n1->attribute("v"));
	logger->logAttribute("v2", n2->attribute("v"));
	logger->logAttribute("v3", n3->attribute("v"));
	logger->logAttribute("i_line", rline->attribute("i_intf"));

	Simulation sim(simName, Logger::Level::info);
	sim.setSystem(sys);
	sim.setTimeStep(timeStep);
	sim.setFinalTime(finalTime);
	sim.initialize();

	sim.addLogger(logger);

	sim.run();
}

void DP_VS_RL_f60() {
	Real timeStep = 0.0001;
	Real finalTime = 0.2;
	String simName = "DP_VS_RL_f60";
	Logger::setLogDir("logs/"+simName);

	// Nodes
	auto n1 = SimNode::make("n1");
	auto n2 = SimNode::make("n2");
	auto n3 = SimNode::make("n3");

	// Components
	auto vs = VoltageSource::make("vs");
	vs->setParameters(Complex(230, 0), 10);
	auto rline = Resistor::make("r_line");
	rline->setParameters(1);
	auto lline = Inductor::make("l_line");
	lline->setParameters(0.02);
	auto rload = Resistor::make("r_load");
	rload->setParameters(10);

	// Connections
	vs->connect({ SimNode::GND, n1 });
	rline->connect({ n1, n2 });
	lline->connect({ n2, n3 });
	rload->connect({ n3, SimNode::GND });

	// Define system topology
	auto sys = SystemTopology(50,
		SystemNodeList{n1, n2, n3},
		SystemComponentList{vs, rline, lline, rload});

	// Logger
	auto logger = CPS::DataLogger::make(simName);
	logger->logAttribute("v1", n1->attribute("v"));
	logger->logAttribute("v2", n2->attribute("v"));
	logger->logAttribute("v3", n3->attribute("v"));
	logger->logAttribute("i_line", rline->attribute("i_intf"));

	Simulation sim(simName, Logger::Level::info);
	sim.setSystem(sys);
	sim.setTimeStep(timeStep);
	sim.setFinalTime(finalTime);
	sim.initialize();

	sim.addLogger(logger);

	sim.run();
}

void DP_VS_RL_f500_largeTs() {
	Real timeStep = 0.002;
	Real finalTime = 0.2;
	String simName = "DP_VS_RL_f500_largeTs";
	Logger::setLogDir("logs/"+simName);

	// Nodes
	auto n1 = SimNode::make("n1");
	auto n2 = SimNode::make("n2");
	auto n3 = SimNode::make("n3");

	// Components
	auto vs = VoltageSource::make("vs");
	vs->setParameters(Complex(230, 0), 450);
	auto rline = Resistor::make("r_line");
	rline->setParameters(1);
	auto lline = Inductor::make("l_line");
	lline->setParameters(0.02);
	auto rload = Resistor::make("r_load");
	rload->setParameters(10);

	// Connections
	vs->connect({ SimNode::GND, n1 });
	rline->connect({ n1, n2 });
	lline->connect({ n2, n3 });
	rload->connect({ n3, SimNode::GND });

	// Define system topology
	auto sys = SystemTopology(50,
		SystemNodeList{n1, n2, n3},
		SystemComponentList{vs, rline, lline, rload});

	// Logger
	auto logger = CPS::DataLogger::make(simName);
	logger->logAttribute("v1", n1->attribute("v"));
	logger->logAttribute("v2", n2->attribute("v"));
	logger->logAttribute("v3", n3->attribute("v"));
	logger->logAttribute("i_line", rline->attribute("i_intf"));

	Simulation sim(simName, Logger::Level::info);
	sim.setSystem(sys);
	sim.setTimeStep(timeStep);
	sim.setFinalTime(finalTime);
	sim.initialize();

	sim.addLogger(logger);

	sim.run();
}

void DP_VS_RL_f500_ph500() {
	Real timeStep = 0.002;
	Real finalTime = 0.2;
	String simName = "DP_VS_RL_f500_ph500";
	Logger::setLogDir("logs/"+simName);

	// Nodes
	auto n1 = SimNode::make("n1");
	auto n2 = SimNode::make("n2");
	auto n3 = SimNode::make("n3");

	// Components
	auto vs = VoltageSource::make("vs");
	vs->setParameters(Complex(230, 0));
	auto rline = Resistor::make("r_line");
	rline->setParameters(1);
	auto lline = Inductor::make("l_line");
	lline->setParameters(0.02);
	auto rload = Resistor::make("r_load");
	rload->setParameters(10);

	// Connections
	vs->connect({ SimNode::GND, n1 });
	rline->connect({ n1, n2 });
	lline->connect({ n2, n3 });
	rload->connect({ n3, SimNode::GND });

	// Define system topology
	auto sys = SystemTopology(500,
		SystemNodeList{n1, n2, n3},
		SystemComponentList{vs, rline, lline, rload});

	// Logger
	auto logger = CPS::DataLogger::make(simName);
	logger->logAttribute("v1", n1->attribute("v"));
	logger->logAttribute("v2", n2->attribute("v"));
	logger->logAttribute("v3", n3->attribute("v"));
	logger->logAttribute("i_line", rline->attribute("i_intf"));

	Simulation sim(simName, Logger::Level::info);
	sim.setSystem(sys);
	sim.setTimeStep(timeStep);
	sim.setFinalTime(finalTime);
	sim.initialize();

	sim.addLogger(logger);

	sim.run();
}

void DP_VS_RL_f500() {
	Real timeStep = 0.00001;
	Real finalTime = 0.2;
	String simName = "DP_VS_RL_f500";
	Logger::setLogDir("logs/"+simName);

	// Nodes
	auto n1 = SimNode::make("n1");
	auto n2 = SimNode::make("n2");
	auto n3 = SimNode::make("n3");

	// Components
	auto vs = VoltageSource::make("vs");
	vs->setParameters(Complex(230, 0), 450);
	auto rline = Resistor::make("r_line");
	rline->setParameters(1);
	auto lline = Inductor::make("l_line");
	lline->setParameters(0.02);
	auto rload = Resistor::make("r_load");
	rload->setParameters(10);

	// Connections
	vs->connect({ SimNode::GND, n1 });
	rline->connect({ n1, n2 });
	lline->connect({ n2, n3 });
	rload->connect({ n3, SimNode::GND });

	// Define system topology
	auto sys = SystemTopology(50,
		SystemNodeList{n1, n2, n3},
		SystemComponentList{vs, rline, lline, rload});

	// Logger
	auto logger = CPS::DataLogger::make(simName);
	logger->logAttribute("v1", n1->attribute("v"));
	logger->logAttribute("v2", n2->attribute("v"));
	logger->logAttribute("v3", n3->attribute("v"));
	logger->logAttribute("i_line", rline->attribute("i_intf"));

	Simulation sim(simName, Logger::Level::info);
	sim.setSystem(sys);
	sim.setTimeStep(timeStep);
	sim.setFinalTime(finalTime);
	sim.initialize();

	sim.addLogger(logger);

	sim.run();
}
