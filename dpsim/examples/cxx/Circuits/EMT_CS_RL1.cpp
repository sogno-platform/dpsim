/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <DPsim.h>

using namespace DPsim;
using namespace CPS::EMT;
using namespace CPS::EMT::Ph1;

int main(int argc, char* argv[]) {
	// Define simulation scenario
	
	std::shared_ptr<SolverParameters> mna_parameter = std::make_shared<SolverParametersMNA>();
	Real timeStep = 1e-4;
	Real finalTime = 1e-3;
	String simName = "EMT_CS_RL1";
	Logger::setLogDir("logs/"+simName);

	// Nodes
	auto n1 = SimNode::make("n1", PhaseType::Single, std::vector<Complex>{ 2 });

	// Components
	auto cs = CurrentSource::make("cs", Logger::Level::info);
	cs->setParameters(Complex(10, 0), 0);
	auto r1 = Resistor::make("r_1");
	r1->setParameters(0.2);
	auto l1 = Inductor::make("l_1", Logger::Level::info);
	l1->setParameters(0.001);

	// Topology
	cs->connect(SimNode::List{ SimNode::GND, n1 });
	r1->connect(SimNode::List{ n1, SimNode::GND });
	l1->connect(SimNode::List{ n1, SimNode::GND });

	// Define system topology
	auto sys = SystemTopology(50, SystemNodeList{n1}, SystemComponentList{cs, r1, l1});

	// Logger
	auto logger = DataLogger::make(simName);
	logger->logAttribute("v1", n1->attribute("v"));
	logger->logAttribute("iR1", r1->attribute("i_intf"));
	logger->logAttribute("iL1", l1->attribute("i_intf"));

	Simulation sim(simName);
	sim.setSystem(sys);
	sim.setTimeStep(timeStep);
	sim.setFinalTime(finalTime);
	sim.setDomain(Domain::EMT);
	sim.addLogger(logger);
	sim.setSolverParameters(Domain::EMT, Solver::Type::MNA, mna_parameter);

	sim.run();

	return 0;
}
