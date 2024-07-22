
/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <DPsim.h>

using namespace DPsim;
using namespace CPS;


void EMT_Ph1_SwitchTest(String simName, Real timeStep, Real finalTime, Real switch_close_time)
{

	// Components
	auto gnd = SimNode<Real>::GND;
	auto n1 = SimNode<Real>::make("n1", PhaseType::Single);
	auto n2 = SimNode<Real>::make("n2", PhaseType::Single);

	auto r1 = EMT::Ph1::Resistor::make("r1");
	r1->setParameters(5.0);

	auto sw = EMT::Ph1::Switch::make("sw");
	sw->setParameters(1e9, 0.001, false);

	auto vs = EMT::Ph1::VoltageSource::make("vs");
	vs->setParameters(Complex(100.0, 0.0), 60.0);

	// Topology
	vs->connect({gnd, n1});
	r1->connect({n1, n2});
	sw->connect({n2, gnd});

	auto system = SystemTopology(60,
			SystemNodeList{gnd, n1, n2},
			SystemComponentList{vs, r1, sw});

	// Logging
	Logger::setLogDir("logs/" + simName);
	auto logger = DataLogger::make(simName);
	logger->logAttribute("I", r1->attribute("i_intf"));

	//Events
	auto sw_event = SwitchEvent::make(switch_close_time, sw, true);

	// Simulation
	Simulation sim(simName, Logger::Level::debug);
	sim.setSystem(system);
	sim.setTimeStep(timeStep);
	sim.setFinalTime(finalTime);
	sim.setDomain(Domain::EMT);
	sim.doInitFromNodesAndTerminals(false);
	sim.setDirectLinearSolverImplementation(DPsim::DirectLinearSolverImpl::SparseLU);
	sim.addLogger(logger);
	sim.addEvent(sw_event);
	sim.run();
}


int main(int argc, char* argv[]) {

	//Simultion parameters
	String simName ="EMT_Ph1_Switch_R1";
	Real finalTime = 0.1;
	Real timeStep = 0.001;
	Real switch_close_time = 0.06;

	EMT_Ph1_SwitchTest(simName, timeStep, finalTime, switch_close_time);
}
