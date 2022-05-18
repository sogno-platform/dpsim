/* Copyright 2017-2022 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <fstream>

#include <DPsim.h>
#include <dpsim/Villas/InterfaceVillas.h>

using namespace DPsim;
using namespace CPS::DP;
using namespace CPS::DP::Ph1;

int main(int argc, char* argv[]) {
	// Very simple test circuit. Just a few resistors and an inductance.
	// Voltage is read from VILLASnode and current through everything is written back.
	String simName = "Shmem_example";
	CPS::Logger::setLogDir("logs/"+simName);
	Real timeStep = 0.1;

	// Nodes
	auto n1 = SimNode::make("n1");
	auto n2 = SimNode::make("n2");
	auto n3 = SimNode::make("n3");
	auto n4 = SimNode::make("n4");

	// Components
	auto evs = VoltageSource::make("v_s");
	evs->setParameters(Complex(5, 0));
	auto rs =  Resistor::make("r_s");
	rs->setParameters(1);
	auto rl =  Resistor::make("r_line");
	rl->setParameters(1);
	auto ll =  Inductor::make("l_line");
	ll->setParameters(1);
	auto rL =  Resistor::make("r_load");
	rL->setParameters(1000);

	// Topology
	evs->connect({ SimNode::GND, n1 });
	rs->connect({ n1, n2 });
	rl->connect({ n2, n3 });
	ll->connect({ n3, n4 });
	rL->connect({ n4, SimNode::GND });

	auto sys = SystemTopology(50,
		SystemNodeList{SimNode::GND, n1, n2, n3, n4},
		SystemComponentList{evs, rs, rl, ll, rL});

	RealTimeSimulation sim(simName);
	sim.setSystem(sys);
	sim.setTimeStep(timeStep);
	sim.setFinalTime(2.0);
	
    std::string shmemConfig = R"STRING(
		{
        "type": "shmem",
        "in": {
            "name": "shmem-dpsim"
        },
        "out": {
            "name": "dpsim-shmem"
        },
		"queuelen": 1024
    })STRING";

    InterfaceVillas intf("dpsim-shmem", shmemConfig);

	// Interface
	//evs->setAttributeRef("V_ref", intf.importComplex(0));
	intf.exportComplex(evs->mIntfVoltage->deriveCoeff<Complex>(0, 0), 0, "v_src");
	intf.exportComplex(rL->mIntfVoltage->deriveCoeff<Complex>(0, 0), 1, "v_load");
	sim.addInterface(&intf, true);

	// Logger
	auto logger = DataLogger::make(simName);
	logger->logAttribute("v1", n1->mVoltage);
	logger->logAttribute("v2", n2->mVoltage);
	logger->logAttribute("v3", n3->mVoltage);
	logger->logAttribute("v4", n4->mVoltage);
	logger->logAttribute("v_src", evs->mVoltageRef);
	logger->logAttribute("i_evs", evs->mIntfCurrent, 1, 1);
	logger->logAttribute("v_evs", evs->mIntfVoltage, 1, 1);
	sim.addLogger(logger);

	sim.run(1);

	//std::ofstream of("task_dependencies.svg");
	//sim.dependencyGraph().render(of);
}
