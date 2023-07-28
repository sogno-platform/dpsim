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

void vsSetParamsDP1ph() {
	Real timeStep = 0.00005;
	Real finalTime = 0.1;
	String simName = "DP_VS_SetParams";
	Logger::setLogDir("logs/"+simName);

	// Nodes
	auto n1 = SimNode<Complex>::make("n1");

	// Components
	auto vs = DP::Ph1::VoltageSource::make("vs1");
	vs->setParameters(CPS::Math::polar(100000, 0));

	// Topology
	vs->connect({ SimNode<Complex>::GND, n1 });

	auto sys = SystemTopology(50,
		SystemNodeList{n1},
		SystemComponentList{vs});

	// Logging
	auto logger = CPS::DataLogger::make(simName);
	logger->logAttribute("v1", n1->attribute("v"));

	// Simulation
	Simulation sim(simName);
	sim.setSystem(sys);
	sim.setTimeStep(timeStep);
	sim.setFinalTime(finalTime);
	sim.setDomain(Domain::DP);
	sim.addLogger(logger);
	sim.run();
}

void vsSetParamsSP1ph() {
	Real timeStep = 0.00005;
	Real finalTime = 0.1;
	String simName = "SP_VS_SetParams";
	Logger::setLogDir("logs/"+simName);

	// Nodes
	auto n1 = SimNode<Complex>::make("n1");

	// Components
	auto vs = SP::Ph1::VoltageSource::make("vs1");
	vs->setParameters(CPS::Math::polar(100000, 0));

	// Topology
	vs->connect({ SimNode<Complex>::GND, n1 });

	auto sys = SystemTopology(50,
		SystemNodeList{n1},
		SystemComponentList{vs});

	// Logging
	auto logger = CPS::DataLogger::make(simName);
	logger->logAttribute("v1", n1->attribute("v"));

	// Simulation
	Simulation sim(simName);
	sim.setSystem(sys);
	sim.setTimeStep(timeStep);
	sim.setFinalTime(finalTime);
	sim.setDomain(Domain::SP);
	sim.addLogger(logger);
	sim.run();
}

void vsSetParamsEMT3ph() {
	Real timeStep = 0.00005;
	Real finalTime = 0.1;
	String simName = "EMT_VS_SetParams";
	Logger::setLogDir("logs/"+simName);

	// Nodes
	auto n1 = SimNode<Real>::make("n1", PhaseType::ABC);

	// Components
	auto vs = EMT::Ph3::VoltageSource::make("vs1");
	vs->setParameters(CPS::Math::singlePhaseVariableToThreePhase(CPS::Math::polar(100000, 0)), 50);

	// Topology
	vs->connect({ SimNode<Real>::GND, n1 });

	auto sys = SystemTopology(50,
		SystemNodeList{n1},
		SystemComponentList{vs});

	// Logging
	auto logger = CPS::DataLogger::make(simName);
	logger->logAttribute("v1", n1->attribute("v"));

	// Simulation
	Simulation sim(simName);
	sim.setSystem(sys);
	sim.setTimeStep(timeStep);
	sim.setFinalTime(finalTime);
	sim.setDomain(Domain::EMT);
	sim.addLogger(logger);
	sim.run();
}

void vsSetAttrDP1ph() {
	Real timeStep = 0.00005;
	Real finalTime = 0.1;
	String simName = "DP_VS_SetAttr";
	Logger::setLogDir("logs/"+simName);

    Complex vref = CPS::Math::polar(100000, 0);

	// Nodes
	auto n1 = SimNode<Complex>::make("n1");

	// Components
	auto vs = DP::Ph1::VoltageSource::make("vs1");
	vs->attributeTyped<Complex>("V_ref")->set(vref);

	// Topology
	vs->connect({ SimNode<Complex>::GND, n1 });

	auto sys = SystemTopology(50,
		SystemNodeList{n1},
		SystemComponentList{vs});

	// Logging
	auto logger = CPS::DataLogger::make(simName);
	logger->logAttribute("v1", n1->attribute("v"));

	// Simulation
	Simulation sim(simName);
	sim.setSystem(sys);
	sim.setTimeStep(timeStep);
	sim.setFinalTime(finalTime);
	sim.setDomain(Domain::DP);
	sim.addLogger(logger);
	sim.run();
}

void vsSetAttrSP1ph() {
	Real timeStep = 0.00005;
	Real finalTime = 0.1;
	String simName = "SP_VS_SetAttr";
	Logger::setLogDir("logs/"+simName);

    Complex vref = CPS::Math::polar(100000, 0);

	// Nodes
	auto n1 = SimNode<Complex>::make("n1");

	// Components
	auto vs = SP::Ph1::VoltageSource::make("vs1");
	vs->attributeTyped<Complex>("V_ref")->set(vref);

	// Topology
	vs->connect({ SimNode<Complex>::GND, n1 });

	auto sys = SystemTopology(50,
		SystemNodeList{n1},
		SystemComponentList{vs});

	// Logging
	auto logger = CPS::DataLogger::make(simName);
	logger->logAttribute("v1", n1->attribute("v"));

	// Simulation
	Simulation sim(simName);
	sim.setSystem(sys);
	sim.setTimeStep(timeStep);
	sim.setFinalTime(finalTime);
	sim.setDomain(Domain::SP);
	sim.addLogger(logger);
	sim.run();
}

void vsSetAttrEMT3ph() {
	Real timeStep = 0.00005;
	Real finalTime = 0.1;
	String simName = "EMT_VS_SetAttr";
	Logger::setLogDir("logs/"+simName);

    MatrixComp vref = CPS::Math::singlePhaseVariableToThreePhase(CPS::Math::polar(100000, 0));

	// Nodes
	auto n1 = SimNode<Real>::make("n1", PhaseType::ABC);

	// Components
	auto vs = EMT::Ph3::VoltageSource::make("vs1");
	vs->attributeTyped<MatrixComp>("V_ref")->set(vref);

	// Topology
	vs->connect({ SimNode<Real>::GND, n1 });

	auto sys = SystemTopology(50,
		SystemNodeList{n1},
		SystemComponentList{vs});

	// Logging
	auto logger = CPS::DataLogger::make(simName);
	logger->logAttribute("v1", n1->attribute("v"));

	// Simulation
	Simulation sim(simName);
	sim.setSystem(sys);
	sim.setTimeStep(timeStep);
	sim.setFinalTime(finalTime);
	sim.setDomain(Domain::EMT);
	sim.addLogger(logger);
	sim.run();
}

void vsSetFromNodeDP1ph() {
	Real timeStep = 0.00005;
	Real finalTime = 0.1;
	String simName = "DP_VS_SetFromNode";
	Logger::setLogDir("logs/"+simName);

	Complex vref = CPS::Math::polar(100000, 0);

	// Nodes
	auto n1 = SimNode<Complex>::make("n1", PhaseType::Single, std::vector<Complex>{vref});
	n1->setInitialVoltage(vref);

	// Components
	auto vs = DP::Ph1::VoltageSource::make("vs1");

	// Topology
	vs->connect({ SimNode<Complex>::GND, n1 });

	auto sys = SystemTopology(50,
		SystemNodeList{n1},
		SystemComponentList{vs});

	// Logging
	auto logger = CPS::DataLogger::make(simName);
	logger->logAttribute("v1", n1->attribute("v"));

	// Simulation
	Simulation sim(simName);
	sim.setSystem(sys);
	sim.setTimeStep(timeStep);
	sim.setFinalTime(finalTime);
	sim.setDomain(Domain::DP);
	sim.addLogger(logger);
	sim.run();
}

void vsSetFromNodeSP1ph() {
	Real timeStep = 0.00005;
	Real finalTime = 0.1;
	String simName = "SP_VS_SetFromNode";
	Logger::setLogDir("logs/"+simName);

	Complex vref = CPS::Math::polar(100000, 0);

	// Nodes
	auto n1 = SimNode<Complex>::make("n1", PhaseType::Single, std::vector<Complex>{vref});
	n1->setInitialVoltage(vref);

	// Components
	auto vs = SP::Ph1::VoltageSource::make("vs1");

	// Topology
	vs->connect({ SimNode<Complex>::GND, n1 });

	auto sys = SystemTopology(50,
		SystemNodeList{n1},
		SystemComponentList{vs});

	// Logging
	auto logger = CPS::DataLogger::make(simName);
	logger->logAttribute("v1", n1->attribute("v"));

	// Simulation
	Simulation sim(simName);
	sim.setSystem(sys);
	sim.setTimeStep(timeStep);
	sim.setFinalTime(finalTime);
	sim.setDomain(Domain::SP);
	sim.addLogger(logger);
	sim.run();
}

void vsSetFromNodeEMT3ph() {
	Real timeStep = 0.00005;
	Real finalTime = 0.1;
	String simName = "EMT_VS_SetFromNode";
	Logger::setLogDir("logs/"+simName);

    Complex vref = CPS::Math::polar(100000, 0);

	// Nodes
	auto n1 = SimNode<Real>::make("n1", PhaseType::ABC);
	n1->setInitialVoltage(vref);

	// Components
	auto vs = EMT::Ph3::VoltageSource::make("vs1");

	// Topology
	vs->connect({ SimNode<Real>::GND, n1 });

	auto sys = SystemTopology(50,
		SystemNodeList{n1},
		SystemComponentList{vs});

	// Logging
	auto logger = CPS::DataLogger::make(simName);
	logger->logAttribute("v1", n1->attribute("v"));

	// Simulation
	Simulation sim(simName);
	sim.setSystem(sys);
	sim.setTimeStep(timeStep);
	sim.setFinalTime(finalTime);
	sim.setDomain(Domain::EMT);
	sim.addLogger(logger);
	sim.run();
}

int main(int argc, char* argv[]) {
	vsSetParamsDP1ph();
	vsSetParamsSP1ph();
    vsSetParamsEMT3ph();

    vsSetAttrDP1ph();
    vsSetAttrSP1ph();
    vsSetAttrEMT3ph();

	vsSetFromNodeDP1ph();
	vsSetFromNodeSP1ph();
	vsSetFromNodeEMT3ph();
}
