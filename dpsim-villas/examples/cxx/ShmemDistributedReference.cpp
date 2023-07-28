// SPDX-License-Identifier: Apache-2.0

#include <DPsim.h>

using namespace DPsim;
using namespace CPS::DP;
using namespace CPS::DP::Ph1;

int main(int argc, char* argv[]) {
	String simName = "ShmemDistributedRef";
	Logger::setLogDir("logs/"+simName);
	Real timeStep = 0.001;
	Real finalTime = 0.1;

	// Nodes
	auto n1 = SimNode::make("n1", PhaseType::Single, std::vector<Complex>{ 10 });
	auto n2 = SimNode::make("n2", PhaseType::Single, std::vector<Complex>{ 5 });

	// Components
	auto vs1 = VoltageSource::make("vs_1", Logger::Level::debug);
	vs1->setParameters(Complex(10, 0));
	auto r12 = Resistor::make("r_12", Logger::Level::debug);
	r12->setParameters(1);
	auto r02 = Resistor::make("r_02", Logger::Level::debug);
	r02->setParameters(1);

	// Connections
	vs1->connect({ SimNode::GND, n1 });
	r12->connect({ n1, n2 });
	r02->connect({ SimNode::GND, n2 });

	auto sys = SystemTopology(50,
		SystemNodeList{ n1, n2 },
		SystemComponentList{ vs1, r12, r02 });

	// Logging
	auto logger = CPS::DataLogger::make(simName);
	logger->logAttribute("v1", n1->mVoltage);
	logger->logAttribute("v2", n2->mVoltage);
	logger->logAttribute("r12", r12->mIntfCurrent);
	logger->logAttribute("r02", r02->mIntfCurrent);

	Simulation sim(simName);
	sim.setSystem(sys);
	sim.setTimeStep(timeStep);
	sim.setFinalTime(finalTime);
	sim.addLogger(logger);

	sim.run();
}
