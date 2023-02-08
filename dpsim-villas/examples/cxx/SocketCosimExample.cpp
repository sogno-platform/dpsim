// SPDX-License-Identifier: Apache-2.0

#include <fstream>

#include <DPsim.h>
#include <dpsim-villas/InterfaceVillas.h>

using namespace DPsim;
using namespace CPS::DP;
using namespace CPS::DP::Ph1;

int main(int argc, char* argv[]) {
	// Very simple test circuit. Just a few resistors and an inductance.
	// Voltage is read from VILLASnode and current through everything is written back.

	if (argc < 2) {
		std::cerr << "not enough arguments (either 0 or 1 for the test number)" << std::endl;
		std::exit(1);
	}

	Real timeStep = 0.01;

	if (String(argv[1]) == "0") {
		String simName = "SocketsCosim_example1";
		CPS::Logger::setLogDir("logs/"+simName);

		// Nodes
		auto n1 = SimNode::make("n1");
		auto n2 = SimNode::make("n2");

		// Components
		auto evs = VoltageSource::make("v_intf");
		evs->setParameters(Complex(5, 0));
		auto vs1 = VoltageSource::make("vs_1");
		vs1->setParameters(Complex(10, 0));
		auto r12 =  Resistor::make("r_12");
		r12->setParameters(1);

		// Topology
		evs->connect({ SimNode::GND, n2 });
		vs1->connect({ SimNode::GND, n1 });
		r12->connect({ n1, n2 });

		auto sys = SystemTopology(50,
			SystemNodeList{SimNode::GND, n1, n2},
			SystemComponentList{evs, vs1, r12});

		RealTimeSimulation sim(simName);
		sim.setSystem(sys);
		sim.setTimeStep(timeStep);
		sim.setFinalTime(0.1);
		
		// Make sure the format is set to json!!
		std::string socketConfig = R"STRING({
			"type": "socket",
			"layer": "udp",
			"format": "json",
        	"hooks": [
            	{"type": "print"}
        	],
			"in": {
				"address": "127.0.0.1:12008",
				"signals": [
					{
						"name": "v_intf",
						"type": "complex"
					}
				]
			},
			"out": {
				"address": "127.0.0.1:12009",
				"signals": [
					{
						"name": "i_intf",
						"type": "complex"
					}
				]
			}
		})STRING";

		// Logger. The logger must be added before the interface!
		auto logger = DataLogger::make(simName);
		logger->logAttribute("v1", n1->mVoltage);
		logger->logAttribute("v2", n2->mVoltage);
		logger->logAttribute("i_r", r12->mIntfCurrent, 1, 1);
		logger->logAttribute("i_evs", evs->mIntfCurrent, 1, 1);
		logger->logAttribute("v_evs", evs->mIntfVoltage, 1, 1);
		sim.addLogger(logger);

		auto intf = std::make_shared<InterfaceVillas>(socketConfig);

		Eigen::MatrixXcd intfCurrent0(1,1);
		intfCurrent0(0,0) = std::complex<double>(5.0,0.0);
		evs->setIntfCurrent(intfCurrent0);

		intf->importAttribute(evs->mVoltageRef, 0, false, false);
		intf->exportAttribute(evs->mIntfCurrent->deriveCoeff<Complex>(0, 0), 0, true, "i_intf");

		// Interface
		sim.addInterface(intf);

		sim.run(1);
	}
	else if (String(argv[1]) == "1") {
		String simName = "SocketsCosim_example2";
		CPS::Logger::setLogDir("logs/"+simName);

		// Nodes
		auto n2 = SimNode::make("n2");

		// Components
		auto ecs = CurrentSource::make("i_intf");
		ecs->setParameters(Complex(5, 0));
		auto r02 =  Resistor::make("r_02");
		r02->setParameters(1);

		// Topology
		ecs->connect({ SimNode::GND, n2 });
		r02->connect({ SimNode::GND, n2 });

		auto sys = SystemTopology(50,
			SystemNodeList{SimNode::GND, n2},
			SystemComponentList{ecs, r02});

		RealTimeSimulation sim(simName);
		sim.setSystem(sys);
		sim.setTimeStep(timeStep);
		sim.setFinalTime(0.1);
		
		// Make sure the format is set to json!!
		std::string socketConfig = R"STRING({
			"type": "socket",
			"layer": "udp",
			"format": "json",
			"hooks": [
				{"type": "print"}
			],
			"in": {
				"address": "127.0.0.1:12009",
				"signals": [
					{
						"name": "i_intf",
						"type": "complex"
					}
				]
			},
			"out": {
				"address": "127.0.0.1:12008",
				"signals": [
					{
						"name": "v_intf",
						"type": "complex"
					}
				]
			}
		})STRING";

		// Logger. The logger must be added before the interface!
		auto logger = DataLogger::make(simName);
		logger->logAttribute("v2", n2->mVoltage);
		logger->logAttribute("i_intf", r02->mIntfCurrent, 1, 1);
		logger->logAttribute("v_ecs", ecs->mIntfVoltage, 1, 1);
		logger->logAttribute("i_ecs", ecs->mIntfCurrent, 1, 1);
		sim.addLogger(logger);

		auto intf = std::make_shared<InterfaceVillas>(socketConfig);
		intf->importAttribute(ecs->mCurrentRef, 0, false, false);
		intf->exportAttribute(ecs->mIntfVoltage->deriveCoeff<Complex>(0, 0), 0, true, "v_intf");

		// Interface
		sim.addInterface(intf);

		sim.run(1);
	}
}
