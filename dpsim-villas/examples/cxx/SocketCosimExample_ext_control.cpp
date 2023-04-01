// SPDX-License-Identifier: Apache-2.0

#include <fstream>
#include <iostream>

#include <DPsim.h>
#include <dpsim-villas/InterfaceVillas.h>

using namespace DPsim;
using namespace CPS::EMT;
using namespace CPS::EMT::Ph1;
using namespace std;

Simulation setDPsim1(float ts, float tf, float u10) {
	String simName = "SocketsCosim_example1";
	CPS::Logger::setLogDir("logs/"+simName);

	// Nodes
	auto n1 = SimNode::make("n1");
	auto n2 = SimNode::make("n2");

	// Components
	auto evs = VoltageSource::make("v_intf");
	evs->setParameters(u10);

	auto r1 =  Resistor::make("r_1");
	r1->setParameters(0.1);
	auto c1 = Capacitor::make("c_1");
	c1->setParameters(1);
	auto rLine = Resistor::make("r_line");
	rLine->setParameters(0.1);

	// Topology
	r1->connect({ n1, SimNode::GND });
	rLine->connect({ n1, n2 });
	c1->connect({ n1, SimNode::GND });
	evs->connect({ SimNode::GND, n2 });

	auto sys = SystemTopology(50,
		SystemNodeList{SimNode::GND, n1, n2},
		SystemComponentList{r1, c1, rLine, evs});

	Simulation sim(simName);
	sim.setDomain(CPS::Domain::EMT);
	sim.setSystem(sys);
	sim.setTimeStep(ts);
	sim.setFinalTime(0.1);
	sim.doSteadyStateInit(true);
	
	auto logger = DataLogger::make(simName);
	logger->logAttribute("1_v_1", n1->mVoltage);
	logger->logAttribute("2_v_2", n2->mVoltage);
	logger->logAttribute("3_i_rline", rLine->mIntfCurrent, 1, 1);
	logger->logAttribute("4_i_evs", evs->mIntfCurrent, 1, 1);
	logger->logAttribute("5_v_evs", evs->mIntfVoltage, 1, 1);
	sim.addLogger(logger);

	// Eigen::MatrixXcd intfCurrent0(1,1);
	// intfCurrent0(0,0) = std::complex<double>(5.0,0.0);
	// evs->setIntfCurrent(intfCurrent0);

	return sim;
}

int main(int argc, char* argv[]) {

	float timeStep = 0.01;
	float finalTime = 1.0;

	// Communication y20 -> S_1 and initialization of S_1
	float y20 = 2.0;
	Simulation sim1 = setDPsim1(timeStep, finalTime, y20);
	sim1.start();
	CPS::AttributeBase *y10 = sim1.getIdObjAttribute("v_intf", "i_intf").get();
	cout << "Output value: " << y10->toString() << endl;
	float ki1 = sim1.next();
	y10 = sim1.getIdObjAttribute("v_intf", "i_intf").get();
	cout << "Step: " << ki1 << ", Output value: " << y10->toString() << endl;

	sim1.stop();

	// if (String(argv[1]) == "0") {


	// 	sim.run();
	// }
	// else if (String(argv[1]) == "1") {
	// 	String simName = "SocketsCosim_example2";
	// 	CPS::Logger::setLogDir("logs/"+simName);

	// 	// Nodes
	// 	auto n2 = SimNode::make("n2");

	// 	// Components
	// 	auto ecs = CurrentSource::make("i_intf");
	// 	ecs->setParameters(Complex(5, 0));
	// 	auto r02 =  Resistor::make("r_02");
	// 	r02->setParameters(1);

	// 	// Topology
	// 	ecs->connect({ SimNode::GND, n2 });
	// 	r02->connect({ SimNode::GND, n2 });

	// 	auto sys = SystemTopology(50,
	// 		SystemNodeList{SimNode::GND, n2},
	// 		SystemComponentList{ecs, r02});

	// 	Simulation sim(simName);
	// 	sim.setSystem(sys);
	// 	sim.setTimeStep(timeStep);
	// 	sim.setFinalTime(0.1);
		
	// 	// Make sure the format is set to json!!
	// 	std::string socketConfig = R"STRING({
	// 		"type": "socket",
	// 		"layer": "udp",
	// 		"format": "json",
	// 		"hooks": [
	// 			{"type": "print"}
	// 		],
	// 		"in": {
	// 			"address": "127.0.0.1:12009",
	// 			"signals": [
	// 				{
	// 					"name": "i_intf",
	// 					"type": "complex"
	// 				}
	// 			]
	// 		},
	// 		"out": {
	// 			"address": "127.0.0.1:12008",
	// 			"signals": [
	// 				{
	// 					"name": "v_intf",
	// 					"type": "complex"
	// 				}
	// 			]
	// 		}
	// 	})STRING";

	// 	// Logger. The logger must be added before the interface!
	// 	auto logger = DataLogger::make(simName);
	// 	logger->logAttribute("v2", n2->mVoltage);
	// 	logger->logAttribute("i_intf", r02->mIntfCurrent, 1, 1);
	// 	logger->logAttribute("v_ecs", ecs->mIntfVoltage, 1, 1);
	// 	logger->logAttribute("i_ecs", ecs->mIntfCurrent, 1, 1);
	// 	sim.addLogger(logger);

	// 	auto intf = std::make_shared<InterfaceVillas>(socketConfig);
	// 	intf->importAttribute(ecs->mCurrentRef, 0, false, true);
	// 	intf->exportAttribute(ecs->mIntfVoltage->deriveCoeff<Complex>(0, 0), 0, true, "v_intf");

	// 	// Interface
	// 	sim.addInterface(intf);

	// 	sim.run();
	// }
}
