// SPDX-License-Identifier: Apache-2.0

#include <fstream>
#include <iostream>

#include <DPsim.h>
#include <dpsim-villas/InterfaceVillas.h>

using namespace DPsim;
using namespace CPS;
using namespace std;

Simulation setDPsim1(float ts, float tf, float u10) {
	float r1_r = 0.1;
	float c1_c = 1;
	float rLine_r = 0.1;
	
	// ----- POWERFLOW FOR INITIALIZATION -----
	// String simNamePF = "Cosim_example1";
	// CPS::Logger::setLogDir("logs/"+simNamePF);

	// auto n1PF = SP::SimNode::make("n1");
	// auto n2PF = SP::SimNode::make("n2");

	// auto evsPF = SP::Ph1::VoltageSource::make("v_intf");
	// evsPF->setParameters(u10);

	// auto r1PF =  SP::Ph1::Resistor::make("r_1");
	// r1PF->setParameters(r1_r);
	// auto c1PF = SP::Ph1::Capacitor::make("c_1");
	// c1PF->setParameters(c1_c);
	// auto rLinePF = SP::Ph1::Resistor::make("r_line");
	// rLinePF->setParameters(rLine_r);

	// r1PF->connect({ n1PF, SP::SimNode::GND });
	// rLinePF->connect({ n1PF, n2PF });
	// c1PF->connect({ n1PF, SP::SimNode::GND });
	// evsPF->connect({ SP::SimNode::GND, n2PF });
	// auto systemPF = SystemTopology(50,
	// 	SystemNodeList{n1PF, n2PF},
	// 	SystemComponentList{c1PF, r1PF, rLinePF, evsPF});

	// auto loggerPF = DataLogger::make(simNamePF);
	// loggerPF->logAttribute("1_v_1", n1PF->mVoltage);
	// loggerPF->logAttribute("2_v_2", n2PF->mVoltage);

	// Simulation simPF(simNamePF, CPS::Logger::Level::debug);
	// simPF.setSystem(systemPF);
	// simPF.setTimeStep(0.1);
	// simPF.setFinalTime(0.1);
	// simPF.setDomain(Domain::SP);
	// simPF.setSolverType(Solver::Type::NRP);
	// simPF.doInitFromNodesAndTerminals(false);
	// simPF.addLogger(loggerPF);
	// simPF.run();
	
	
	// ----- DYNAMIC SIMULATION -----
	String simName = "Cosim_example1";
	CPS::Logger::setLogDir("logs/"+simName);

	// Nodes
	auto n1 = EMT::SimNode::make("n1");
	auto n2 = EMT::SimNode::make("n2");

	// Components
	auto evs = EMT::Ph1::VoltageSource::make("v_intf");
	evs->setParameters(u10);

	auto r1 =  EMT::Ph1::Resistor::make("r_1");
	r1->setParameters(r1_r);
	auto c1 = EMT::Ph1::Capacitor::make("c_1");
	c1->setParameters(c1_c);
	auto rLine = EMT::Ph1::Resistor::make("r_line");
	rLine->setParameters(rLine_r);

	// Topology
	r1->connect({ n1, EMT::SimNode::GND });
	rLine->connect({ n1, n2 });
	c1->connect({ n1, EMT::SimNode::GND });
	evs->connect({ EMT::SimNode::GND, n2 });

	auto sys = SystemTopology(50,
		SystemNodeList{EMT::SimNode::GND, n1, n2},
		SystemComponentList{r1, c1, rLine, evs});

	auto logger = DataLogger::make(simName);
	logger->logAttribute("1_v_1", n1->mVoltage);
	logger->logAttribute("2_v_2", n2->mVoltage);
	logger->logAttribute("3_i_rline", rLine->mIntfCurrent, 1, 1);
	logger->logAttribute("4_i_evs", evs->mIntfCurrent, 1, 1);
	logger->logAttribute("5_v_evs", evs->mIntfVoltage, 1, 1);
	
	Simulation sim(simName);
	sim.setDomain(CPS::Domain::EMT);
	sim.addLogger(logger);
	sim.setSystem(sys);
	sim.setTimeStep(ts);
	sim.setFinalTime(0.1);
	// sim.doSteadyStateInit(true);
	sim.doInitFromNodesAndTerminals(false);

	// initialize currents and voltages
	// Eigen::VectorXd n_v0(2);
	// n_v0 << 5, 5;

	Eigen::MatrixXd n1_v0(1,1);
	n1_v0(0,0) = 5;
	Eigen::MatrixXd n2_v0(1,1);
	n2_v0(0,0) = 2;

	Eigen::MatrixXd ir1_0(1,1);
	ir1_0(0,0) = n1_v0(0,0) / r1_r;
	Eigen::MatrixXd irLine_0(1,1);
	irLine_0(0,0) = (n1_v0(0,0) - n2_v0(0,0)) / rLine_r;

	r1->setIntfVoltage(n1_v0);
	r1->setIntfCurrent(ir1_0);
	c1->setIntfVoltage(n1_v0);
	c1->setIntfCurrent(ir1_0 - irLine_0);
	rLine->setIntfVoltage(n1_v0 - n2_v0);
	rLine->setIntfCurrent(irLine_0);
	// cout << "rLine voltage: " << rLine->mIntfVoltage->toString() << endl;
	// cout << "r1 current: " << r1->mIntfCurrent->toString() << endl;
	// cout << "c1 current: " << c1->mIntfCurrent->toString() << endl;

	evs->setIntfVoltage(n2_v0);
	evs->setIntfCurrent(irLine_0);
	cout << "evs current: " << evs->mIntfCurrent->toString() << endl;

	// Eigen::MatrixXd r1_i(1,1);
	// r1_i(0,0) = n1_v0(0,0) / r1_r;
	// r1->setIntfCurrent(r1_i);
	// c1->setIntfCurrent(Eigen::MatrixXd(20));
	// rLine->setIntfVoltage(Eigen::MatrixXd(3));
	// rLine->setIntfCurrent(Eigen::MatrixXd(30));

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
