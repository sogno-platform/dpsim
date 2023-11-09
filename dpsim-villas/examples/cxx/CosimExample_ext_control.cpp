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
	// Logger::setLogDir("logs/"+simNamePF);

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

	// Simulation simPF(simNamePF, Logger::Level::debug);
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
	auto evs = EMT::Ph1::VoltageSource::make("v_in_1");
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
	evs->connect({ n2, EMT::SimNode::GND });

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
	sim.setDomain(Domain::EMT);
	sim.addLogger(logger);
	sim.setSystem(sys);
	sim.setTimeStep(ts);
	sim.setFinalTime(tf);
	// sim.doSteadyStateInit(true);
	sim.doInitFromNodesAndTerminals(false);

	// initialize currents and voltages
	// Eigen::VectorXd n_v0(2);
	// n_v0 << 5, 5;

	// Initial conditions, given by the problem
	Eigen::MatrixXd n1_v0(1,1);
	n1_v0(0,0) = 5.0;
	Eigen::MatrixXd n2_v0(1,1);
	n2_v0(0,0) = u10;

	Eigen::MatrixXd ir1_0(1,1);
	// ir1_0(0,0) = n1_v0(0,0) / r1_r;
	ir1_0(0,0) = 50;
	Eigen::MatrixXd irLine_0(1,1);
	// irLine_0(0,0) = (n1_v0(0,0) - n2_v0(0,0)) / rLine_r;
	irLine_0(0,0) = 30;

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

Simulation setDPsim2(float ts, float tf, float u20) {
	float r3_r = 1;
	float c2_c = 1;
	
	// ----- POWERFLOW FOR INITIALIZATION -----
	// String simNamePF = "Cosim_example1";
	// Logger::setLogDir("logs/"+simNamePF);

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

	// Simulation simPF(simNamePF, Logger::Level::debug);
	// simPF.setSystem(systemPF);
	// simPF.setTimeStep(0.1);
	// simPF.setFinalTime(0.1);
	// simPF.setDomain(Domain::SP);
	// simPF.setSolverType(Solver::Type::NRP);
	// simPF.doInitFromNodesAndTerminals(false);
	// simPF.addLogger(loggerPF);
	// simPF.run();
	
	
	// ----- DYNAMIC SIMULATION -----
	String simName = "Cosim_example2";
	CPS::Logger::setLogDir("logs/"+simName);

	// Nodes
	auto n2 = EMT::SimNode::make("n2");

	// Components
	auto is = EMT::Ph1::CurrentSource::make("i_in_2");
	is->setParameters(u20);

	auto r3 =  EMT::Ph1::Resistor::make("r_3");
	r3->setParameters(r3_r);
	auto c2 = EMT::Ph1::Capacitor::make("c_2");
	c2->setParameters(c2_c);

	// Topology
	r3->connect({ n2, EMT::SimNode::GND });
	c2->connect({ n2, EMT::SimNode::GND });
	is->connect({ EMT::SimNode::GND, n2 });

	auto sys = SystemTopology(50,
		SystemNodeList{EMT::SimNode::GND, n2},
		SystemComponentList{r3, c2, is});

	auto logger = DataLogger::make(simName);
	logger->logAttribute("2_v_2", n2->mVoltage);
	logger->logAttribute("4_i_evs", is->mIntfCurrent, 1, 1);
	logger->logAttribute("5_v_evs", is->mIntfVoltage, 1, 1);
	
	Simulation sim(simName);
	sim.setDomain(Domain::EMT);
	sim.addLogger(logger);
	sim.setSystem(sys);
	sim.setTimeStep(ts);
	sim.setFinalTime(tf);
	// sim.doSteadyStateInit(true);
	sim.doInitFromNodesAndTerminals(false);

	// initialize currents and voltages
	// Eigen::VectorXd n_v0(2);
	// n_v0 << 5, 5;

	// Initial conditions, given by the problem
	Eigen::MatrixXd n2_v0(1,1);
	n2_v0(0,0) = 2.0;
	Eigen::MatrixXd i_rLine0(1,1);
	i_rLine0(0,0) = u20; 

	Eigen::MatrixXd ir3_0(1,1);
	ir3_0(0,0) = n2_v0(0,0) / r3_r;

	r3->setIntfVoltage(n2_v0);
	r3->setIntfCurrent(ir3_0);
	c2->setIntfVoltage(n2_v0);
	c2->setIntfCurrent(i_rLine0 - ir3_0);
	// cout << "rLine voltage: " << rLine->mIntfVoltage->toString() << endl;
	// cout << "r1 current: " << r1->mIntfCurrent->toString() << endl;
	// cout << "c1 current: " << c1->mIntfCurrent->toString() << endl;

	is->setIntfVoltage(n2_v0);
	is->setIntfCurrent(i_rLine0);
	cout << "is current: " << is->mIntfCurrent->toString() << endl;

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

	// ** Initialization **
	// Communication y20 -> S_1 and initialization of S_1
	// float y20 = 2.0;
	
	// Set up subsytem 1
	Simulation sim1 = setDPsim1(timeStep, finalTime, 2.0);
	sim1.start();

	AttributeBase::Ptr y10_base = sim1.getIdObjAttribute("v_in_1", "i_intf");
	
	// try {
	auto y10_matrix = std::dynamic_pointer_cast<Attribute<Matrix>>(y10_base.getPtr());
	Attribute<Real>::Ptr y10 = y10_matrix->deriveCoeff<Real>(0,0);
	cout << "Output value from S1: " << y10->toString() << endl;
	cout << "Output value from S1: " << **y10 << endl;

	// } catch(...) {
	// 	throw InvalidAttributeException();
	// }

	// "Communication" y10 -> S_2 and initialization of S_2
	AttributeBase::Ptr u20_base = y10->cloneValueOntoNewAttribute();
	auto u20Attr = std::dynamic_pointer_cast<Attribute<Real>>(u20_base.getPtr());
	// std::shared_ptr<CPS::AttributeBase> u20_base_ptr = u20_base.getPtr();
	// cout << "Input value to S2: " << *u20_base_ptr << endl;
	cout << "Input value to S2: " << u20Attr->toString() << endl;

	float u20 = u20Attr->get();

	Simulation sim2 = setDPsim2(timeStep, finalTime, u20);
	sim2.start();

	// Verify initialization
	AttributeBase::Ptr u20_base_test = sim2.getIdObjAttribute("i_in_2", "i_intf");
	cout << "Current value in S2: " << u20_base_test->toString() << endl;

	AttributeBase::Ptr y20_base = sim2.getIdObjAttribute("i_in_2", "v_intf");
	// auto y20_matrix = std::dynamic_pointer_cast<Attribute<Matrix>>(y20_base.getPtr());
	// Attribute<Real>::Ptr y20 = y20_matrix->deriveCoeff<Real>(0,0);
	// cout << "Output value from S2: " << y20->toString() << endl;
	cout << "Output value from S2: " << y20_base->toString() << endl;
	
	// Main loop
	float t = 0.0;

	while (t < finalTime) {
		t = sim1.next();

		cout << "t = " << t << endl;
		
		AttributeBase::Ptr y1_base = sim1.getIdObjAttribute("v_in_1", "i_intf");	
		auto y1_matrix = std::dynamic_pointer_cast<Attribute<Matrix>>(y1_base.getPtr());
		Attribute<Real>::Ptr y1 = y1_matrix->deriveCoeff<Real>(0,0);
		cout << "Output value from S1: " << **y1 << endl;
		// cout << "Output value from S1: " << y1->toString() << endl;
		// cout << "Output value from S1: " << y1_base->toString() << endl;

		// "Communication" y10 -> S_2 and initialization of S_2
		// AttributeBase::Ptr u2_base = y1->cloneValueOntoNewAttribute();
		// auto u2Attr = std::dynamic_pointer_cast<Attribute<Real>>(u2_base.getPtr());
		// cout << "Input value to S2: " << u2Attr->toString() << endl;

		// Get corresponding attribute in S_2
		AttributeBase::Ptr u2_base = sim2.getIdObjAttribute("i_in_2", "i_intf");
		auto u2_matrix = std::dynamic_pointer_cast<Attribute<Matrix>>(u2_base.getPtr());
		Attribute<Real>::Ptr u2 = u2_matrix->deriveCoeff<Real>(0,0);
		cout << "Current value in S2: " << **u2 << endl;
		// cout << "Current value in S2: " << u2->toString() << endl;
		// cout << "Current value in S2: " << u2_base->toString() << endl;
		
		// Put value
		// *u2_base = *y1_base;
		// *u2_base.getPtr() = *y1_base.getPtr();
		
		// This way doesn't work
		// **u2 = 27.2727;
		// Real u2_test = **u2;

		// this way does work
		u2->set(**y1);
		Real u2_test2 = **u2;

		// Verify
		// u2_matrix = std::dynamic_pointer_cast<Attribute<Matrix>>(u2_base.getPtr());
		// u2 = u2_matrix->deriveCoeff<Real>(0,0);
		// cout << "Input value to S2: " << u2_test << endl;
		cout << "Input value to S2 (option 2): " << u2_test2 << endl;

		AttributeBase::Ptr u2_base_test = sim2.getIdObjAttribute("i_in_2", "i_intf");
		// auto u2_matrix_test = std::dynamic_pointer_cast<Attribute<Matrix>>(u2_base_test.getPtr());
		// Attribute<Real>::Ptr u2_test = u2_matrix_test->deriveCoeff<Real>(0,0);
		// cout << "Current value in S2: " << u2_test->toString() << endl;
		cout << "Current value in S2: " << u2_base_test->toString() << endl;
		
		sim2.next();

		AttributeBase::Ptr y2_base = sim2.getIdObjAttribute("i_in_2", "v_intf");	
		auto y2_matrix = std::dynamic_pointer_cast<Attribute<Matrix>>(y2_base.getPtr());
		Attribute<Real>::Ptr y2 = y2_matrix->deriveCoeff<Real>(0,0);
		cout << "Output value from S2: " << **y2 << endl;
		// cout << "Output value from S2: " << y2->toString() << endl;
		// cout << "Output value from S2: " << y2_base->toString() << endl;

		// Get corresponding attribute in S_1
		AttributeBase::Ptr u1_base = sim1.getIdObjAttribute("v_in_1", "v_intf");
		auto u1_matrix = std::dynamic_pointer_cast<Attribute<Matrix>>(u1_base.getPtr());
		Attribute<Real>::Ptr u1 = u1_matrix->deriveCoeff<Real>(0,0);
		cout << "Current value in S1: " << u1->toString() << endl;

		// Put value
		// *u1_base = *y2_base;

		u1->set(**y2);
		Real u1_test2 = **u1;

		// Verify
		// u1_matrix = std::dynamic_pointer_cast<Attribute<Matrix>>(u1_base.getPtr());
		// u1 = u1_matrix->deriveCoeff<Real>(0,0);
		// cout << "Input value to S1: " << u1->toString() << endl;
		cout << "Input value to S1 (option 2): " << u1_test2 << endl;

		AttributeBase::Ptr u1_base_test = sim1.getIdObjAttribute("v_in_1", "v_intf");
		// auto u2_matrix_test = std::dynamic_pointer_cast<Attribute<Matrix>>(u2_base_test.getPtr());
		// Attribute<Real>::Ptr u2_test = u2_matrix_test->deriveCoeff<Real>(0,0);
		// cout << "Current value in S2: " << u2_test->toString() << endl;
		cout << "Current value in S1: " << u1_base_test->toString() << endl;

	}

	// Get only works for the typed ones
	// AttributeBase::Ptr attr = AttributeStatic<Real>::make(0.001);
	
	// Attribute<Real>::Ptr attr = AttributeStatic<Real>::make(0.001);
	// Real read1 = attr->get();
	// cout << read1 << endl;

	// Set up subsystem 2
	// Simulation sim1 = setDPsim1(timeStep, finalTime, y20);
	// sim1.start();

	sim1.stop();
	sim2.stop();

	// if (String(argv[1]) == "0") {


	// 	sim.run();
	// }
	// else if (String(argv[1]) == "1") {
	// 	String simName = "SocketsCosim_example2";
	// 	Logger::setLogDir("logs/"+simName);

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

	return 0;
}
