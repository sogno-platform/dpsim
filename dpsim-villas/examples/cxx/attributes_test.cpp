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

	// Initial conditions, given by the problem
	Eigen::MatrixXd n1_v0(1,1);
	n1_v0(0,0) = 5.0;
	Eigen::MatrixXd n2_v0(1,1);
	n2_v0(0,0) = u10;

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

	evs->setIntfVoltage(n2_v0);
	evs->setIntfCurrent(irLine_0);

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

	Attribute<Real>::Ptr attr = AttributeStatic<Real>::make(0.001);

	Real read1 = **attr; //read1 = 0.001
	cout << "attr value: " << read1 << endl;

	**attr = 0.002;
	Real read2 = **attr; //read2 = 0.002
	cout << "attr value: " << read2 << endl;
		
	attr->set(0.003);
	Real read3 = **attr; //read3 = 0.003
	cout << "attr value: " << read3 << endl;

	// ** Initialization **
	float timeStep = 0.01;
	float finalTime = 0.05;

	Simulation sim1 = setDPsim1(timeStep, finalTime, 2.0);
	sim1.start();

	AttributeBase::Ptr y10_base = sim1.getIdObjAttribute("v_in_1", "i_intf");
	auto y10_matrix = std::dynamic_pointer_cast<Attribute<Matrix>>(y10_base.getPtr());
	Attribute<Real>::Ptr y10 = y10_matrix->deriveCoeff<Real>(0,0);
	cout << "Output value from S1: " << **y10 << endl;

	**y10 = 27.2727;
	Real u2_test = **y10;
	cout << "New y10 value: " << u2_test << endl;

	y10->set(27.2727);
	Real u2_test2 = **y10;
	cout << "New y10 value: " << u2_test2 << endl;

	return 0;
}