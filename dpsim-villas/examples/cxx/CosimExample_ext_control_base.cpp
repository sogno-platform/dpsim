// SPDX-License-Identifier: Apache-2.0

#include <fstream>
#include <iostream>

#include <DPsim.h>
#include <dpsim-villas/InterfaceVillas.h>

using namespace DPsim;
using namespace CPS;
using namespace std;

int main(int argc, char* argv[]) {

	float r1_r = 0.1;
	float c1_c = 1;
	float rLine_r = 0.1;
	float r3_r = 1;
	float c2_c = 1;
	
	// ----- DYNAMIC SIMULATION -----
	String simName = "Cosim_example_base";
	CPS::Logger::setLogDir("logs/"+simName);

	// Nodes
	auto n1 = EMT::SimNode::make("n1");
	auto n2 = EMT::SimNode::make("n2");

	// Components
	auto r1 =  EMT::Ph1::Resistor::make("r_1");
	r1->setParameters(r1_r);
	auto c1 = EMT::Ph1::Capacitor::make("c_1");
	c1->setParameters(c1_c);
	auto rLine = EMT::Ph1::Resistor::make("r_line");
	rLine->setParameters(rLine_r);
	auto r3 =  EMT::Ph1::Resistor::make("r_3");
	r3->setParameters(r3_r);
	auto c2 = EMT::Ph1::Capacitor::make("c_2");
	c2->setParameters(c2_c);

	// Topology
	r1->connect({ n1, EMT::SimNode::GND });
	rLine->connect({ n1, n2 });
	c1->connect({ n1, EMT::SimNode::GND });
	r3->connect({ n2, EMT::SimNode::GND });
	c2->connect({ n2, EMT::SimNode::GND });

	auto sys = SystemTopology(50,
		SystemNodeList{EMT::SimNode::GND, n1, n2},
		SystemComponentList{r1, c1, rLine, c2, r3});

	auto logger = DataLogger::make(simName);
	logger->logAttribute("1_v_1", n1->mVoltage);
	logger->logAttribute("2_v_2", n2->mVoltage);
	logger->logAttribute("3_i_rline", rLine->mIntfCurrent, 1, 1);
	
	Simulation sim(simName);
	sim.setDomain(CPS::Domain::EMT);
	sim.addLogger(logger);
	sim.setSystem(sys);
	sim.setTimeStep(0.01);
	sim.setFinalTime(1.0);
	// sim.doSteadyStateInit(true);
	sim.doInitFromNodesAndTerminals(false);

	// initialize currents and voltages
	// Eigen::VectorXd n_v0(2);
	// n_v0 << 5, 5;

	// Initial conditions, given by the problem
	Eigen::MatrixXd n1_v0(1,1);
	n1_v0(0,0) = 5.0;
	Eigen::MatrixXd n2_v0(1,1);
	n2_v0(0,0) = 2.0;

	Eigen::MatrixXd ir1_0(1,1);
	ir1_0(0,0) = -n1_v0(0,0) / r1_r;
	Eigen::MatrixXd irLine_0(1,1);
	irLine_0(0,0) = (n1_v0(0,0) - n2_v0(0,0)) / rLine_r;

	Eigen::MatrixXd ir3_0(1,1);
	ir3_0(0,0) = n2_v0(0,0) / r3_r;

	Eigen::MatrixXd ic1_0 = ir1_0 - irLine_0;
	Eigen::MatrixXd vrLine_0 = n1_v0 - n2_v0;
	Eigen::MatrixXd ic2_0 = irLine_0 - ir3_0;

	r1->setIntfVoltage(n1_v0);
	r1->setIntfCurrent(ir1_0);
	c1->setIntfVoltage(n1_v0);
	c1->setIntfCurrent(ic1_0);
	rLine->setIntfVoltage(vrLine_0);
	rLine->setIntfCurrent(irLine_0);
	r3->setIntfVoltage(n2_v0);
	r3->setIntfCurrent(ir3_0);
	c2->setIntfVoltage(n2_v0);
	c2->setIntfCurrent(ic2_0);

	sim.run();
}
