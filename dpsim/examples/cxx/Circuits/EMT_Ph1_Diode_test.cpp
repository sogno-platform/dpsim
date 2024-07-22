#include <DPsim.h>
#include "../Examples.h"

using namespace DPsim;
using namespace CPS::EMT;


void EMT_Ph1_Diode(){
	// Define simulation scenario
	Real timeStep = 0.0001;
	Real finalTime = 0.1;
	String simName = "EMT_Ph1_Diode_test";
	Logger::setLogDir("logs/" + simName);

	// Nodes
	auto n1 = SimNode::make("n1", PhaseType::Single);
	auto n2 = SimNode::make("n1", PhaseType::Single);

	// Components

	auto vs0 = Ph1::VoltageSource::make("vs0");
	vs0->setParameters(CPS::Complex(1.,0.), 50.0);

	auto load = CPS::EMT::Ph1::Resistor::make("Load");
	load->setParameters(10.);

	auto diode = Ph1::Diode::make("Diode");
	
	// Topology
	load->connect(SimNode::List{ n1, n2 });

	diode->connect(SimNode::List{ n2, SimNode::GND }); //SimNode::GND, n2

	vs0->connect(SimNode::List{SimNode::GND, n1});

	// Define system topology
	auto sys = SystemTopology(50, SystemNodeList{n1, n2}, SystemComponentList{load, vs0, diode});

	// Logging
	auto logger = DataLogger::make(simName);
	logger->logAttribute("I_Diode", diode->attribute("i_intf"));
	logger->logAttribute("V_Diode", diode->attribute("v_intf"));

	Simulation sim(simName, Logger::Level::info);
	sim.doInitFromNodesAndTerminals(true);
	sim.setSystem(sys);
	sim.addLogger(logger);
	sim.setDomain(Domain::EMT);
	sim.setSolverType(Solver::Type::ITERATIVEMNA);
	sim.setDirectLinearSolverConfiguration(DirectLinearSolverConfiguration());
	sim.setTimeStep(timeStep);
	sim.setFinalTime(finalTime);
	sim.run();	
}


int main(){
	EMT_Ph1_Diode();
    return 0;
}