#include <DPsim.h>

using namespace DPsim;
using namespace CPS::EMT;


void EMT_PH3_R3_C1_L1_CS()
{
	// Define simulation scenario
	Real timeStep = 0.0001;
	Real finalTime = 0.1;
	String simName = "EMT_Ph3_LinearSampleCircuit";

	// Nodes
	auto n1 = SimNode::make("n1", PhaseType::ABC);
	auto n2 = SimNode::make("n2", PhaseType::ABC);

	// Components

    Matrix param = Matrix::Zero(3, 3);
	param <<
		1., 0, 0,
		0, 1., 0,
		0, 0, 1.;


	auto cs0 = Ph3::CurrentSource::make("cs0");
	cs0->setParameters(CPS::Math::singlePhaseVariableToThreePhase(CPS::Math::polar(1.0,0.0)),50.0);

	auto r1 = Ph3::Resistor::make("r1", Logger::Level::debug);
	r1->setParameters(10*param);

    auto r2 = Ph3::Resistor::make("r2", Logger::Level::debug);
    r2->setParameters(param);

    auto r3 = Ph3::Resistor::make("r3", Logger::Level::debug);
    r3->setParameters(5*param);

	auto l1 = Ph3::Inductor::make("l1");
	l1->setParameters(0.02 * param);

	auto c1 = Ph3::Capacitor::make("c1");
    c1->setParameters(0.001 * param);

	// Topology
	cs0->connect(SimNode::List{ n1, SimNode::GND });

	r1->connect(SimNode::List{ n2, n1 });
    r2->connect(SimNode::List{ n2, SimNode::GND });
    r3->connect(SimNode::List{ n2, SimNode::GND });

	l1->connect(SimNode::List{ n2, SimNode::GND });

	c1->connect(SimNode::List{ n1, n2 });

	// Define system topology
	auto sys = SystemTopology(50, SystemNodeList{n1, n2}, SystemComponentList{cs0, r1, r2, r3, l1, c1});

	// Logging
	Logger::setLogDir("logs/" + simName);
	auto logger = DataLogger::make(simName);
	logger->logAttribute("v1", n1->attribute("v"));
	logger->logAttribute("v2", n2->attribute("v"));
	logger->logAttribute("v_c1", c1->attribute("v_intf"));
	logger->logAttribute("i_L1", l1->attribute("i_intf"));

	Simulation sim(simName, Logger::Level::info);
	sim.setSystem(sys);
	sim.addLogger(logger);
	sim.setDomain(Domain::EMT);
	sim.setTimeStep(timeStep);
	sim.setFinalTime(finalTime);
	sim.run();
}


void EMT_Ph1_RLC_VS()
{
	// Define simulation scenario
	Real timeStep = 0.0001;
	Real finalTime = 1.;
	String simName = "EMT_Ph3_LinearSampleCircuit";

	// Nodes
	auto n1 = SimNode::make("n1", PhaseType::Single);
	auto n2 = SimNode::make("n2", PhaseType::Single);
	auto n3 = SimNode::make("n3", PhaseType::Single);

	// Components

	Real param = 1.;

	auto vs0 = Ph1::VoltageSource::make("vs0");
	vs0->setParameters(CPS::Math::polar(1.0,0.0),50.0);

	auto r1 = Ph1::Resistor::make("r1", Logger::Level::debug);
	r1->setParameters(10.*param);

	auto l1 = Ph1::Inductor::make("l1");
	l1->setParameters(0.02 * param);

	auto c1 = Ph1::Capacitor::make("c1");
    c1->setParameters(0.001 * param);

	// Topology
	vs0->connect(SimNode::List{ n1, SimNode::GND });

	r1->connect(SimNode::List{ n1, n2 });

	l1->connect(SimNode::List{ n2, n3 });

	c1->connect(SimNode::List{ n3, SimNode::GND });

	// Define system topology
	auto sys = SystemTopology(50, SystemNodeList{n1, n2, n3}, SystemComponentList{vs0, r1, l1, c1});

	// Logging
	Logger::setLogDir("logs/" + simName);
	auto logger = DataLogger::make(simName);
	logger->logAttribute("I_R", r1->attribute("i_intf"));

	Simulation sim(simName, Logger::Level::info);
	sim.setSystem(sys);
	sim.addLogger(logger);
	sim.setDomain(Domain::EMT);
	sim.setTimeStep(timeStep);
	sim.setFinalTime(finalTime);
	sim.run();
}


void EMT_Ph3_RLC_VS()
{
	// Define simulation scenario
	Real timeStep = 0.0001;
	Real finalTime = 0.1;
	String simName = "EMT_Ph3_LinearSampleCircuit";

	// Nodes
	auto n1 = SimNode::make("n1", PhaseType::ABC);
	auto n2 = SimNode::make("n2", PhaseType::ABC);
	auto n3 = SimNode::make("n3", PhaseType::ABC);

	// Components

    Matrix param = Matrix::Zero(3, 3);
	param <<
		1., 0, 0,
		0, 1., 0,
		0, 0, 1.;


	auto vs0 = Ph3::VoltageSource::make("vs0");
	vs0->setParameters(CPS::Math::singlePhaseVariableToThreePhase(CPS::Math::polar(1.0,0.0)),50.0);

	auto r1 = Ph3::Resistor::make("r1", Logger::Level::debug);
	r1->setParameters(1.*param);

	auto l1 = Ph3::Inductor::make("l1");
	l1->setParameters(0.05 * param);

	auto c1 = Ph3::Capacitor::make("c1");
    c1->setParameters(0.01 * param);

	// Topology
	vs0->connect(SimNode::List{ n1, SimNode::GND });

	r1->connect(SimNode::List{ n1, n2 });

	l1->connect(SimNode::List{ n2, n3 });

	c1->connect(SimNode::List{ n3, SimNode::GND });

	// Define system topology
	auto sys = SystemTopology(50, SystemNodeList{n1, n2, n3}, SystemComponentList{vs0, r1, l1, c1});

	// Logging
	Logger::setLogDir("logs/" + simName);
	auto logger = DataLogger::make(simName);
	logger->logAttribute("I_R", r1->attribute("i_intf"));

	Simulation sim(simName, Logger::Level::info);
	sim.setSystem(sys);
	sim.addLogger(logger);
	sim.setDomain(Domain::EMT);
	sim.setTimeStep(timeStep);
	sim.setFinalTime(finalTime);
	sim.run();
}


int main(int argc, char* argv[])
{
   	//EMT_PH3_R3_C1_L1_CS();

	//EMT_Ph1_RLC_VS();

	EMT_Ph3_RLC_VS();
}
