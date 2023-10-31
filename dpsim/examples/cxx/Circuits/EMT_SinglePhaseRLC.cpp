#include <DPsim.h>

using namespace DPsim;
using namespace CPS::EMT;
using namespace CPS::EMT::Ph1;

int main(int argc, char* argv[]) {
	Real timeStep = 1e-4;
	Real finalTime = 0.5;
	String simName = "EMT_SinglePhaseRLC";
	Logger::setLogDir("logs/"+simName);

	// Nodes
	auto n1 = SimNode::make("n1");
	auto n2 = SimNode::make("n2");
	auto n3 = SimNode::make("n3");

	// Components
	auto vs = VoltageSource::make("vs");
	vs->setParameters(Complex(100, 0));
	auto r = Resistor::make("r", Logger::Level::info);
	r->setParameters(100);
	auto l = Inductor::make("l", Logger::Level::info);
	l->setParameters(5);
	auto c = Capacitor::make("c", Logger::Level::info);
	c->setParameters(250e-6);

	// Connections
	vs->connect(SimNode::List{ SimNode::GND, n3 });
	r->connect(SimNode::List{ n3, n1 });
	l->connect(SimNode::List{ n1, n2 });
	c->connect(SimNode::List{ n2, SimNode::GND});

	// Define system topology
	auto sys = SystemTopology(50,
		SystemNodeList{n1, n2, n3},
		SystemComponentList{vs, r, l, c});

	// Logger
	auto logger = DataLogger::make(simName);
	logger->logAttribute("v1", n1->attribute("v"));
	logger->logAttribute("v2", n2->attribute("v"));
	logger->logAttribute("i", r->attribute("i_intf"));

	Simulation sim(simName);
	sim.setSystem(sys);
	sim.setTimeStep(timeStep);
	sim.setFinalTime(finalTime);
	sim.setDomain(Domain::EMT);
	sim.addLogger(logger);
	sim.extractEigenvalues();
	
	sim.run();

	return 0;
}
