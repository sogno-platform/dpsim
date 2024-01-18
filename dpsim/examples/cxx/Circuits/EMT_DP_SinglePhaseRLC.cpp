#include <DPsim.h>

using namespace DPsim;
using namespace CPS;

void runEMT()
{
	Real timeStep = 1e-4;
	Real finalTime = 0.5;
	String simName = "EMT_SinglePhaseRLC";
	Logger::setLogDir("logs/" + simName);

	// Nodes
	auto n1 = SimNode<Real>::make("n1");
	auto n2 = SimNode<Real>::make("n2");
	auto n3 = SimNode<Real>::make("n3");

	// Components
	auto vs = EMT::Ph1::VoltageSource::make("vs");
	vs->setParameters(Complex(100, 0));
	auto r = EMT::Ph1::Resistor::make("r", Logger::Level::info);
	r->setParameters(100);
	auto l = EMT::Ph1::Inductor::make("l", Logger::Level::info);
	l->setParameters(5);
	auto c = EMT::Ph1::Capacitor::make("c", Logger::Level::info);
	c->setParameters(250e-6);

	// Connections
	vs->connect(SimNode<Real>::List{SimNode<Real>::GND, n3});
	r->connect(SimNode<Real>::List{n3, n1});
	l->connect(SimNode<Real>::List{n1, n2});
	c->connect(SimNode<Real>::List{n2, SimNode<Real>::GND});

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
	sim.setEigenvalueExtractionMode(EigenvalueExtractionMode::AtStart);
	sim.addLogger(logger);

	sim.run();
}

void runDP()
{
	Real timeStep = 1e-4;
	Real finalTime = 0.5;
	String simName = "DP_SinglePhaseRLC";
	Logger::setLogDir("logs/" + simName);

	// Nodes
	auto n1 = SimNode<Complex>::make("n1");
	auto n2 = SimNode<Complex>::make("n2");
	auto n3 = SimNode<Complex>::make("n3");

	// Components
	auto vs = DP::Ph1::VoltageSource::make("vs");
	vs->setParameters(Complex(100, 0));
	auto r = DP::Ph1::Resistor::make("r", Logger::Level::info);
	r->setParameters(100);
	auto l = DP::Ph1::Inductor::make("l", Logger::Level::info);
	l->setParameters(5);
	auto c = DP::Ph1::Capacitor::make("c", Logger::Level::info);
	c->setParameters(250e-6);

	// Connections
	vs->connect(SimNode<Complex>::List{SimNode<Complex>::GND, n3});
	r->connect(SimNode<Complex>::List{n3, n1});
	l->connect(SimNode<Complex>::List{n1, n2});
	c->connect(SimNode<Complex>::List{n2, SimNode<Complex>::GND});

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
	sim.setDomain(Domain::DP);
	sim.setEigenvalueExtractionMode(EigenvalueExtractionMode::AtStart);
	sim.addLogger(logger);	

	sim.run();
}

int main(int argc, char *argv[])
{
	runEMT();
	runDP();

	return 0;
}
