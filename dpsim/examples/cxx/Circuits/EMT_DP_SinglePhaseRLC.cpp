#include <DPsim.h>

using namespace DPsim;
using namespace CPS;

void runEMT(Real resistance) {
  Real timeStep = 1e-4;
  Real finalTime = 1e-3;
  String simName =
      "EMT_SinglePhaseRLC_resistance_" + std::to_string(resistance) + "_Ohm";
  Logger::setLogDir("logs/" + simName);

  // Nodes
  auto n1 = SimNode<Real>::make("n1");
  auto n2 = SimNode<Real>::make("n2");
  auto n3 = SimNode<Real>::make("n3");

  // Components
  auto vs = EMT::Ph1::VoltageSource::make("vs");
  vs->setParameters(Complex(100, 0));
  auto r = EMT::Ph1::Resistor::make("r", Logger::Level::info);
  r->setParameters(resistance);
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
  auto sys = SystemTopology(50, SystemNodeList{n1, n2, n3},
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
  sim.doEigenvalueExtraction(true);
  sim.addLogger(logger);

  sim.run();
}

void runDP() {
  Real timeStep = 1e-4;
  Real finalTime = 1e-3;
  String simName = "DP_SinglePhaseRLC";
  Logger::setLogDir("logs/" + simName);

  // Nodes
  auto n1 = SimNode<Complex>::make("n1");
  auto n2 = SimNode<Complex>::make("n2");
  auto n3 = SimNode<Complex>::make("n3");
  auto n4 = SimNode<Complex>::make("n2");
  auto n5 = SimNode<Complex>::make("n3");

  // Components
  auto vs = DP::Ph1::VoltageSource::make("vs");
  vs->setParameters(Complex(100, 0));
  auto r = DP::Ph1::Resistor::make("r", Logger::Level::info);
  r->setParameters(100);
  auto r2 = DP::Ph1::Resistor::make("r2", Logger::Level::info);
  r2->setParameters(1e-3);
  auto l = DP::Ph1::Inductor::make("l", Logger::Level::info);
  l->setParameters(5);
  auto c = DP::Ph1::Capacitor::make("c", Logger::Level::info);
  c->setParameters(250e-6);
  auto s = DP::Ph1::Switch::make("s", Logger::Level::info);
  s->setParameters(1e8, 1e-4, true);
  auto s2 = DP::Ph1::Switch::make("s2", Logger::Level::info);
  s2->setParameters(1e8, 1e-4, false);

  // Switch events
  auto sEvent = SwitchEvent::make(5e-4, s, false);
  auto s2Event = SwitchEvent::make(5e-4, s2, true);

  // Connections
  vs->connect(SimNode<Complex>::List{SimNode<Complex>::GND, n1});
  r->connect(SimNode<Complex>::List{n4, SimNode<Complex>::GND});
  r2->connect(SimNode<Complex>::List{n5, SimNode<Complex>::GND});
  l->connect(SimNode<Complex>::List{n1, n2});
  c->connect(SimNode<Complex>::List{n2, n3});
  s->connect(SimNode<Complex>::List{n3, n4});
  s2->connect(SimNode<Complex>::List{n3, n5});

  // Define system topology
  auto sys = SystemTopology(50, SystemNodeList{n1, n2, n3, n4, n5},
                            SystemComponentList{vs, r, r2, l, c, s, s2});

  // Logger
  auto logger = DataLogger::make(simName);
  logger->logAttribute("v1", n1->attribute("v"));
  logger->logAttribute("v2", n2->attribute("v"));
  logger->logAttribute("ir", r->attribute("i_intf"));
  logger->logAttribute("ir1", r2->attribute("i_intf"));

  Simulation sim(simName);
  sim.setSystem(sys);
  sim.addEvent(sEvent);
  sim.addEvent(s2Event);
  sim.setTimeStep(timeStep);
  sim.setFinalTime(finalTime);
  sim.setDomain(Domain::DP);
  sim.doEigenvalueExtraction(true);
  sim.addLogger(logger);

  sim.run();
}

int main(int argc, char *argv[]) {
  runEMT(100);
  runEMT(1.1e-3);
  runDP();

  return 0;
}
