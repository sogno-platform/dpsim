#include <DPsim.h>

using namespace DPsim;
using namespace CPS;

int main() {
  constexpr Real timeStep = 50e-6;
  constexpr Real finalTime = 5.0;
  const String simName = "SP_Ph3_Resistor_Optimization";

  auto node = SimNode<Complex>::make("bus", PhaseType::ABC,
                                     std::vector<Complex>{Complex(100e3, 0),
                                                          Complex(100e3, 0),
                                                          Complex(100e3, 0)});

  auto source = SP::Ph3::VoltageSource::make("source", Logger::Level::off);
  source->setParameters(Complex(100e3, 0));
  source->connect({SimNode<Complex>::GND, node});

  auto resistor = SP::Ph3::Resistor::make("resistor", Logger::Level::off);
  resistor->setParameters(Matrix::Identity(3, 3) * 10.0);
  resistor->connect({node, SimNode<Complex>::GND});

  auto system = SystemTopology(50.0, SystemNodeList{node},
                               SystemComponentList{source, resistor});
  auto logger = DataLogger::make(simName, Logger::Level::off);
  logger->logAttribute("voltage", resistor->attribute("v_intf"));
  logger->logAttribute("current", resistor->attribute("i_intf"));

  Simulation simulation(simName, Logger::Level::off);
  simulation.setSystem(system);
  simulation.setDomain(Domain::SP);
  simulation.setTimeStep(timeStep);
  simulation.setFinalTime(finalTime);
  simulation.addLogger(logger);
  simulation.run();

  return 0;
}
