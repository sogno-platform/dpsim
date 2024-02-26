// SPDX-License-Identifier: Apache-2.0

#include <DPsim.h>
#include <dpsim-villas/InterfaceShmem.h>

using namespace DPsim;
using namespace CPS::DP;
using namespace CPS::DP::Ph1;

int main(int argc, char *argv[]) {
  Real timeStep = 0.001;
  Real finalTime = 10;
  String simName = "ShmemControllableSource";

  InterfaceShmem intf("/dpsim01", "/dpsim10");

  // Nodes
  auto n1 = SimNode::make("n1");

  // Components
  auto ecs = CurrentSource::make("v_intf");
  ecs->setParameters(Complex(10, 0));
  auto r1 = Resistor::make("r_1");
  r1->setParameters(1);

  ecs->connect({SimNode::GND, n1});
  r1->connect({SimNode::GND, n1});

  intf.importAttribute(ecs->mCurrentRef, 0);
  intf.exportAttribute(ecs->mIntfVoltage->deriveCoeff<Complex>(0, 0), 0);

  auto sys =
      SystemTopology(50, SystemNodeList{n1}, SystemComponentList{ecs, r1});

  RealTimeSimulation sim(simName, CPS::Logger::Level::info);
  sim.setSystem(sys);
  sim.setTimeStep(timeStep);
  sim.setFinalTime(finalTime);
  sim.setDomain(Domain::DP);
  sim.setSolverType(Solver::Type::MNA);

  sim.addInterface(std::shared_ptr<Interface>(&intf));
  sim.run();

  return 0;
}
