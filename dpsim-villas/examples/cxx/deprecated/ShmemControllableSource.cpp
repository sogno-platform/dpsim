// SPDX-License-Identifier: Apache-2.0

#include <DPsim.h>
#include <dpsim-villas/Interfaces.h>

using namespace DPsim;
using namespace CPS::DP;
using namespace CPS::DP::Ph1;

int main(int argc, char *argv[]) {
  Real timeStep = 0.001;
  Real finalTime = 10;
  String simName = "ShmemControllableSource";

  const std::string shmemConfig = R"STRING(
    {
      "type": "shmem",
      "in": {
        "name": "dpsim01"
      },
      "out": {
        "name": "dpsim10"
      },
      "queuelen": 1024
    })STRING";

  auto intf = std::make_shared<InterfaceVillas>(shmemConfig);

  // Nodes
  auto n1 = SimNode::make("n1");

  // Components
  auto ecs = CurrentSource::make("v_intf");
  ecs->setParameters(Complex(10, 0));
  auto r1 = Resistor::make("r_1");
  r1->setParameters(1);

  ecs->connect({SimNode::GND, n1});
  r1->connect({SimNode::GND, n1});

  intf->importAttribute(ecs->mCurrentRef, 0, true);
  intf->exportAttribute(ecs->mIntfVoltage->deriveCoeff<Complex>(0, 0), 0, true);

  auto sys =
      SystemTopology(50, SystemNodeList{n1}, SystemComponentList{ecs, r1});

#ifdef WITH_RT
  RealTimeSimulation sim(simName, CPS::Logger::Level::info);
#else
  Simulation sim(simName, CPS::Logger::Level::info);
#endif
  sim.setSystem(sys);
  sim.setTimeStep(timeStep);
  sim.setFinalTime(finalTime);
  sim.setDomain(Domain::DP);
  sim.setSolverType(Solver::Type::MNA);

  sim.addInterface(intf);
#ifdef WITH_RT
  sim.run(10);
#else
  sim.run();
#endif

  return 0;
}
