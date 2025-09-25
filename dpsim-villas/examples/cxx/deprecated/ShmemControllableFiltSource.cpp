// SPDX-License-Identifier: Apache-2.0

#include <DPsim.h>
#include <dpsim-villas/Interfaces.h>

using namespace DPsim;
using namespace CPS::Signal;
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

  // Controllers and filter
  std::vector<Real> coefficients = {
      -0.0024229, -0.0020832, 0.0067703, 0.016732,  0.011117,  -0.0062311,
      -0.0084016, 0.0092568,  0.012983,  -0.010121, -0.018274, 0.011432,
      0.026176,   -0.012489,  -0.037997, 0.013389,  0.058155,  -0.014048,
      -0.10272,   0.014462,   0.31717,   0.48539,   0.31717,   0.014462,
      -0.10272,   -0.014048,  0.058155,  0.013389,  -0.037997, -0.012489,
      0.026176,   0.011432,   -0.018274, -0.010121, 0.012983,  0.0092568,
      -0.0084016, -0.0062311, 0.011117,  0.016732,  0.0067703, -0.0020832,
      -0.0024229};

  auto filtP =
      FIRFilter::make("filter_p", coefficients, 10, CPS::Logger::Level::debug);
  auto filtQ =
      FIRFilter::make("filter_q", coefficients, 0, CPS::Logger::Level::debug);

  // Nodes
  auto n1 = SimNode::make("n1");

  // Components
  auto ecs = CurrentSource::make("v_intf");
  ecs->setParameters(Complex(10, 0));
  auto r1 = Resistor::make("r_1");
  r1->setParameters(1);
  auto load = PQLoadCS::make("load_cs");
  load->setParameters(10., 0., 10.);

  ecs->connect({SimNode::GND, n1});
  r1->connect({SimNode::GND, n1});
  load->connect({n1});

  load->mActivePower->setReference(filtP->mOutput);
  load->mReactivePower->setReference(filtQ->mOutput);

  auto filtPInput = CPS::AttributeDynamic<Real>::make(0.0);
  intf->importAttribute(filtPInput, 0, true);
  filtP->setInput(filtPInput);

  auto filtQInput = CPS::AttributeDynamic<Real>::make(0.0);
  intf->importAttribute(filtQInput, 1, true);
  filtQ->setInput(filtQInput);

  auto sys = SystemTopology(50, SystemNodeList{n1},
                            SystemComponentList{ecs, r1, load, filtP, filtQ});

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
  sim.run();

  return 0;
}
