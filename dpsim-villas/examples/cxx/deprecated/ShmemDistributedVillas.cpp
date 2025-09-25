// SPDX-License-Identifier: Apache-2.0

#include <DPsim.h>
#include <dpsim-villas/Interfaces.h>

using namespace DPsim;
using namespace CPS::DP;
using namespace CPS::DP::Ph1;

int main(int argc, char *argv[]) {
  SystemComponentList comps, comps2;
  SystemNodeList nodes;

  if (argc < 2) {
    std::cerr << "Not enough arguments (either 0 or 1 for the test number)"
              << std::endl;
    std::exit(1);
  }

  String in, out;

  if (String(argv[1]) == "0") {
    in = "/villas0-in";
    out = "/villas0-out";
  } else if (String(argv[1]) == "1") {
    in = "/villas1-in";
    out = "/villas1-out";
  }

  const auto shmemConfig = fmt::format(
      R"STRING(
    {{
      "type": "shmem",
      "in": {{
        "name": "{}"
      }},
      "out": {{
        "name": "{}"
      }},
      "queuelen": 1024
    }})STRING",
      in, out);

  auto intf = std::make_shared<InterfaceVillas>(shmemConfig);

  if (String(argv[1]) == "0") {
    // Nodes
    auto n1 = SimNode::make("n1");
    auto n2 = SimNode::make("n2");
    auto n3 = SimNode::make("n3");

    // Components
    auto evs = VoltageSource::make("v_t");
    auto vs = VoltageSourceNorton::make("v_s");
    auto l1 = Inductor::make("l_1");
    auto r1 = Resistor::make("r_1");

    // Topology
    evs->connect({SimNode::GND, n3});
    vs->connect({SimNode::GND, n1});
    l1->connect({n1, n2});
    r1->connect({n2, n3});

    // Parameters
    evs->setParameters(Complex(0, 0));
    vs->setParameters(Complex(10000, 0), 1);
    l1->setParameters(0.1);
    r1->setParameters(1);

    comps = SystemComponentList{evs, vs, l1, r1};
    nodes = SystemNodeList{SimNode::GND, n1, n2, n3};

    intf->importAttribute(evs->mVoltageRef, 0, true);
    intf->exportAttribute(evs->mIntfCurrent->deriveCoeff<Complex>(0, 0), 0,
                          true);

  } else if (String(argv[1]) == "1") {
    // Nodes
    auto n4 = SimNode::make("n4");
    auto n5 = SimNode::make("n5");

    // Components
    auto ecs = CurrentSource::make("v_s");
    auto r2A = Resistor::make("r_2");
    auto r2B = Resistor::make("r_2");
    auto sw = Ph1::Switch::make("sw");

    // Topology
    ecs->connect({SimNode::GND, n4});
    r2A->connect({SimNode::GND, n4});
    sw->connect({n4, n5});
    r2B->connect({SimNode::GND, n5});

    // Parameters
    ecs->setParameters(Complex(0, 0));
    r2A->setParameters(10);
    r2B->setParameters(8);
    sw->setParameters(1e9, 0.1, false);

    comps = SystemComponentList{ecs, sw, r2A, r2B};
    nodes = SystemNodeList{SimNode::GND, n4, n5};

    intf->importAttribute(ecs->mCurrentRef, 0, true);
    intf->exportAttribute(ecs->mIntfVoltage->deriveCoeff<Complex>(0, 0), 0,
                          true);
  } else {
    std::cerr << "invalid test number" << std::endl;
    std::exit(1);
  }

  String simName = "ShmemDistributed";
  Real timeStep = 0.001;

  auto sys = SystemTopology(50, nodes, comps);

#ifdef WITH_RT
  RealTimeSimulation sim(simName + argv[1]);
#else
  Simulation sim(simName + argv[1]);
#endif
  sim.setSystem(sys);
  sim.setTimeStep(timeStep);
  sim.setFinalTime(20);
  sim.addInterface(intf);

  if (String(argv[1]) == "1") {
    auto evt = SwitchEvent::make(
        10, sys.component<CPS::Base::Ph1::Switch>("sw"), true);

    sim.addEvent(evt);
  }

#ifdef WITH_RT
  sim.run(10);
#else
  sim.run();
#endif

  return 0;
}
