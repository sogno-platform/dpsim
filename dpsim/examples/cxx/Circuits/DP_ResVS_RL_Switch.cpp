/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <DPsim.h>

using namespace DPsim;
using namespace CPS::DP::Ph1;

int main(int argc, char *argv[]) {
  // Nodes
  auto n1 = SimNode::make("n1");
  auto n2 = SimNode::make("n2");
  auto n3 = SimNode::make("n3");

  // Components
  auto vs = VoltageSourceNorton::make("v_s");
  auto rl = Resistor::make("r_line");
  auto ll = Inductor::make("l_line");

  // Topology
  vs->connect({n1, GND});
  rl->connect({n1, n2});
  ll->connect({n2, n3});

  // Parameters
  vs->setParameters(Complex(10000, 0), 1);
  rl->setParameters(1) ll->setParameters(1)

      // Define system topology
      SystemTopology system0(50, SystemTopologyNode{n1, n2, n3, GND},
                             SystemTopologyComponent{vs, rl, ll});

  SystemTopology system1 = system0;
  SystemTopology system2 = system0;
  system1.addComponent(Resistor::make("r_load", 3, 0, 1000));
  system2.addComponent(Resistor::make("r_load", 3, 0, 800));

  // Define simulation scenario
  Real timeStep = 0.001;
  Real finalTime = 0.3;
  String simName = "DP_ResVC_RxLine1_" + std::to_string(timeStep);

  Simulation sim(simName, system1, timeStep, finalTime);
  sim.addSystemTopology(system2);
  sim.setSwitchTime(0.1, 1);

  sim.run();

  return 0;
}
