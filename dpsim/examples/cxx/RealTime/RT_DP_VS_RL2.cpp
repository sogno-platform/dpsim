/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <DPsim.h>

using namespace DPsim;
using namespace CPS::DP;
using namespace CPS::DP::Ph1;

int main(int argc, char *argv[]) {
  Real timeStep = 0.0001;
  Real finalTime = 1;
  String simName = "RT_DP_VS_RL2";
  Logger::setLogDir("logs/" + simName);

  // Nodes
  auto n1 = SimNode::make("n1");
  auto n2 = SimNode::make("n2");
  auto n3 = SimNode::make("n3");

  // Components
  auto vs = VoltageSource::make("v_s");
  vs->setParameters(Complex(1000, 0));
  auto rl = Resistor::make("r_line");
  rl->setParameters(1);
  auto ll = Inductor::make("l_line");
  ll->setParameters(0.01);
  auto rL = Resistor::make("r_load");
  rL->setParameters(100);

  // Connections
  vs->connect({SimNode::GND, n1});
  rl->connect({n1, n2});
  ll->connect({n2, n3});
  rL->connect({SimNode::GND, n3});

  auto sys = SystemTopology(50, SystemNodeList{SimNode::GND, n1, n2, n3},
                            SystemComponentList{vs, rl, ll, rL});

  RealTimeSimulation sim(simName);
  sim.setSystem(sys);
  sim.setTimeStep(timeStep);
  sim.setFinalTime(finalTime);

  auto startIn = std::chrono::seconds(5);
  sim.run(startIn);

  return 0;
}
