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
  // Nodes
  auto n1 = SimNode::make("n1");
  auto n2 = SimNode::make("n2");
  auto n3 = SimNode::make("n3");

  // Components
  auto v1 = VoltageSource::make("v_1");
  //	auto l1 = Inductor::make("l_1");
  //	auto r2 = Resistor::make("r_2");
  auto t1 = Transformer::make("trafo_1");
  auto r1 = Resistor::make("r_1");

  // Topology
  v1->connect({SimNode::GND, n1});
  //	l1->connect({ n1, n2 });
  //	r2->connect({ n2, SimNode::GND });
  t1->connect({n1, n2});
  r1->connect({n2, SimNode::GND});

  // Parameters
  v1->setParameters(CPS::Math::polarDeg(100., 0 * -90.));
  //	l1->setParameters(0.1);
  //	r2->setParameters(1);
  t1->setParameters(10, 0, 0, 0.1);
  r1->setParameters(1);

  // Define system topology
  SystemTopology system(50, SystemNodeList{n1, n2, n3, SimNode::GND},
                        SystemComponentList{v1, t1, r1});

  // Define simulation scenario
  Real timeStep = 0.00005;
  Real finalTime = 0.2;
  String simName = "DP_IdealVS_Trafo_" + std::to_string(timeStep);

  Simulation sim(simName, system, timeStep, finalTime);
  sim.run();

  return 0;
}
