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
  Real timeStep = 0.00005;

  // Nodes
  auto n1 = SimNode::make("n1");
  auto n2 = SimNode::make("n2");
  //auto n3 = SimNode::make("n3");

  // Components
  auto vs = VoltageSource::make("v_s");
  auto rl = Resistor::make("r_line");
  //auto ll = Inductor::make("l_line");
  auto rL = Resistor::make("r_load");

  // Topology
  vs->connect({SimNode::GND, n1});
  rl->connect({n1, n2});
  //ll->connect({ n2, n3 });
  rL->connect({SimNode::GND, n2});

  // Parameters
  vs->setParameters(Complex(10000, 0));
  rl->setParameters(1);
  //ll->setParameters(1);
  rL->setParameters(1000);

  String simName = "DAE_DP_test" + std::to_string(timeStep);

  auto sys = SystemTopology(50, SystemNodeList{SimNode::GND, n1, n2},
                            SystemComponentList{vs, rl, rL});
  Simulation sim(simName);
  sim.setSystem(sys);
  sim.setTimeStep(timeStep);
  sim.setFinalTime(1);
  sim.setDomain(Domain::DP);
  sim.setSolverType(Solver::Type::DAE);

  sim.run();

  return 0;
}
