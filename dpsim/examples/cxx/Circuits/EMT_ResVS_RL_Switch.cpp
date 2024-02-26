/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <DPsim.h>

using namespace DPsim;
using namespace CPS::EMT::Ph1;

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
  rl->setParameters(1);
  ll->setParameters(1);

  // Define system topology
  SystemTopology system0(50, {GND, n1, n2, n3}, {vs, rl, ll});

  SystemTopology system1 = system0;
  SystemTopology system2 = system0;
  system1.mComponents.push_back(
      Resistor::make("r_load", 2, DEPRECATEDGND, 1000));
  system2.mComponents.push_back(
      Resistor::make("r_load", 2, DEPRECATEDGND, 800));

  // Define simulation scenario
  Real timeStep = 0.001;
  Real finalTime = 0.3;
  String simName = "EMT_ResVS_RxLine_Switch1_" + std::to_string(timeStep);

  Simulation sim(simName, Logger::Level::info);
  sim.setSystem(system1);
  sim.setTimeStep(timeStep);
  sim.setFinalTime(finalTime);
  sim.setDomain(Domain::EMT);
  sim.setSolverType(Solver::Type::MNA);
  sim.addSystemTopology(system2);
  sim.setSwitchTime(0.1, 1);

  sim.run();

  return 0;
}
