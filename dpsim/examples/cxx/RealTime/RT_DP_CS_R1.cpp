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

void RT_DP_CS_R1();
void Ref_DP_CS_R1();

int main(int argc, char *argv[]) {
  RT_DP_CS_R1();
  Ref_DP_CS_R1();
}

void RT_DP_CS_R1() {
  Real timeStep = 0.001;
  Real finalTime = 5;
  String simName = "RT_DP_CS_R1";
  Logger::setLogDir("logs/" + simName);

  // Nodes
  auto n1 = SimNode::make("n1");

  // Components
  auto cs = CurrentSource::make("cs");
  cs->setParameters(Complex(10, 0));
  auto r1 = Resistor::make("r_1");
  r1->setParameters(1);

  // Connections
  cs->connect({SimNode::GND, n1});
  r1->connect({SimNode::GND, n1});

  // Define system topology
  auto sys = SystemTopology(50, SystemNodeList{SimNode::GND, n1},
                            SystemComponentList{cs, r1});

  RealTimeSimulation sim(simName, Logger::Level::off);
  sim.setSystem(sys);
  sim.setTimeStep(timeStep);
  sim.setFinalTime(finalTime);

  sim.run();
  sim.logStepTimes(simName + "_step_times");
}

void Ref_DP_CS_R1() {
  Real timeStep = 0.001;
  Real finalTime = 5;
  String simName = "Ref_DP_CS_R1";
  Logger::setLogDir("logs/" + simName);

  // Nodes
  auto n1 = SimNode::make("n1");

  // Components
  auto cs = CurrentSource::make("cs");
  cs->setParameters(Complex(10, 0));
  auto r1 = Resistor::make("r_1");
  r1->setParameters(1);

  // Connections
  cs->connect({SimNode::GND, n1});
  r1->connect({SimNode::GND, n1});

  // Define system topology
  auto sys = SystemTopology(50, SystemNodeList{SimNode::GND, n1},
                            SystemComponentList{cs, r1});

  Simulation sim(simName, Logger::Level::off);
  sim.setSystem(sys);
  sim.setTimeStep(timeStep);
  sim.setFinalTime(finalTime);

  sim.run();
  sim.logStepTimes(simName + "_step_times");
}
