/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <DPsim.h>

using namespace DPsim;
using namespace CPS::SP;

void SP_Ph3_VS_R2L3() {
  // Define simulation scenario
  Real timeStep = 0.0001;
  Real finalTime = 0.1;
  String simName = "SP_Ph3_VS_R2L3";
  Logger::setLogDir("logs/" + simName);

  // Nodes
  auto n1 = SimNode::make("n1", PhaseType::ABC);
  auto n2 = SimNode::make("n2", PhaseType::ABC);
  auto n3 = SimNode::make("n3", PhaseType::ABC);
  auto n4 = SimNode::make("n4", PhaseType::ABC);

  // Components
  auto vs = Ph3::VoltageSource::make("vs");
  vs->setParameters(10);
  auto r1 = Ph3::Resistor::make("r_1");
  Matrix r1_param = Matrix::Zero(3, 3);
  r1_param << 1., 0, 0, 0, 1., 0, 0, 0, 1.;
  r1->setParameters(r1_param);
  auto l1 = Ph3::Inductor::make("l_1");
  Matrix l_param = Matrix::Zero(3, 3);
  l_param << 1., 0, 0, 0, 1., 0, 0, 0, 1.;
  l1->setParameters(0.02 * l_param);
  auto l2 = Ph3::Inductor::make("l_2");
  l2->setParameters(0.1 * l_param);
  auto l3 = Ph3::Inductor::make("l_3");
  l3->setParameters(0.05 * l_param);
  auto r2 = Ph3::Resistor::make("r_2");
  r2->setParameters(2 * r1_param);

  // Topology
  vs->connect(SimNode::List{SimNode::GND, n1});
  r1->connect(SimNode::List{n1, n2});
  l1->connect(SimNode::List{n2, n3});
  l2->connect(SimNode::List{n3, SimNode::GND});
  l3->connect(SimNode::List{n3, n4});
  r2->connect(SimNode::List{n4, SimNode::GND});

  auto sys = SystemTopology(50, SystemNodeList{n1, n2, n3, n4},
                            SystemComponentList{vs, r1, l1, l2, l3, r2});

  // Logging
  auto logger = DataLogger::make(simName);
  logger->logAttribute("v1", n1->attribute("v"));
  logger->logAttribute("v2", n2->attribute("v"));
  logger->logAttribute("v3", n3->attribute("v"));
  logger->logAttribute("v4", n4->attribute("v"));
  logger->logAttribute("i12", r1->attribute("i_intf"));
  logger->logAttribute("i34", l3->attribute("i_intf"));

  Simulation sim(simName, Logger::Level::info);
  sim.setSystem(sys);
  sim.addLogger(logger);
  sim.setDomain(Domain::SP);
  sim.setTimeStep(timeStep);
  sim.setFinalTime(finalTime);
  sim.run();
}

void SP_Ph3_VS_RC1() {
  // Define simulation scenario
  Real timeStep = 0.0001;
  Real finalTime = 0.1;
  String simName = "SP_Ph3_VS_RC1";
  Logger::setLogDir("logs/" + simName);

  // Nodes
  auto n1 = SimNode::make("n1", PhaseType::ABC);
  auto n2 = SimNode::make("n2", PhaseType::ABC);

  // Components
  auto vs = Ph3::VoltageSource::make("vs", Logger::Level::info);
  vs->setParameters(Complex(10, 0));
  auto r1 = Ph3::Resistor::make("r_1", Logger::Level::info);
  Matrix r1_param = Matrix::Zero(3, 3);
  r1_param << 1., 0, 0, 0, 1., 0, 0, 0, 1.;
  r1->setParameters(r1_param);
  auto c1 = Ph3::Capacitor::make("c_1", Logger::Level::info);
  Matrix c_param = Matrix::Zero(3, 3);
  c_param << 0.001, 0, 0, 0, 0.001, 0, 0, 0, 0.001;
  c1->setParameters(c_param);

  // Topology
  vs->connect({SimNode::GND, n1});
  r1->connect({n1, n2});
  c1->connect({n2, SimNode::GND});

  auto sys = SystemTopology(50, SystemNodeList{n1, n2},
                            SystemComponentList{vs, r1, c1});

  // Logging
  auto logger = DataLogger::make(simName);
  logger->logAttribute("v1", n1->attribute("v"));
  logger->logAttribute("v2", n2->attribute("v"));
  logger->logAttribute("i12", r1->attribute("i_intf"));

  Simulation sim(simName, Logger::Level::info);
  sim.setSystem(sys);
  sim.addLogger(logger);
  sim.setDomain(Domain::SP);
  sim.setTimeStep(timeStep);
  sim.setFinalTime(finalTime);
  sim.run();
}

int main(int argc, char *argv[]) { SP_Ph3_VS_R2L3(); }
