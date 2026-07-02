// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <DPsim.h>

using namespace DPsim;
using namespace CPS::DP;

int main(int argc, char *argv[]) {
  // Define simulation scenario
  Real timeStep = 0.0001;
  Real finalTime = 0.1;
  String simName = "DP_Ph3_R3C1L1CS1";

  // Nodes
  auto n1 = SimNode::make("n1", PhaseType::ABC);
  auto n2 = SimNode::make("n2", PhaseType::ABC);

  // Components

  Matrix param = Matrix::Zero(3, 3);
  param << 1., 0, 0, 0, 1., 0, 0, 0, 1.;

  auto cs0 = Ph3::CurrentSource::make("CS0");
  cs0->setParameters(
      CPS::Math::singlePhaseVariableToThreePhase(CPS::Math::polar(1.0, 0.0)));

  auto r1 = Ph3::Resistor::make("R1");
  r1->setParameters(10 * param);

  auto r2 = Ph3::Resistor::make("R2");
  r2->setParameters(param);

  auto r3 = Ph3::Resistor::make("R3");
  r3->setParameters(5 * param);

  auto l1 = Ph3::Inductor::make("L1");
  l1->setParameters(0.02 * param);

  auto c1 = Ph3::Capacitor::make("C1");
  c1->setParameters(0.001 * param);

  // Topology
  cs0->connect(SimNode::List{n1, SimNode::GND});

  r1->connect(SimNode::List{n2, n1});
  r2->connect(SimNode::List{n2, SimNode::GND});
  r3->connect(SimNode::List{n2, SimNode::GND});

  l1->connect(SimNode::List{n2, SimNode::GND});

  c1->connect(SimNode::List{n1, n2});

  // Define system topology
  auto sys = SystemTopology(50, SystemNodeList{n1, n2},
                            SystemComponentList{cs0, r1, r2, r3, l1, c1});

  // Logging
  Logger::setLogDir("logs/" + simName);
  auto logger = DataLogger::make(simName);
  logger->logAttribute("V1", n1->attribute("v"));
  logger->logAttribute("V2", n2->attribute("v"));
  logger->logAttribute("V_C1", c1->attribute("v_intf"));
  logger->logAttribute("I_L1", l1->attribute("i_intf"));

  Simulation sim(simName, Logger::Level::info);
  sim.setSystem(sys);
  sim.addLogger(logger);
  sim.setDomain(Domain::DP);
  sim.setTimeStep(timeStep);
  sim.setFinalTime(finalTime);
  sim.run();
}
