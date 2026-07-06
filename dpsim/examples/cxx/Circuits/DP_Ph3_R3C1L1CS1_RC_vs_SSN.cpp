// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <DPsim.h>

using namespace DPsim;
using namespace CPS::DP;

// Classical DP reference: current source into a resistive network with a
// shunt inductor and a series capacitor, built from discrete three-phase
// components.
void DP_Ph3_R3_C1_L1_CS1() {
  Real timeStep = 0.0001;
  Real finalTime = 0.1;
  String simNameRC = "DP_Ph3_R3C1L1CS1_RC_vs_SSN_RC";

  auto n1 = SimNode::make("n1", PhaseType::ABC);
  auto n2 = SimNode::make("n2", PhaseType::ABC);

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

  cs0->connect(SimNode::List{n1, SimNode::GND});
  r1->connect(SimNode::List{n2, n1});
  r2->connect(SimNode::List{n2, SimNode::GND});
  r3->connect(SimNode::List{n2, SimNode::GND});
  l1->connect(SimNode::List{n2, SimNode::GND});
  c1->connect(SimNode::List{n1, n2});

  auto sys = SystemTopology(50, SystemNodeList{n1, n2},
                            SystemComponentList{cs0, r1, r2, r3, l1, c1});

  Logger::setLogDir("logs/" + simNameRC);
  auto logger = DataLogger::make(simNameRC);
  logger->logAttribute("V1_RC", n1->attribute("v"));
  logger->logAttribute("V2_RC", n2->attribute("v"));
  logger->logAttribute("V_C1_RC", c1->attribute("v_intf"));
  logger->logAttribute("I_L1_RC", l1->attribute("i_intf"));

  Simulation sim(simNameRC, Logger::Level::info);
  sim.setSystem(sys);
  sim.addLogger(logger);
  sim.setDomain(Domain::DP);
  sim.setTimeStep(timeStep);
  sim.setFinalTime(finalTime);
  sim.run();
}

// DP SSN: the same circuit with L1 as a generic V-type SSN component and C1
// as a generic I-type SSN component, both parametrized directly by (A, B, C,
// D) instead of dedicated Inductor/Capacitor classes.
void DP_Ph3_SSN_R3_C1_L1_CS1() {
  Real timeStep = 0.0001;
  Real finalTime = 0.1;
  String simNameSSN = "DP_Ph3_R3C1L1CS1_RC_vs_SSN_SSN";

  auto n1 = SimNode::make("n1", PhaseType::ABC);
  auto n2 = SimNode::make("n2", PhaseType::ABC);

  Matrix param = Matrix::Zero(3, 3);
  param << 1., 0, 0, 0, 1., 0, 0, 0, 1.;
  Matrix identity3 = Matrix::Identity(3, 3);

  auto cs0 = Ph3::CurrentSource::make("CS0");
  cs0->setParameters(
      CPS::Math::singlePhaseVariableToThreePhase(CPS::Math::polar(1.0, 0.0)));

  auto r1 = Ph3::Resistor::make("R1");
  r1->setParameters(10 * param);

  auto r2 = Ph3::Resistor::make("R2");
  r2->setParameters(param);

  auto r3 = Ph3::Resistor::make("R3");
  r3->setParameters(5 * param);

  // Shunt inductor as V-type SSN: x = i_abc, u = v_abc, y = i_abc.
  Matrix inductance = 0.02 * param;
  auto l1 = Ph3::GenericTwoTerminalVTypeSSN::make("L1_genVSSN");
  l1->setParameters(Matrix::Zero(3, 3), inductance.inverse(), identity3,
                    Matrix::Zero(3, 3));

  // Series capacitor as I-type SSN: x = vC_abc, u = i_abc, y = v_abc.
  Matrix capacitance = 0.001 * param;
  auto c1 = Ph3::GenericTwoTerminalITypeSSN::make("C1_genISSN");
  c1->setParameters(Matrix::Zero(3, 3), capacitance.inverse(), identity3,
                    Matrix::Zero(3, 3));

  cs0->connect(SimNode::List{n1, SimNode::GND});
  r1->connect(SimNode::List{n2, n1});
  r2->connect(SimNode::List{n2, SimNode::GND});
  r3->connect(SimNode::List{n2, SimNode::GND});
  l1->connect(SimNode::List{n2, SimNode::GND});
  c1->connect(SimNode::List{n1, n2});

  auto sys = SystemTopology(50, SystemNodeList{n1, n2},
                            SystemComponentList{cs0, r1, r2, r3, l1, c1});

  Logger::setLogDir("logs/" + simNameSSN);
  auto logger = DataLogger::make(simNameSSN);
  logger->logAttribute("V1_SSN", n1->attribute("v"));
  logger->logAttribute("V2_SSN", n2->attribute("v"));
  logger->logAttribute("V_C1_SSN", c1->attribute("v_intf"));
  logger->logAttribute("I_L1_SSN", l1->attribute("i_intf"));

  Simulation sim(simNameSSN, Logger::Level::info);
  sim.setSystem(sys);
  sim.addLogger(logger);
  sim.setDomain(Domain::DP);
  sim.setTimeStep(timeStep);
  sim.setFinalTime(finalTime);
  sim.run();
}

int main(int argc, char *argv[]) {
  DP_Ph3_R3_C1_L1_CS1();
  DP_Ph3_SSN_R3_C1_L1_CS1();
}
