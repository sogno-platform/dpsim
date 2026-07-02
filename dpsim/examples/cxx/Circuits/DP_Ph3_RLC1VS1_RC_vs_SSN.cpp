// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <DPsim.h>

using namespace DPsim;
using namespace CPS::DP;

// Classical DP reference: series R-L-C built from discrete three-phase
// components, voltage-driven.
void DP_Ph3_RLC1_VS1() {
  Real timeStep = 0.0001;
  Real finalTime = 0.1;
  String simNameRC = "DP_Ph3_RLC1VS1_RC_vs_SSN_RC";

  auto n1 = SimNode::make("n1", PhaseType::ABC);
  auto n2 = SimNode::make("n2", PhaseType::ABC);
  auto n3 = SimNode::make("n3", PhaseType::ABC);

  Matrix param = Matrix::Zero(3, 3);
  param << 1., 0, 0, 0, 1., 0, 0, 0, 1.;

  auto vs0 = Ph3::VoltageSource::make("VS0");
  vs0->setParameters(
      CPS::Math::singlePhaseVariableToThreePhase(CPS::Math::polar(1.0, 0.0)));

  auto r1 = Ph3::Resistor::make("R1");
  r1->setParameters(1. * param);

  auto l1 = Ph3::Inductor::make("L1");
  l1->setParameters(0.05 * param);

  auto c1 = Ph3::Capacitor::make("C1");
  c1->setParameters(0.01 * param);

  vs0->connect(SimNode::List{n1, SimNode::GND});
  r1->connect(SimNode::List{n1, n2});
  l1->connect(SimNode::List{n2, n3});
  c1->connect(SimNode::List{n3, SimNode::GND});

  auto sys = SystemTopology(50, SystemNodeList{n1, n2, n3},
                            SystemComponentList{vs0, r1, l1, c1});

  Logger::setLogDir("logs/" + simNameRC);
  auto logger = DataLogger::make(simNameRC);
  logger->logAttribute("I_L1_RC", l1->attribute("i_intf"));
  logger->logAttribute("V1_RC", n1->attribute("v"));

  Simulation sim(simNameRC, Logger::Level::info);
  sim.setSystem(sys);
  sim.addLogger(logger);
  sim.setDomain(Domain::DP);
  sim.setTimeStep(timeStep);
  sim.setFinalTime(finalTime);
  sim.run();
}

// DP V-type SSN: the same series RLC one-port as a single three-phase SSN
// component.
void DP_Ph3_SSN_RLC1_VS1() {
  Real timeStep = 0.0001;
  Real finalTime = 0.1;
  String simNameSSN = "DP_Ph3_RLC1VS1_RC_vs_SSN_SSN";

  auto n1 = SimNode::make("n1", PhaseType::ABC);

  Matrix param = Matrix::Zero(3, 3);
  param << 1., 0, 0, 0, 1., 0, 0, 0, 1.;

  auto vs0 = Ph3::VoltageSource::make("VS0");
  vs0->setParameters(
      CPS::Math::singlePhaseVariableToThreePhase(CPS::Math::polar(1.0, 0.0)));

  auto rlc = Ph3::SSN::Full_Serial_RLC::make("RLC");
  rlc->setParameters(1. * param, 0.05 * param, 0.01 * param);

  vs0->connect(SimNode::List{n1, SimNode::GND});
  rlc->connect(SimNode::List{n1, SimNode::GND});

  auto sys =
      SystemTopology(50, SystemNodeList{n1}, SystemComponentList{vs0, rlc});

  Logger::setLogDir("logs/" + simNameSSN);
  auto logger = DataLogger::make(simNameSSN);
  logger->logAttribute("I_RLC_SSN", rlc->attribute("i_intf"));
  logger->logAttribute("V1_SSN", n1->attribute("v"));

  Simulation sim(simNameSSN, Logger::Level::info);
  sim.setSystem(sys);
  sim.addLogger(logger);
  sim.setDomain(Domain::DP);
  sim.setSolverType(Solver::Type::MNA);
  sim.setTimeStep(timeStep);
  sim.setFinalTime(finalTime);
  sim.run();
}

int main(int argc, char *argv[]) {
  DP_Ph3_RLC1_VS1();
  DP_Ph3_SSN_RLC1_VS1();
}
