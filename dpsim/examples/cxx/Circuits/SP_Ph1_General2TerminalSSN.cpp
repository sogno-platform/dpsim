// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <DPsim.h>

using namespace DPsim;
using namespace CPS::SP;

void SP_Ph1_VS_RLC(double R, double L, double C) {
  // Define simulation scenario
  Real timeStep = 0.0001;
  Real finalTime = 0.1;
  String simName = "SP_Ph1_VS_rlc_RC";
  Logger::setLogDir("logs/" + simName);

  // Nodes
  auto n1 = SimNode::make("n1", PhaseType::Single);
  auto n2 = SimNode::make("n2", PhaseType::Single);
  auto n3 = SimNode::make("n3", PhaseType::Single);

  // Components
  auto vs = Ph1::VoltageSource::make("vs");
  vs->setParameters(Complex(1.0, 2.0));
  auto r1 = Ph1::Resistor::make("r_1");
  r1->setParameters(R);
  auto l1 = Ph1::Inductor::make("l_1");
  l1->setParameters(L);
  auto c1 = Ph1::Capacitor::make("c_1");
  c1->setParameters(C);

  // Topology
  vs->connect(SimNode::List{SimNode::GND, n1});
  r1->connect(SimNode::List{n1, n2});
  l1->connect(SimNode::List{n2, n3});
  c1->connect(SimNode::List{n3, SimNode::GND});

  auto sys = SystemTopology(50, SystemNodeList{n1, n2, n3},
                            SystemComponentList{vs, r1, l1, c1});

  // Logging
  auto logger = DataLogger::make(simName);
  logger->logAttribute("v1", n1->attribute("v"));
  logger->logAttribute("v2", n2->attribute("v"));
  logger->logAttribute("v3", n3->attribute("v"));
  logger->logAttribute("i12", r1->attribute("i_intf"));

  Simulation sim(simName, Logger::Level::info);
  sim.setSystem(sys);
  sim.addLogger(logger);
  sim.doInitFromNodesAndTerminals(true);
  sim.setDomain(Domain::SP);
  sim.setTimeStep(timeStep);
  sim.setFinalTime(finalTime);
  sim.run();
}

void SP_Ph1_SSNsingleComps_VS_RLC(double R, double L, double C) {
  // Define simulation scenario
  Real timeStep = 0.0001;
  Real finalTime = 0.1;
  String simName = "SP_Ph1_VS_rlc_SSNsingleComps";
  Logger::setLogDir("logs/" + simName);

  // Nodes
  auto n1 = SimNode::make("n1", PhaseType::Single);
  auto n2 = SimNode::make("n2", PhaseType::Single);
  auto n3 = SimNode::make("n3", PhaseType::Single);

  // Components
  auto vs = Ph1::VoltageSource::make("vs");
  vs->setParameters(Complex(1.0, 2.0));

  Matrix A_r1 = Matrix::Zero(1, 1);
  Matrix B_r1 = Matrix::Zero(1, 1);
  Matrix C_r1 = Matrix::Zero(1, 1);
  Matrix D_r1 = Matrix::Zero(1, 1);
  D_r1 << 1. / R;
  auto r1 = Ph1::SSNTypeV2T::make("r_1");
  r1->setParameters(A_r1, B_r1, C_r1, D_r1);

  Matrix A_l1 = Matrix::Zero(1, 1);
  Matrix B_l1 = Matrix::Zero(1, 1);
  B_l1 << 1. / L;
  Matrix C_l1 = Matrix::Zero(1, 1);
  C_l1 << 1.;
  Matrix D_l1 = Matrix::Zero(1, 1);
  auto l1 = Ph1::SSNTypeV2T::make("l_1");
  l1->setParameters(A_l1, B_l1, C_l1, D_l1); //Idot = U/L , B = 1/L, C = 1

  Matrix A_c1 = Matrix::Zero(1, 1);
  Matrix B_c1 = Matrix::Zero(1, 1);
  B_c1 << 1. / C;
  Matrix C_c1 = Matrix::Zero(1, 1);
  C_c1 << 1.;
  Matrix D_c1 = Matrix::Zero(1, 1);
  auto c1 = Ph1::SSNTypeI2T::make("c_1");
  c1->setParameters(A_c1, B_c1, C_c1, D_c1); //Udot = I/C, B = 1/C, C = 1

  // Topology
  vs->connect(SimNode::List{SimNode::GND, n1});
  r1->connect(SimNode::List{n1, n2});
  l1->connect(SimNode::List{n2, n3});
  c1->connect(SimNode::List{n3, SimNode::GND});

  auto sys = SystemTopology(50, SystemNodeList{n1, n2, n3},
                            SystemComponentList{vs, r1, l1, c1});

  // Logging
  auto logger = DataLogger::make(simName);
  logger->logAttribute("v1", n1->attribute("v"));
  logger->logAttribute("v2", n2->attribute("v"));
  logger->logAttribute("v3", n3->attribute("v"));
  logger->logAttribute("i12", r1->attribute("i_intf"));

  Simulation sim(simName, Logger::Level::info);
  sim.setSystem(sys);
  sim.addLogger(logger);
  sim.doInitFromNodesAndTerminals(true);
  sim.setDomain(Domain::SP);
  sim.setTimeStep(timeStep);
  sim.setFinalTime(finalTime);
  sim.run();
}

void SP_Ph1_SSNcombined_VS_RLC(double R, double L, double C) {
  // Define simulation scenario
  Real timeStep = 0.0001;
  Real finalTime = 0.1;
  String simName = "SP_Ph1_VS_rlc_genSSNcombined";
  Logger::setLogDir("logs/" + simName);

  // Nodes
  auto n1 = SimNode::make("n1", PhaseType::Single);

  // Components
  auto vs = Ph1::VoltageSource::make("vs");
  vs->setParameters(Complex(1.0, 2.0));

  Matrix A_rlc = Matrix::Zero(2, 2);
  A_rlc << 0, 1. / C, -1. / L, -R / L;
  Matrix B_rlc = Matrix::Zero(2, 1);
  B_rlc << 0, 1. / L;
  Matrix C_rlc = Matrix::Zero(1, 2);
  C_rlc << 0, 1.;
  Matrix D_rlc = Matrix::Zero(1, 1);
  auto rlc = Ph1::SSNTypeV2T::make("rlc");
  rlc->setParameters(A_rlc, B_rlc, C_rlc, D_rlc);

  // Topology
  vs->connect(SimNode::List{SimNode::GND, n1});
  rlc->connect(SimNode::List{n1, SimNode::GND});

  auto sys =
      SystemTopology(50, SystemNodeList{n1}, SystemComponentList{vs, rlc});

  // Logging
  auto logger = DataLogger::make(simName);
  logger->logAttribute("v1", n1->attribute("v"));
  logger->logAttribute("i12", rlc->attribute("i_intf"));

  Simulation sim(simName, Logger::Level::info);
  sim.setSystem(sys);
  sim.addLogger(logger);
  sim.doInitFromNodesAndTerminals(true);
  sim.setDomain(Domain::SP);
  sim.setTimeStep(timeStep);
  sim.setFinalTime(finalTime);
  sim.run();
}

int main(int argc, char *argv[]) {
  const double R = 1.5;
  const double L = 0.02;
  const double C = 0.08;

  SP_Ph1_VS_RLC(R, L, C);
  SP_Ph1_SSNsingleComps_VS_RLC(R, L, C);
  SP_Ph1_SSNcombined_VS_RLC(R, L, C);
}
