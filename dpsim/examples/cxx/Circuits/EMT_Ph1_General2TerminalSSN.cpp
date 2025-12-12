// SPDX-FileCopyrightText: 2025 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <DPsim.h>

using namespace DPsim;
using namespace CPS::EMT;

void EMT_Ph1_C1R1Vs_RC() {
  // Define simulation scenario
  Real timeStep = 0.0001;
  Real finalTime = 0.1;
  String simName = "EMT_Ph1_General2TerminalSSN_C1R1Vs_RC";

  // Nodes
  auto n1 = SimNode::make("n1", PhaseType::Single);
  auto n2 = SimNode::make("n2", PhaseType::Single);

  // Components

  auto c = Ph1::Capacitor::make("c_rc");
  c->setParameters(0.01);

  auto r = Ph1::Resistor::make("r_rc");
  r->setParameters(10.0);

  auto vs = Ph1::VoltageSource::make("vs_rc");
  vs->setParameters((CPS::Math::polar(1.0, CPS::Math::degToRad(-90.0))), 50.0);

  // Topology
  vs->connect(SimNode::List{SimNode::GND, n1});

  c->connect(SimNode::List{n1, n2});
  r->connect(SimNode::List{n2, SimNode::GND});

  // Define system topology
  auto sys =
      SystemTopology(50, SystemNodeList{n1, n2}, SystemComponentList{vs, c, r});

  // Logging
  Logger::setLogDir("logs/" + simName);
  auto logger = DataLogger::make(simName);
  logger->logAttribute("v_c_rc", c->attribute("v_intf"));
  logger->logAttribute("i_c_rc", c->attribute("i_intf"));

  Simulation sim(simName, Logger::Level::info);
  sim.setSystem(sys);
  sim.addLogger(logger);
  sim.setDomain(Domain::EMT);
  sim.setTimeStep(timeStep);
  sim.setFinalTime(finalTime);
  sim.run();
}

void EMT_Ph1_C1R1Vs_generalizedSSN() {
  // Define simulation scenario
  Real timeStep = 0.0001;
  Real finalTime = 0.1;
  String simName = "EMT_Ph1_General2TerminalSSN_C1R1Vs_generalizedSSN";

  // Nodes
  auto n1 = SimNode::make("n1", PhaseType::Single);
  auto n2 = SimNode::make("n2", PhaseType::Single);

  // Components

  auto c = Ph1::SSNTypeI2T::make("c_genSSN");
  //du_c/dt = i_c/c , y = u_c --> u = i_c, x = u_c, A = 0, B = 1/c, C = 1, D = 0
  double c_param = 0.01;
  Matrix A = Matrix::Zero(1, 1);
  Matrix B = Matrix::Zero(1, 1);
  B(0, 0) = (1 / c_param);
  Matrix C = Matrix::Zero(1, 1);
  C(0, 0) = 1;
  Matrix D = Matrix::Zero(1, 1);
  c->setParameters(A, B, C, D);

  auto r = Ph1::Resistor::make("r_genSSN");
  r->setParameters(10.0);

  auto vs = Ph1::VoltageSource::make("vs_genSSN");
  vs->setParameters((CPS::Math::polar(1.0, CPS::Math::degToRad(-90.0))), 50.0);

  // Topology
  vs->connect(SimNode::List{SimNode::GND, n1});

  c->connect(SimNode::List{n1, n2});
  r->connect(SimNode::List{n2, SimNode::GND});

  // Define system topology
  auto sys =
      SystemTopology(50, SystemNodeList{n1, n2}, SystemComponentList{vs, c, r});

  // Logging
  Logger::setLogDir("logs/" + simName);
  auto logger = DataLogger::make(simName);
  logger->logAttribute("v_c_genSSN", c->attribute("v_intf"));
  logger->logAttribute("i_c_genSSN", c->attribute("i_intf"));

  Simulation sim(simName, Logger::Level::info);
  sim.setSystem(sys);
  sim.addLogger(logger);
  sim.setDomain(Domain::EMT);
  sim.setTimeStep(timeStep);
  sim.setFinalTime(finalTime);
  sim.run();
}

void EMT_Ph1_L1R1Vs_RC() {
  // Define simulation scenario
  Real timeStep = 0.0001;
  Real finalTime = 0.1;
  String simName = "EMT_Ph1_General2TerminalSSN_L1R1Vs_RC";

  // Nodes
  auto n1 = SimNode::make("n1", PhaseType::Single);
  auto n2 = SimNode::make("n2", PhaseType::Single);

  // Components

  auto l = Ph1::Inductor::make("l_rc");
  l->setParameters(0.01);

  auto r = Ph1::Resistor::make("r_rc");
  r->setParameters(10.0);

  auto vs = Ph1::VoltageSource::make("vs_rc");
  vs->setParameters((CPS::Math::polar(1.0, CPS::Math::degToRad(-90.0))), 50.0);

  // Topology
  vs->connect(SimNode::List{SimNode::GND, n1});

  l->connect(SimNode::List{n1, n2});
  r->connect(SimNode::List{n2, SimNode::GND});

  // Define system topology
  auto sys =
      SystemTopology(50, SystemNodeList{n1, n2}, SystemComponentList{vs, l, r});

  // Logging
  Logger::setLogDir("logs/" + simName);
  auto logger = DataLogger::make(simName);
  logger->logAttribute("v_l_rc", l->attribute("v_intf"));
  logger->logAttribute("i_l_rc", l->attribute("i_intf"));

  Simulation sim(simName, Logger::Level::info);
  sim.setSystem(sys);
  sim.addLogger(logger);
  sim.setDomain(Domain::EMT);
  sim.setTimeStep(timeStep);
  sim.setFinalTime(finalTime);
  sim.run();
}

void EMT_Ph1_L1R1Vs_generalizedSSN() {
  // Define simulation scenario
  Real timeStep = 0.0001;
  Real finalTime = 0.1;
  String simName = "EMT_Ph1_General2TerminalSSN_L1R1Vs_generalizedSSN";

  // Nodes
  auto n1 = SimNode::make("n1", PhaseType::Single);
  auto n2 = SimNode::make("n2", PhaseType::Single);

  // Components

  auto l = Ph1::SSNTypeV2T::make("l_genSSN");
  //di_L/dt = v_L/L , y = i_L --> u = v_L, x = i_L, A = 0, B = 1/L, C = 1, D = 0
  double l_param = 0.01;
  Matrix A = Matrix::Zero(1, 1);
  Matrix B = Matrix::Zero(1, 1);
  B(0, 0) = (1 / l_param);
  Matrix C = Matrix::Zero(1, 1);
  C(0, 0) = 1;
  Matrix D = Matrix::Zero(1, 1);
  l->setParameters(A, B, C, D);

  auto r = Ph1::Resistor::make("R1");
  r->setParameters(10.0);

  auto vs = Ph1::VoltageSource::make("vs");
  vs->setParameters((CPS::Math::polar(1.0, CPS::Math::degToRad(-90.0))), 50.0);

  // Topology
  vs->connect(SimNode::List{SimNode::GND, n1});

  l->connect(SimNode::List{n1, n2});
  r->connect(SimNode::List{n2, SimNode::GND});

  // Define system topology
  auto sys =
      SystemTopology(50, SystemNodeList{n1, n2}, SystemComponentList{vs, l, r});

  // Logging
  Logger::setLogDir("logs/" + simName);
  auto logger = DataLogger::make(simName);
  logger->logAttribute("v_l_genSSN", l->attribute("v_intf"));
  logger->logAttribute("i_l_genSSN", l->attribute("i_intf"));

  Simulation sim(simName, Logger::Level::info);
  sim.setSystem(sys);
  sim.addLogger(logger);
  sim.setDomain(Domain::EMT);
  sim.setTimeStep(timeStep);
  sim.setFinalTime(finalTime);
  sim.run();
}

void EMT_Ph1_RLCVs_RC() {
  // Define simulation scenario
  Real timeStep = 0.0001;
  Real finalTime = 0.1;
  String simName = "EMT_Ph1_General2TerminalSSN_RLCVs_RC";

  // Nodes
  auto n1 = SimNode::make("n1", PhaseType::Single);
  auto n2 = SimNode::make("n2", PhaseType::Single);
  auto n3 = SimNode::make("n3", PhaseType::Single);

  // Components

  auto r = Ph1::Resistor::make("r_rc");
  r->setParameters(10.0);

  auto l = Ph1::Inductor::make("l_rc");
  l->setParameters(0.01);

  auto c = Ph1::Capacitor::make("c_rc");
  c->setParameters(0.002);

  auto vs = Ph1::VoltageSource::make("vs_rc");
  vs->setParameters((CPS::Math::polar(1.0, CPS::Math::degToRad(-90.0))), 50.0);

  // Topology
  vs->connect(SimNode::List{SimNode::GND, n1});

  r->connect(SimNode::List{n1, n2});
  l->connect(SimNode::List{n2, n3});
  c->connect(SimNode::List{n3, SimNode::GND});

  // Define system topology
  auto sys = SystemTopology(50, SystemNodeList{n1, n2, n3},
                            SystemComponentList{vs, r, l, c});

  // Logging
  Logger::setLogDir("logs/" + simName);
  auto logger = DataLogger::make(simName);
  logger->logAttribute("v_l_rlc_rc", l->attribute("v_intf"));
  logger->logAttribute("i_l_rlc_rc", l->attribute("i_intf"));

  Simulation sim(simName, Logger::Level::info);
  sim.setSystem(sys);
  sim.addLogger(logger);
  sim.setDomain(Domain::EMT);
  sim.setTimeStep(timeStep);
  sim.setFinalTime(finalTime);
  sim.run();
}

void EMT_Ph1_RLCVs_explicitSSN() {
  // Define simulation scenario
  Real timeStep = 0.0001;
  Real finalTime = 0.1;
  String simName = "EMT_Ph1_General2TerminalSSN_RLCVs_explicitSSN";

  // Nodes
  auto n1 = SimNode::make("n1", PhaseType::Single);

  // Components

  auto rlc = Ph1::SSN::Full_Serial_RLC::make("rlc");
  double r_param = 10.0;
  double l_param = 0.01;
  double c_param = 0.002;

  rlc->setParameters(r_param, l_param, c_param);

  auto vs = Ph1::VoltageSource::make("vs");
  vs->setParameters((CPS::Math::polar(1.0, CPS::Math::degToRad(-90.0))), 50.0);

  // Topology
  vs->connect(SimNode::List{SimNode::GND, n1});

  rlc->connect(SimNode::List{n1, SimNode::GND});

  // Define system topology
  auto sys =
      SystemTopology(50, SystemNodeList{n1}, SystemComponentList{vs, rlc});

  // Logging
  Logger::setLogDir("logs/" + simName);
  auto logger = DataLogger::make(simName);
  logger->logAttribute("v_rlc_explSSN", rlc->attribute("v_intf"));
  logger->logAttribute("i_rlc_explSSN", rlc->attribute("i_intf"));

  Simulation sim(simName, Logger::Level::info);
  sim.setSystem(sys);
  sim.addLogger(logger);
  sim.setDomain(Domain::EMT);
  sim.setTimeStep(timeStep);
  sim.setFinalTime(finalTime);
  sim.run();
}

void EMT_Ph1_RLCVs_generalizedSSN() {
  // Define simulation scenario
  Real timeStep = 0.0001;
  Real finalTime = 0.1;
  String simName = "EMT_Ph1_General2TerminalSSN_RLCVs_generalizedSSN";

  // Nodes
  auto n1 = SimNode::make("n1", PhaseType::Single);

  // Components

  auto rlc = Ph1::SSNTypeV2T::make("rlc_genSSN");

  double r = 10.0;
  double l = 0.01;
  double c = 0.002;

  Matrix A = Matrix::Zero(2, 2);
  A(0, 0) = 0;
  A(0, 1) = 1. / c;
  A(1, 0) = -1. / l;
  A(1, 1) = -r / l;
  Matrix B = Matrix::Zero(2, 1);
  B(0, 0) = 0;
  B(1, 0) = 1. / l;
  Matrix C = Matrix::Zero(1, 2);
  C(0, 0) = 0;
  C(0, 1) = 1;
  Matrix D = Matrix::Zero(1, 1);
  rlc->setParameters(A, B, C, D);

  auto vs = Ph1::VoltageSource::make("vs");
  vs->setParameters((CPS::Math::polar(1.0, CPS::Math::degToRad(-90.0))), 50.0);

  // Topology
  vs->connect(SimNode::List{SimNode::GND, n1});

  rlc->connect(SimNode::List{n1, SimNode::GND});

  // Define system topology
  auto sys =
      SystemTopology(50, SystemNodeList{n1}, SystemComponentList{vs, rlc});

  // Logging
  Logger::setLogDir("logs/" + simName);
  auto logger = DataLogger::make(simName);
  logger->logAttribute("v_rlc_genSSN", rlc->attribute("v_intf"));
  logger->logAttribute("i_rlc_genSSN", rlc->attribute("i_intf"));

  Simulation sim(simName, Logger::Level::info);
  sim.setSystem(sys);
  sim.addLogger(logger);
  sim.setDomain(Domain::EMT);
  sim.setTimeStep(timeStep);
  sim.setFinalTime(finalTime);
  sim.run();
}

int main(int argc, char *argv[]) {
  EMT_Ph1_C1R1Vs_RC();
  EMT_Ph1_C1R1Vs_generalizedSSN();

  EMT_Ph1_L1R1Vs_RC();
  EMT_Ph1_L1R1Vs_generalizedSSN();

  EMT_Ph1_RLCVs_RC();
  EMT_Ph1_RLCVs_explicitSSN();
  EMT_Ph1_RLCVs_generalizedSSN();
}
