// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <DPsim.h>

using namespace DPsim;
using namespace CPS::DP;

// V-type reference: series R-L-C from discrete components, voltage-driven.
void DP_Ph1_RLCVs_RC() {
  Real timeStep = 0.0001;
  Real finalTime = 0.1;
  String simName = "DP_Ph1_General2TerminalSSN_RLCVs_RC";

  auto n1 = SimNode::make("n1");
  auto n2 = SimNode::make("n2");
  auto n3 = SimNode::make("n3");

  auto r = Ph1::Resistor::make("r_rc");
  r->setParameters(10.0);
  auto l = Ph1::Inductor::make("l_rc");
  l->setParameters(0.01);
  auto c = Ph1::Capacitor::make("c_rc");
  c->setParameters(0.002);

  auto vs = Ph1::VoltageSource::make("vs_rc");
  vs->setParameters(CPS::Math::polar(1.0, CPS::Math::degToRad(-90.0)));

  vs->connect(SimNode::List{SimNode::GND, n1});
  r->connect(SimNode::List{n1, n2});
  l->connect(SimNode::List{n2, n3});
  c->connect(SimNode::List{n3, SimNode::GND});

  auto sys = SystemTopology(50, SystemNodeList{n1, n2, n3},
                            SystemComponentList{vs, r, l, c});

  Logger::setLogDir("logs/" + simName);
  auto logger = DataLogger::make(simName);
  logger->logAttribute("v_l_rc", l->attribute("v_intf"));
  logger->logAttribute("i_l_rc", l->attribute("i_intf"));

  Simulation sim(simName, Logger::Level::info);
  sim.setSystem(sys);
  sim.addLogger(logger);
  sim.setDomain(Domain::DP);
  sim.setTimeStep(timeStep);
  sim.setFinalTime(finalTime);
  sim.run();
}

// V-type generic SSN: the same series RLC one-port supplied as (A, B, C, D).
void DP_Ph1_RLCVs_genVSSN() {
  Real timeStep = 0.0001;
  Real finalTime = 0.1;
  String simName = "DP_Ph1_General2TerminalSSN_RLCVs_genVSSN";

  auto n1 = SimNode::make("n1");

  auto rlc = Ph1::GenericTwoTerminalVTypeSSN::make("rlc_genSSN");

  double r = 10.0;
  double l = 0.01;
  double c = 0.002;

  // States x = [v_C; i_L], input = port voltage, output = port current (i_L).
  Matrix A = Matrix::Zero(2, 2);
  A(0, 1) = 1. / c;
  A(1, 0) = -1. / l;
  A(1, 1) = -r / l;
  Matrix B = Matrix::Zero(2, 1);
  B(1, 0) = 1. / l;
  Matrix C = Matrix::Zero(1, 2);
  C(0, 1) = 1.;
  Matrix D = Matrix::Zero(1, 1);
  rlc->setParameters(A, B, C, D);

  auto vs = Ph1::VoltageSource::make("vs");
  vs->setParameters(CPS::Math::polar(1.0, CPS::Math::degToRad(-90.0)));

  vs->connect(SimNode::List{SimNode::GND, n1});
  rlc->connect(SimNode::List{n1, SimNode::GND});

  auto sys =
      SystemTopology(50, SystemNodeList{n1}, SystemComponentList{vs, rlc});

  Logger::setLogDir("logs/" + simName);
  auto logger = DataLogger::make(simName);
  logger->logAttribute("v_rlc_genSSN", rlc->attribute("v_intf"));
  logger->logAttribute("i_rlc_genSSN", rlc->attribute("i_intf"));

  Simulation sim(simName, Logger::Level::info);
  sim.setSystem(sys);
  sim.addLogger(logger);
  sim.setDomain(Domain::DP);
  sim.setTimeStep(timeStep);
  sim.setFinalTime(finalTime);
  sim.run();
}

// I-type reference: current source feeding a parallel R-C from discrete parts.
void DP_Ph1_C1R1Is_RC() {
  Real timeStep = 0.0001;
  Real finalTime = 0.1;
  String simName = "DP_Ph1_General2TerminalSSN_C1R1Is_RC";

  auto n1 = SimNode::make("n1");

  auto cs = Ph1::CurrentSource::make("cs_rc");
  cs->setParameters(CPS::Math::polar(1.0, CPS::Math::degToRad(-90.0)));

  auto r = Ph1::Resistor::make("r_rc");
  r->setParameters(10.0);
  auto c = Ph1::Capacitor::make("c_rc");
  c->setParameters(0.002);

  cs->connect(SimNode::List{SimNode::GND, n1});
  r->connect(SimNode::List{n1, SimNode::GND});
  c->connect(SimNode::List{n1, SimNode::GND});

  auto sys =
      SystemTopology(50, SystemNodeList{n1}, SystemComponentList{cs, r, c});

  Logger::setLogDir("logs/" + simName);
  auto logger = DataLogger::make(simName);
  logger->logAttribute("v_c_rc", c->attribute("v_intf"));
  logger->logAttribute("i_c_rc", c->attribute("i_intf"));

  Simulation sim(simName, Logger::Level::info);
  sim.setSystem(sys);
  sim.addLogger(logger);
  sim.setDomain(Domain::DP);
  sim.setTimeStep(timeStep);
  sim.setFinalTime(finalTime);
  sim.run();
}

// I-type generic SSN: the capacitor supplied as (A, B, C, D), current-driven.
void DP_Ph1_C1R1Is_genISSN() {
  Real timeStep = 0.0001;
  Real finalTime = 0.1;
  String simName = "DP_Ph1_General2TerminalSSN_C1R1Is_genISSN";

  auto n1 = SimNode::make("n1");

  auto cap = Ph1::GenericTwoTerminalITypeSSN::make("c_genSSN");

  // dv_C/dt = i_C/C, y = v_C --> u = i_C, x = v_C, A = 0, B = 1/C, C = 1, D = 0.
  double c = 0.002;
  Matrix A = Matrix::Zero(1, 1);
  Matrix B = Matrix::Zero(1, 1);
  B(0, 0) = 1. / c;
  Matrix C = Matrix::Zero(1, 1);
  C(0, 0) = 1.;
  Matrix D = Matrix::Zero(1, 1);
  cap->setParameters(A, B, C, D);

  auto cs = Ph1::CurrentSource::make("cs");
  cs->setParameters(CPS::Math::polar(1.0, CPS::Math::degToRad(-90.0)));

  auto r = Ph1::Resistor::make("r_genSSN");
  r->setParameters(10.0);

  cs->connect(SimNode::List{SimNode::GND, n1});
  r->connect(SimNode::List{n1, SimNode::GND});
  cap->connect(SimNode::List{n1, SimNode::GND});

  auto sys =
      SystemTopology(50, SystemNodeList{n1}, SystemComponentList{cs, r, cap});

  Logger::setLogDir("logs/" + simName);
  auto logger = DataLogger::make(simName);
  logger->logAttribute("v_c_genSSN", cap->attribute("v_intf"));
  logger->logAttribute("i_c_genSSN", cap->attribute("i_intf"));

  Simulation sim(simName, Logger::Level::info);
  sim.setSystem(sys);
  sim.addLogger(logger);
  sim.setDomain(Domain::DP);
  sim.setTimeStep(timeStep);
  sim.setFinalTime(finalTime);
  sim.run();
}

int main(int argc, char *argv[]) {
  DP_Ph1_RLCVs_RC();
  DP_Ph1_RLCVs_genVSSN();

  DP_Ph1_C1R1Is_RC();
  DP_Ph1_C1R1Is_genISSN();
}
