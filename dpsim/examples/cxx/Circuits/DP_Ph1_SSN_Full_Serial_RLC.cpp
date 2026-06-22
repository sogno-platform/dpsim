// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <DPsim.h>

using namespace DPsim;
using namespace CPS::DP;

// Classical DP reference: series R-L-C built from discrete components.
void DP_Ph1_RLCVs_RC() {
  Real timeStep = 0.0001;
  Real finalTime = 0.1;
  String simName = "DP_Ph1_SSN_Full_Serial_RLC_RC";

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

// DP V-type SSN: the same series RLC one-port as a single SSN component.
void DP_Ph1_RLCVs_SSN() {
  Real timeStep = 0.0001;
  Real finalTime = 0.1;
  String simName = "DP_Ph1_SSN_Full_Serial_RLC_SSN";

  auto n1 = SimNode::make("n1");

  auto rlc = Ph1::SSN::Full_Serial_RLC::make("rlc");
  rlc->setParameters(10.0, 0.01, 0.002);

  auto vs = Ph1::VoltageSource::make("vs");
  vs->setParameters(CPS::Math::polar(1.0, CPS::Math::degToRad(-90.0)));

  vs->connect(SimNode::List{SimNode::GND, n1});
  rlc->connect(SimNode::List{n1, SimNode::GND});

  auto sys =
      SystemTopology(50, SystemNodeList{n1}, SystemComponentList{vs, rlc});

  Logger::setLogDir("logs/" + simName);
  auto logger = DataLogger::make(simName);
  logger->logAttribute("v_rlc_ssn", rlc->attribute("v_intf"));
  logger->logAttribute("i_rlc_ssn", rlc->attribute("i_intf"));

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
  DP_Ph1_RLCVs_SSN();
}
