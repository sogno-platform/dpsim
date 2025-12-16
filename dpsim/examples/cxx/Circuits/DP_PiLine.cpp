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

void simElements() {
  Real timeStep = 0.00005;
  Real finalTime = 1;
  String simName = "DP_PiLine_Elements";
  Logger::setLogDir("logs/" + simName);

  // Nodes
  auto n1 = SimNode::make("n1");
  auto n2 = SimNode::make("n2");
  auto vn1 = SimNode::make("vn1");

  // Components
  auto vs = Ph1::VoltageSource::make("v_1");
  vs->setParameters(CPS::Math::polar(100000, 0));

  // Parametrization of components
  Real resistance = 5;
  Real inductance = 0.16;
  Real capacitance = 1.0e-6;
  Real conductance = 1e-6;

  auto res = Ph1::Resistor::make("R_line");
  res->setParameters(resistance);
  auto ind = Ph1::Inductor::make("L_line");
  ind->setParameters(inductance);
  auto cap1 = Ph1::Capacitor::make("Cp_1");
  cap1->setParameters(capacitance / 2.);
  auto cap2 = Ph1::Capacitor::make("Cp_2");
  cap2->setParameters(capacitance / 2.);
  auto con1 = Ph1::Resistor::make("Gp_1");
  con1->setParameters(2. / conductance);
  auto con2 = Ph1::Resistor::make("Gp_2");
  con2->setParameters(2. / conductance);

  auto load = Ph1::Resistor::make("R_load");
  load->setParameters(10000);

  // Topology
  vs->connect({SimNode::GND, n1});
  res->connect({n1, vn1});
  ind->connect({vn1, n2});
  cap1->connect({n1, SimNode::GND});
  cap2->connect({n2, SimNode::GND});
  con1->connect({n1, SimNode::GND});
  con2->connect({n2, SimNode::GND});
  load->connect({n2, SimNode::GND});

  auto sys = SystemTopology(
      50, SystemNodeList{n1, n2, vn1},
      SystemComponentList{vs, res, ind, cap1, cap2, con1, con2, load});

  // Logging
  auto logger = DataLogger::make(simName);
  logger->logAttribute("v1", n1->attribute("v"));
  logger->logAttribute("v2", n2->attribute("v"));
  logger->logAttribute("iline", ind->attribute("i_intf"));

  Simulation sim(simName);
  sim.setSystem(sys);
  sim.setTimeStep(timeStep);
  sim.setFinalTime(finalTime);
  sim.addLogger(logger);

  sim.run();
}

void simPiLine() {
  Real timeStep = 0.00005;
  Real finalTime = 1;
  String simName = "DP_PiLine_Component";
  Logger::setLogDir("logs/" + simName);

  // Nodes
  auto n1 = SimNode::make("n1");
  auto n2 = SimNode::make("n2");

  // Components
  auto vs = Ph1::VoltageSource::make("v_1");
  vs->setParameters(CPS::Math::polar(100000, 0));

  // Parametrization of components
  Real resistance = 5;
  Real inductance = 0.16;
  Real capacitance = 1.0e-6;
  Real conductance = 1e-6;

  auto line = Ph1::PiLine::make("Line");
  line->setParameters(resistance, inductance, capacitance, conductance);

  auto load = Ph1::Resistor::make("R_load");
  load->setParameters(10000);

  // Topology
  vs->connect({SimNode::GND, n1});
  line->connect({n1, n2});
  load->connect({n2, SimNode::GND});

  auto sys = SystemTopology(50, SystemNodeList{n1, n2},
                            SystemComponentList{vs, line, load});

  // Logging
  auto logger = DataLogger::make(simName);
  logger->logAttribute("v1", n1->attribute("v"));
  logger->logAttribute("v2", n2->attribute("v"));
  logger->logAttribute("iline", line->attribute("i_intf"));

  Simulation sim(simName);
  sim.setSystem(sys);
  sim.setTimeStep(timeStep);
  sim.setFinalTime(finalTime);
  sim.addLogger(logger);

  sim.run();
}

void simPiLineDiakoptics() {
  Real timeStep = 0.00005;
  Real finalTime = 0.1;
  String simName = "DP_PiLine_Diakoptics";
  Logger::setLogDir("logs/" + simName);

  // Nodes
  auto n1 = SimNode::make("n1");
  auto n2 = SimNode::make("n2");

  // Components
  auto vs = Ph1::VoltageSource::make("v_1");
  vs->setParameters(CPS::Math::polar(100000, 0));

  // R=5, X=50 (L=0.16), B=0.003 (C=1e-6)
  auto line = Ph1::PiLine::make("Line");
  line->setParameters(5, 0.16, 1e-6);

  auto load = Ph1::Resistor::make("R_load");
  load->setParameters(10000);

  // Topology
  vs->connect({SimNode::GND, n1});
  line->connect({n1, n2});
  load->connect({n2, SimNode::GND});

  auto sys =
      SystemTopology(50, SystemNodeList{n1, n2}, SystemComponentList{vs, load});
  sys.addTearComponent(line);

  // Logging
  auto logger = DataLogger::make(simName);
  logger->logAttribute("v1", n1->attribute("v"));
  logger->logAttribute("v2", n2->attribute("v"));
  logger->logAttribute("iline", line->attribute("i_intf"));

  Simulation sim(simName);
  sim.setSystem(sys);
  sim.setTearingComponents(sys.mTearComponents);
  sim.setTimeStep(timeStep);
  sim.setFinalTime(finalTime);
  sim.addLogger(logger);

  sim.run();
}

void simElements3ph() {
  Real timeStep = 0.00005;
  Real finalTime = 1;
  String simName = "DP_PiLine_Elements_3ph";
  Logger::setLogDir("logs/" + simName);

  // Nodes
  auto n1 = SimNode::make("n1", PhaseType::ABC);
  auto n2 = SimNode::make("n2", PhaseType::ABC);
  auto vn1 = SimNode::make("vn1", PhaseType::ABC);

  // Components
  auto vs = Ph3::VoltageSource::make("v_1");
  vs->setParameters(CPS::Math::polar(100000, 0));

  // Parametrization of components
  Real resistance = 5;
  Real inductance = 0.16;
  Real capacitance = 1.0e-6;
  Real conductance = 1e-6;

  auto res = Ph3::Resistor::make("R_line");
  res->setParameters(CPS::Math::singlePhaseParameterToThreePhase(resistance));
  auto ind = Ph3::Inductor::make("L_line");
  ind->setParameters(CPS::Math::singlePhaseParameterToThreePhase(inductance));
  auto cap1 = Ph3::Capacitor::make("Cp_1");
  cap1->setParameters(
      CPS::Math::singlePhaseParameterToThreePhase(capacitance / 2.));
  auto cap2 = Ph3::Capacitor::make("Cp_2");
  cap2->setParameters(
      CPS::Math::singlePhaseParameterToThreePhase(capacitance / 2.));
  auto con1 = Ph3::Resistor::make("Gp_1");
  con1->setParameters(
      CPS::Math::singlePhaseParameterToThreePhase(2. / conductance));
  auto con2 = Ph3::Resistor::make("Gp_2");
  con2->setParameters(
      CPS::Math::singlePhaseParameterToThreePhase(2. / conductance));

  auto load = Ph3::Resistor::make("R_load");
  load->setParameters(CPS::Math::singlePhaseParameterToThreePhase(10000));

  // Topology
  vs->connect({SimNode::GND, n1});
  res->connect({n1, vn1});
  ind->connect({vn1, n2});
  cap1->connect({n1, SimNode::GND});
  cap2->connect({n2, SimNode::GND});
  con1->connect({n1, SimNode::GND});
  con2->connect({n2, SimNode::GND});
  load->connect({n2, SimNode::GND});

  auto sys = SystemTopology(
      50, SystemNodeList{n1, n2, vn1},
      SystemComponentList{vs, res, ind, cap1, cap2, con1, con2, load});

  // Logging
  auto logger = DataLogger::make(simName);
  logger->logAttribute("v1", n1->attribute("v"));
  logger->logAttribute("v2", n2->attribute("v"));
  logger->logAttribute("iline", ind->attribute("i_intf"));

  Simulation sim(simName);
  sim.setSystem(sys);
  sim.setTimeStep(timeStep);
  sim.setFinalTime(finalTime);
  sim.addLogger(logger);

  sim.run();
}

void simPiLine3ph() {
  Real timeStep = 0.00005;
  Real finalTime = 1;
  String simName = "DP_PiLine_Component_3ph";
  Logger::setLogDir("logs/" + simName);

  // Nodes
  auto n1 = SimNode::make("n1", PhaseType::ABC);
  auto n2 = SimNode::make("n2", PhaseType::ABC);

  // Components
  auto vs = Ph3::VoltageSource::make("v_1");
  vs->setParameters(CPS::Math::polar(100000, 0));

  // Parametrization of components
  Real resistance = 5;
  Real inductance = 0.16;
  Real capacitance = 1.0e-6;
  Real conductance = 1e-6;

  auto line = Ph3::PiLine::make("Line");
  line->setParameters(CPS::Math::singlePhaseParameterToThreePhase(resistance),
                      CPS::Math::singlePhaseParameterToThreePhase(inductance),
                      CPS::Math::singlePhaseParameterToThreePhase(capacitance),
                      CPS::Math::singlePhaseParameterToThreePhase(conductance));

  auto load = Ph3::Resistor::make("R_load");
  load->setParameters(CPS::Math::singlePhaseParameterToThreePhase(10000));

  // Topology
  vs->connect({SimNode::GND, n1});
  line->connect({n1, n2});
  load->connect({n2, SimNode::GND});

  auto sys = SystemTopology(50, SystemNodeList{n1, n2},
                            SystemComponentList{vs, line, load});

  // Logging
  auto logger = DataLogger::make(simName);
  logger->logAttribute("v1", n1->attribute("v"));
  logger->logAttribute("v2", n2->attribute("v"));
  logger->logAttribute("iline", line->attribute("i_intf"));

  Simulation sim(simName);
  sim.setSystem(sys);
  sim.setTimeStep(timeStep);
  sim.setFinalTime(finalTime);
  sim.addLogger(logger);

  sim.run();
}

void simPiLineDiakoptics3ph() {
  Real timeStep = 0.00005;
  Real finalTime = 0.1;
  String simName = "DP_PiLine_Diakoptics_3ph";
  Logger::setLogDir("logs/" + simName);

  // Nodes
  auto n1 = SimNode::make("n1", PhaseType::ABC);
  auto n2 = SimNode::make("n2", PhaseType::ABC);

  // Components
  auto vs = Ph3::VoltageSource::make("v_1");
  vs->setParameters(CPS::Math::polar(100000, 0));

  // Parametrization of components
  Real resistance = 5;
  Real inductance = 0.16;
  Real capacitance = 1.0e-6;
  Real conductance = 1e-6;

  auto line = Ph3::PiLine::make("Line");
  line->setParameters(CPS::Math::singlePhaseParameterToThreePhase(resistance),
                      CPS::Math::singlePhaseParameterToThreePhase(inductance),
                      CPS::Math::singlePhaseParameterToThreePhase(capacitance),
                      CPS::Math::singlePhaseParameterToThreePhase(conductance));

  auto load = Ph3::Resistor::make("R_load");
  load->setParameters(CPS::Math::singlePhaseParameterToThreePhase(10000));

  // Topology
  vs->connect({SimNode::GND, n1});
  line->connect({n1, n2});
  load->connect({n2, SimNode::GND});

  auto sys =
      SystemTopology(50, SystemNodeList{n1, n2}, SystemComponentList{vs, load});
  sys.addTearComponent(line);

  // Logging
  auto logger = DataLogger::make(simName);
  logger->logAttribute("v1", n1->attribute("v"));
  logger->logAttribute("v2", n2->attribute("v"));
  logger->logAttribute("iline", line->attribute("i_intf"));

  Simulation sim(simName);
  sim.setSystem(sys);
  sim.setTearingComponents(sys.mTearComponents);
  sim.setTimeStep(timeStep);
  sim.setFinalTime(finalTime);
  sim.addLogger(logger);

  sim.run();
}

int main(int argc, char *argv[]) {
  simElements();
  simPiLine();
  simPiLineDiakoptics();

  simElements3ph();
  simPiLine3ph();
  simPiLineDiakoptics3ph();
}
