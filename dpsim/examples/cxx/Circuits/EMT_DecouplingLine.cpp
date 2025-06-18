/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include "dpsim-models/Definitions.h"
#include "dpsim/Definitions.h"
#include <DPsim.h>

using namespace DPsim;
using namespace CPS::EMT;
using namespace CPS::EMT::Ph1;
using namespace CPS::Signal;

void simElements() {
  Real timeStep = 0.00005;
  Real finalTime = 1;
  String simName = "EMT_DecouplingLine_Elements";
  Logger::setLogDir("logs/" + simName);

  // Nodes
  auto n1 = SimNode::make("n1");
  auto n2 = SimNode::make("n2");
  auto vn1 = SimNode::make("vn1");

  // Components
  auto vs = VoltageSource::make("v_1", Logger::Level::debug);
  vs->setParameters(CPS::Math::polar(100000, 0), 50);

  // Parametrization of components
  Real resistance = 5;
  Real inductance = 0.16;
  Real capacitance = 1.0e-6;
  Real conductance = 1e-6;

  auto res = Resistor::make("R_line", Logger::Level::debug);
  res->setParameters(resistance);
  auto ind = Inductor::make("L_line", Logger::Level::debug);
  ind->setParameters(inductance);
  auto cap1 = Capacitor::make("Cp_1", Logger::Level::debug);
  cap1->setParameters(capacitance / 2.);
  auto cap2 = Capacitor::make("Cp_2", Logger::Level::debug);
  cap2->setParameters(capacitance / 2.);
  auto con1 = Resistor::make("Gp_1", Logger::Level::debug);
  con1->setParameters(2. / conductance);
  auto con2 = Resistor::make("Gp_2", Logger::Level::debug);
  con2->setParameters(2. / conductance);

  auto load = Resistor::make("R_load", Logger::Level::debug);
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
  //SystemComponentList{vs, res, ind, cap1, cap2, load});
  //SystemComponentList{vs, res, ind, load});

  // Logging
  auto logger = DataLogger::make(simName);
  logger->logAttribute("v1", n1->attribute("v"));
  logger->logAttribute("v2", n2->attribute("v"));
  logger->logAttribute("iline", ind->attribute("i_intf"));

  Simulation sim(simName, Logger::Level::debug);
  sim.setSystem(sys);
  sim.setDomain(Domain::EMT);
  sim.setTimeStep(timeStep);
  sim.setFinalTime(finalTime);
  sim.addLogger(logger);

  sim.run();
}

void simDecouplingLine() {
  Real timeStep = 0.00005;
  Real finalTime = 0.1;
  String simName = "EMT_DecouplingLine_Component";
  Logger::setLogDir("logs/" + simName);

  // Nodes
  auto n1 = SimNode::make("n1");
  auto n2 = SimNode::make("n2");

  // Components
  auto vs = VoltageSource::make("v_1", Logger::Level::debug);
  vs->setParameters(Complex(100000, 0), 50);

  // Parametrization of components
  Real resistance = 5;
  Real inductance = 0.16;
  Real capacitance = 1.0e-6;

  auto dline = DecouplingLineEMT::make("dline", Logger::Level::debug);
  dline->setParameters(n1, n2, resistance, inductance, capacitance);

  auto load = Resistor::make("R_load", Logger::Level::debug);
  load->setParameters(10000);

  // Topology
  vs->connect({SimNode::GND, n1});
  load->connect({n2, SimNode::GND});

  auto sys = SystemTopology(50, SystemNodeList{n1, n2},
                            SystemComponentList{vs, dline, load});
  sys.addComponents(dline->getLineComponents());

  // Logging
  auto logger = DataLogger::make(simName);
  logger->logAttribute("v1", n1->attribute("v"));
  logger->logAttribute("v2", n2->attribute("v"));
  logger->logAttribute("i1", vs->attribute("i_intf"));
  logger->logAttribute("i2", load->attribute("i_intf"));
  logger->logAttribute("i_src1", dline->attribute("i_src1"));
  logger->logAttribute("i_src2", dline->attribute("i_src2"));

  Simulation sim(simName, Logger::Level::debug);
  sim.setSystem(sys);
  sim.setDomain(Domain::EMT);
  sim.setTimeStep(timeStep);
  sim.setFinalTime(finalTime);
  sim.addLogger(logger);

  sim.run();
}

int main(int argc, char *argv[]) {
//   simElements();
  simDecouplingLine();
}
