/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include "dpsim-models/Definitions.h"
#include "dpsim-models/SP/SP_Ph1_PiLine.h"
#include "dpsim-models/SP/SP_Ph1_Resistor.h"
#include "dpsim-models/SP/SP_Ph1_VoltageSource.h"
#include "dpsim-models/Signal/DecouplingIdealTransformer_SP_Ph1.h"
#include "dpsim-models/SimNode.h"
#include "dpsim/Definitions.h"
#include <DPsim.h>
#include <memory>

using namespace DPsim;
using namespace CPS::SP;
using namespace CPS::SP::Ph1;
using namespace CPS::Signal;

void simMonolithic() {
  Real timeStep = 0.00005;
  Real finalTime = 0.1;
  String simName = "SP_DecouplingITM_Ph1_withPiLine_Monolithic";
  Logger::setLogDir("logs/" + simName);

  // Nodes
  auto n0 = SimNode::make("n0");
  auto n1 = SimNode::make("n1");
  auto n2 = SimNode::make("n2");

  Real R_1_R = 6000;

  // Parametrization of components
  Real r = 0.0529;
  Real x_l = 0.529;
  Real b = 3.308e-6;

  Real l_length = 220;

  Real resistance = l_length * r;
  Real inductance = l_length * (x_l / (2 * PI * 50));
  Real capacitance = l_length * (b / (2 * PI * 50));
  Real conductance = 0;

  // Real resistance = 5;
  // Real inductance = 0.16;
  // Real capacitance = 1.0e-6;
  // Real conductance = 1.0e-6;

  // Components
  auto vs = VoltageSource::make("v_1", Logger::Level::debug);
  vs->setParameters(Complex(100000, 0), 50);

  auto R_1 = Resistor::make("R_1", Logger::Level::debug);
  R_1->setParameters(R_1_R);

  auto line = PiLine::make("line", Logger::Level::debug);
  line->setParameters(resistance, inductance, capacitance, conductance);

  auto load = Resistor::make("R_load", Logger::Level::debug);
  load->setParameters(10000);

  // Topology
  vs->connect({SimNode::GND, n0});
  R_1->connect({n0, n1});
  line->connect({n1, n2});
  load->connect({n2, SimNode::GND});

  auto sys = SystemTopology(50, SystemNodeList{n0, n1, n2},
                            SystemComponentList{vs, R_1, line, load});

  // Logging
  auto logger = DataLogger::make(simName);
  logger->logAttribute("v1", n1->attribute("v"));
  logger->logAttribute("v2", n2->attribute("v"));
  logger->logAttribute("iline", line->attribute("i_intf"));
  logger->logAttribute("iload", load->attribute("i_intf"));

  Simulation sim(simName, Logger::Level::debug);
  sim.setSystem(sys);
  sim.setDomain(Domain::SP);
  sim.setTimeStep(timeStep);
  sim.setFinalTime(finalTime);
  sim.addLogger(logger);

  sim.run();
}

// void simElements() {
//   Real timeStep = 0.00005;
//   Real finalTime = 0.2;
//   String simName = "SP_DecouplingITM_Ph1_withPiLine_Elements";
//   Logger::setLogDir("logs/" + simName);

//   // Nodes
//   auto n0 = SimNode::make("n0");
//   auto n1 = SimNode::make("n1");
//   auto n2_1 = SimNode::make("n2_1");
//   auto n2_2 = SimNode::make("n2_2");

//   Real R_1_R = 60;

//   // Parametrization of components
//   Real resistance = 5;
//   Real inductance = 0.16;
//   Real capacitance = 1.0e-6;
//   Real conductance = 1.0e-6;

//   // Components
//   auto vs = VoltageSource::make("v_1", Logger::Level::debug);
//   vs->setParameters(Complex(10000, 0), 50);

//   auto R_1 = Resistor::make("R_1", Logger::Level::debug);
//   R_1->setParameters(R_1_R);

//   auto line = PiLine::make("line", Logger::Level::debug);
//   line->setParameters(resistance, inductance, capacitance, conductance);

//   auto v_itm = VoltageSource::make("v_itm");
//   auto i_itm = CurrentSource::make("i_itm");

//   auto load = Resistor::make("R_load", Logger::Level::debug);
//   load->setParameters(10000);

//   // Topology
//   vs->connect({SimNode::GND, n0});
//   R_1->connect({n0, n1});
//   line->connect({n1, n2_1});
//   v_itm->connect({SimNode::GND, n2_1});
//   i_itm->connect({SimNode::GND, n2_2});
//   load->connect({n2_2, SimNode::GND});

//   auto sys = SystemTopology(50, SystemNodeList{n0, n1, n2_1, n2_2},
//                             SystemComponentList{vs, R_1, line, v_itm, i_itm, load});

//   // Logging
//   auto logger = DataLogger::make(simName);
//   logger->logAttribute("v1", n1->attribute("v"));
//   logger->logAttribute("v2_1", n2_1->attribute("v"));
//   logger->logAttribute("v2_2", n2_2->attribute("v"));
//   logger->logAttribute("iline", line->attribute("i_intf"));
//   logger->logAttribute("iload", load->attribute("i_intf"));

//   Simulation sim(simName, Logger::Level::debug);
//   sim.setSystem(sys);
//   sim.setDomain(Domain::SP);
//   sim.setTimeStep(timeStep);
//   sim.setFinalTime(finalTime);
//   sim.addLogger(logger);

//   sim.run();
// }

void simITM_SP_Ph1() {
  Real timeStep = 0.00005;
  Real finalTime = 0.1;
  String simName = "SP_DecouplingITM_Ph1_withPiLine_Component";
  Logger::setLogDir("logs/" + simName);

  // Nodes
  auto n0 = SimNode::make("n0");
  auto n1_1 = SimNode::make("n1_1");
  auto n1_2 = SimNode::make("n1_2");
  auto n2 = SimNode::make("n2");

  Real R_1_R = 6000;

  // Parametrization of components
  Real r = 0.0529;
  Real x_l = 0.529;
  Real b = 3.308e-6;

  Real l_length = 220;

  Real resistance = l_length * r;
  Real inductance = l_length * (x_l / (2 * PI * 50));
  Real capacitance = l_length * (b / (2 * PI * 50));
  Real conductance = 0;

  // Real resistance = 5;
  // Real inductance = 0.16;
  // Real capacitance = 1.0e-6;
  // Real conductance = 1.0e-6;

  // Components
  auto vs = VoltageSource::make("v_1", Logger::Level::debug);
  vs->setParameters(Complex(100000, 0), 50);

  auto R_1 = Resistor::make("R_1", Logger::Level::debug);
  R_1->setParameters(R_1_R);

  auto line = PiLine::make("line", Logger::Level::debug);
  line->setParameters(resistance, inductance, capacitance, conductance);

  auto itm = CPS::Signal::DecouplingIdealTransformer_SP_Ph1::make(
      "itm", Logger::Level::debug);
  itm->setParameters(n1_1, n1_2, 0, Matrix::Zero(1, 1), Complex(0, 0),
                     CPS::CouplingMethod::DELAY);

  auto load = Resistor::make("R_load", Logger::Level::debug);
  load->setParameters(10000);

  // n1_1->setInitialVoltage(100000);
  // n1_2->setInitialVoltage(100000);

  // Topology
  vs->connect({SimNode::GND, n0});
  R_1->connect({n0, n1_1});
  line->connect({n1_2, n2});
  load->connect({n2, SimNode::GND});

  auto sys = SystemTopology(50, SystemNodeList{n0, n1_1, n1_2, n2},
                            SystemComponentList{vs, R_1, line, itm, load});
  sys.addComponents(itm->getComponents());
  sys.addNode(itm->getVirtualNode());

  // Logging
  auto logger = DataLogger::make(simName);
  logger->logAttribute("v0", n0->attribute("v"));
  logger->logAttribute("v1_1", n1_1->attribute("v"));
  logger->logAttribute("v1_2", n1_2->attribute("v"));
  logger->logAttribute("v2", n2->attribute("v"));
  logger->logAttribute("iline", line->attribute("i_intf"));
  logger->logAttribute("iref_itm", itm->attribute("i_ref"));
  logger->logAttribute("i_intf_itm", itm->attribute("i_intf"));
  logger->logAttribute("vref_itm", itm->attribute("v_ref"));
  logger->logAttribute("v_intf_itm", itm->attribute("v_intf"));
  logger->logAttribute("iload", load->attribute("i_intf"));

  Simulation sim(simName, Logger::Level::debug);
  sim.setSystem(sys);
  sim.setDomain(Domain::SP);
  sim.doSplitSubnets(true);
  sim.setTimeStep(timeStep);
  sim.setFinalTime(finalTime);
  sim.addLogger(logger);

  sim.run();
}

int main(int argc, char *argv[]) {
  simMonolithic();
  simITM_SP_Ph1();
}
