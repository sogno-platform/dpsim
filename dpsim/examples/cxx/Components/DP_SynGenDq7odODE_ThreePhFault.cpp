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
using namespace CPS::DP::Ph3;

int main(int argc, char *argv[]) {
  // Define simulation parameters
  Real timeStep = 0.00005;
  Real finalTime = 0.3;
  String simName = "DP_SynGenDq7odODE_ThreePhFault";
  Logger::setLogDir("logs/" + simName);

  // Define machine parameters in per unit
  Real nomPower = 555e6;
  Real nomPhPhVoltRMS = 24e3;
  Real nomFreq = 60;
  Real nomFieldCurr = 1300;
  Int poleNum = 2;
  Real H = 3.7;
  Real Rs = 0.003;
  Real Ll = 0.15;
  Real Lmd = 1.6599;
  Real Lmq = 1.61;
  Real Rfd = 0.0006;
  Real Llfd = 0.1648;
  Real Rkd = 0.0284;
  Real Llkd = 0.1713;
  Real Rkq1 = 0.0062;
  Real Llkq1 = 0.7252;
  Real Rkq2 = 0.0237;
  Real Llkq2 = 0.125;
  // Initialization parameters
  Real initActivePower = 300e6;
  Real initReactivePower = 0;
  Real initTerminalVolt = 24000 / sqrt(3) * sqrt(2);
  Real initVoltAngle = -PI / 2;
  Real mechPower = 300e6;
  // Define grid parameters
  Real Rload = 1.92;
  Real BreakerOpen = 1e6;
  Real BreakerClosed = 0.001;

  // Nodes
  std::vector<Complex> initVoltN1 = std::vector<Complex>(
      {Complex(initTerminalVolt * cos(initVoltAngle),
               initTerminalVolt * sin(initVoltAngle)),
       Complex(initTerminalVolt * cos(initVoltAngle - 2 * PI / 3),
               initTerminalVolt * sin(initVoltAngle - 2 * PI / 3)),
       Complex(initTerminalVolt * cos(initVoltAngle + 2 * PI / 3),
               initTerminalVolt * sin(initVoltAngle + 2 * PI / 3))});
  auto n1 = SimNode::make("n1", PhaseType::ABC, initVoltN1);

  // Components
  auto gen =
      Ph3::SynchronGeneratorDQODE::make("DP_SynGen", Logger::Level::debug);
  gen->setParametersFundamentalPerUnit(
      nomPower, nomPhPhVoltRMS, nomFreq, poleNum, nomFieldCurr, Rs, Ll, Lmd,
      Lmq, Rfd, Llfd, Rkd, Llkd, Rkq1, Llkq1, Rkq2, Llkq2, H, initActivePower,
      initReactivePower, initTerminalVolt, initVoltAngle, mechPower);

  auto res = Ph3::SeriesResistor::make("R_load");
  res->setParameters(Rload);

  auto fault = Ph3::SeriesSwitch::make("Br_fault");
  fault->setParameters(BreakerOpen, BreakerClosed);
  fault->open();

  // Connections
  gen->connect({n1});
  res->connect({SimNode::GND, n1});
  fault->connect({SimNode::GND, n1});

  // Logging
  auto logger = DataLogger::make(simName);
  logger->logAttribute("v1", n1->attribute("v"));
  logger->logAttribute("i_gen", gen->attribute("i_intf"));
  logger->logAttribute("i_load", res->attribute("i_intf"));
  logger->logAttribute("wr_gen", gen->attribute("w_r"));

  // System
  auto sys = SystemTopology(60, SystemNodeList{n1},
                            SystemComponentList{gen, res, fault});
  Simulation sim(simName, Logger::Level::info);
  sim.setSystem(sys);
  sim.setTimeStep(timeStep);
  sim.setFinalTime(finalTime);
  sim.setDomain(Domain::DP);
  sim.setSolverType(Solver::Type::MNA);
  sim.addLogger(logger);

  // Events
  auto sw1 = SwitchEvent::make(0.1, fault, true);
  sim.addEvent(sw1);
  auto sw2 = SwitchEvent::make(0.2, fault, false);
  sim.addEvent(sw2);

  sim.run();

  return 0;
}
