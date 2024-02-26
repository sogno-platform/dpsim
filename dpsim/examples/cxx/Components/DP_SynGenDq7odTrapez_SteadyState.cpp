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
  Real timeStep = 0.0005;
  Real finalTime = 0.03;
  String name = "DP_SynGenDq7odTrapez_SteadyState";
  Logger::setLogDir("logs/" + name);
  std::cout << std::getenv("CPS_LOG_DIR");

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
  Real initMechPower = 300e6;
  Real initTerminalVolt = 24000 / sqrt(3) * sqrt(2);
  Real initVoltAngle = -PI / 2;

  // Define grid parameters
  Real Rload = 1.92;

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
  std::shared_ptr<Ph3::SynchronGeneratorDQ> gen =
      Ph3::SynchronGeneratorDQTrapez::make("DP_SynGen_dq_SteadyState_SynGen");
  gen->setParametersFundamentalPerUnit(
      nomPower, nomPhPhVoltRMS, nomFreq, poleNum, nomFieldCurr, Rs, Ll, Lmd,
      Lmq, Rfd, Llfd, Rkd, Llkd, Rkq1, Llkq1, Rkq2, Llkq2, H, initActivePower,
      initReactivePower, initTerminalVolt, initVoltAngle, initMechPower);
  gen->connect({n1});

  auto res = Ph3::SeriesResistor::make("R_load");
  res->setParameters(Rload);
  res->connect({SimNode::GND, n1});

  // System
  auto sys =
      SystemTopology(60, SystemNodeList{n1}, SystemComponentList{gen, res});

  // Simulation
  Simulation sim(name, Logger::Level::info);
  sim.setSystem(sys);
  sim.setTimeStep(timeStep);
  sim.setFinalTime(finalTime);
  sim.setDomain(Domain::DP);
  sim.setSolverType(Solver::Type::MNA);

  sim.run();

  return 0;
}
