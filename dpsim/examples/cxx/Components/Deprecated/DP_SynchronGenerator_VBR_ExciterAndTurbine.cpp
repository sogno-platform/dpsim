/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <DPsim.h>

using namespace DPsim;
using namespace DPsim::DP::Ph1;

int main(int argc, char *argv[]) {
  // Define machine parameters in per unit
  Real nomPower = 555e6;
  Real nomPhPhVoltRMS = 24e3;
  Real nomFreq = 60;
  Real nomFieldCurr = 1300;
  Int poleNum = 2;
  Real J = 2.8898e+04;
  Real H = 3.7;

  Real Rs = 0.003;
  Real Ll = 0.15;
  Real Lmd = 1.6599;
  Real Lmd0 = 1.6599;
  Real Lmq = 1.61;
  Real Lmq0 = 1.61;
  Real Rfd = 0.0006;
  Real Llfd = 0.1648;
  Real Rkd = 0.0284;
  Real Llkd = 0.1713;
  Real Rkq1 = 0.0062;
  Real Llkq1 = 0.7252;
  Real Rkq2 = 0.0237;
  Real Llkq2 = 0.125;
  //Real Rkq2 = 0;
  //Real Llkq2 = 0;

  //Exciter
  Real Ka = 46;
  Real Ta = 0.06;
  Real Ke = -0.043478260869565223;
  Real Te = 0.46;
  Real Kf = 0.1;
  Real Tf = 1;
  Real Tr = 0.02;

  // Turbine
  Real Ta_t = 0.3;
  Real Fa = 0.3;
  Real Tb = 7;
  Real Fb = 0.3;
  Real Tc = 0.2;
  Real Fc = 0.4;
  Real Tsr = 0.1;
  Real Tsm = 0.3;
  Real Kg = 20;

  // Set up simulation
  Real tf, dt, t;
  Real om = 2.0 * M_PI * 60.0;
  tf = 20;
  dt = 0.01;
  t = 0;
  Int downSampling = 1;

  // Declare circuit components
  String mGeneratorName = "DP_VBR_" + std::to_string(dt);
  Component::Ptr gen = SynchronGeneratorVBRNew::make(
      mGeneratorName, 0, 1, 2, nomPower, nomPhPhVoltRMS, nomFreq, poleNum,
      nomFieldCurr, Rs, Ll, Lmd, Lmd0, Lmq, Lmq0, Rfd, Llfd, Rkd, Llkd, Rkq1,
      Llkq1, Rkq2, Llkq2, H, Logger::Level::info);

  Real loadRes = 1.92;

  //Load
  Component::Ptr r1 = Resistor::make("r1", 0, GND, loadRes);
  Component::Ptr r2 = Resistor::make("r2", 1, GND, loadRes);
  Component::Ptr r3 = Resistor::make("r3", 2, GND, loadRes);

  SystemComponentList comps = {gen, r1, r2, r3};

  // Declare circuit components for resistance change
  Real breakerRes = 19.2 + 0.001;
  Component::Ptr rBreaker1 = Resistor::make("rbreak1", 0, GND, breakerRes);
  Component::Ptr rBreaker2 = Resistor::make("rbreak2", 1, GND, breakerRes);
  Component::Ptr rBreaker3 = Resistor::make("rbreak3", 2, GND, breakerRes);

  SystemComponentList compsBreakerOn = {
      gen, r1, r2, r3, rBreaker1, rBreaker2, rBreaker3,
  };

  String mSimulationName = "DP_SynchronGenerator_VBR_" + std::to_string(dt);
  SynGenSimulation sim(mSimulationName, Logger::Level::info);
  sim.setSystem(comps);
  sim.setTimeStep(dt);
  sim.setFinalTime(tf);
  sim.setDomain(Domain::DP);
  sim.setSolverType(Solver::Type::MNA);
  sim.setNumericalMethod(NumericalMethod::Trapezoidal_flux);
  sim.addSystemTopology(compsBreakerOn);
  sim.switchSystemMatrix(0);

  // Initialize generator
  Real initActivePower = 300e6;
  Real initReactivePower = 0;
  Real initTerminalVolt = 24000 / sqrt(3) * sqrt(2);
  Real initVoltAngle = -DPS_PI / 2;
  Real mechPower = 300e6;
  auto genPtr = std::dynamic_pointer_cast<DP::Ph3SynchronGeneratorVBRNew>(gen);
  genPtr->initialize(om, dt, initActivePower, initReactivePower,
                     initTerminalVolt, initVoltAngle, mechPower);
  genPtr->AddExciter(Ta, Ka, Te, Ke, Tf, Kf, Tr, Lmd, Rfd);
  genPtr->AddGovernor(Ta_t, Tb, Tc, Fa, Fb, Fc, Kg, Tsr, Tsm,
                      initActivePower / nomPower, mechPower / nomPower);

  std::cout << "A matrix:" << std::endl;
  std::cout << sim.systemMatrix() << std::endl;
  std::cout << "vt vector:" << std::endl;
  std::cout << sim.leftSideVector() << std::endl;
  std::cout << "j vector:" << std::endl;
  std::cout << sim.rightSideVector() << std::endl;

  Real lastLogTime = 0;
  Real logTimeStep = 0.00005;
  sim.setSwitchTime(0.1, 1);
  //sim.setSwitchTime(0.2, 0);

  sim.run();

  return 0;
}
