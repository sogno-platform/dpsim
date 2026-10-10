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
using namespace CPS::DP::Ph1;

int main(int argc, char *argv[]) {
  Real timeStep = 0.0001;
  Real finalTime = 0.002;
  String simName = "DP_Transformer_Harmonics";
  Logger::setLogDir("logs/" + simName);

  Matrix frequencies(2, 1);
  frequencies << 50, 250;

  auto n1 = SimNode::make("n1");
  auto n2 = SimNode::make("n2");

  Logger::Level level = Logger::Level::off;

  auto vs = VoltageSource::make("vs", level);
  vs->setParameters(Complex(1000, 0));

  auto trafo = Transformer::make("trafo", level);
  trafo->setParameters(1000, 1000, 1e6, 1.0, 0.0, 1.0, 0.01);

  auto load = Resistor::make("load", level);
  load->setParameters(100);

  vs->connect({SimNode::GND, n1});
  trafo->connect({n1, n2});
  load->connect({n2, SimNode::GND});

  auto sys = SystemTopology(50, frequencies, SystemNodeList{n1, n2},
                            SystemComponentList{vs, trafo, load});

  auto logger = DataLogger::make(simName);
  logger->logAttribute("v1", n1->attribute("v"));
  logger->logAttribute("v2", n2->attribute("v"));

  Simulation sim(simName, level);
  sim.setSystem(sys);
  sim.setTimeStep(timeStep);
  sim.setFinalTime(finalTime);
  sim.doFrequencyParallelization(true);
  sim.addLogger(logger);

  sim.run();
}
