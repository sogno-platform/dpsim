/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <DPsim.h>
#include <dpsim/ThreadLevelScheduler.h>

using namespace DPsim;
using namespace CPS::DP;
using namespace CPS::DP::Ph1;

int main(int argc, char *argv[]) {
  CommandLineArgs args(argc, argv);
  std::cout << "Simulate sequence " << args.getOptionInt("seq") << std::endl;

  // Define simulation scenario
  Real timeStep = 0.000001;
  Real finalTime = 0.05;
  String simName = "DP_Inverter_Grid_Sequential_FreqSplit_s" +
                   std::to_string(args.getOptionInt("seq"));
  Logger::setLogDir("logs/" + simName);

  // Set system frequencies
  //Matrix frequencies(5,1);
  //frequencies << 50, 19850, 19950, 20050, 20150;
  Matrix frequencies(9, 1);
  frequencies << 50, 19850, 19950, 20050, 20150, 39750, 39950, 40050, 40250;
  //Matrix frequencies(15,1);
  //frequencies << 50,
  //			19850, 19950, 20050, 20150,
  //			39750, 39850, 39950, 40050, 40150, 40250,
  //			59850, 59950, 60050, 60150;

  // Nodes
  auto n1 = SimNode::make("n1");
  auto n2 = SimNode::make("n2");
  auto n3 = SimNode::make("n3");
  auto n4 = SimNode::make("n4");
  auto n5 = SimNode::make("n5");

  Logger::Level level = Logger::Level::off;

  // Components
  auto inv = Inverter::make("inv", level);
  inv->setParameters(
      std::vector<CPS::Int>{2, 2, 2, 2, 4, 4, 4, 4},
      std::vector<CPS::Int>{-3, -1, 1, 3, -5, -1, 1, 5},
      //std::vector<CPS::Int>{2,2,2,2,4,4,4,4,4,4,6,6,6,6},
      //std::vector<CPS::Int>{-3,-1,1,3,-5,-3,-1,1,3,5,-3,-1,1,3},
      360, 0.87, 0);
  auto r1 = Resistor::make("r1", level);
  r1->setParameters(0.1);
  auto l1 = Inductor::make("l1", level);
  l1->setParameters(600e-6);
  auto r2 = Resistor::make("r2", level);
  Real r2g = 0.1 + 0.001;
  r2->setParameters(r2g);
  auto l2 = Inductor::make("l2", level);
  Real l2g = 150e-6 + 0.001 / (2. * PI * 50.);
  l2->setParameters(l2g);
  auto c1 = Capacitor::make("c1", level);
  c1->setParameters(10e-6);
  auto rc = Capacitor::make("rc", level);
  rc->setParameters(1e-6);
  auto grid = VoltageSource::make("grid", level);
  grid->setParameters(Complex(0, -311.1270));

  // Topology
  inv->connect({n1});
  r1->connect({n1, n2});
  l1->connect({n2, n3});
  c1->connect({SimNode::GND, n3});
  rc->connect({SimNode::GND, n3});
  r2->connect({n3, n4});
  l2->connect({n4, n5});
  grid->connect({SimNode::GND, n5});

  // Define system topology
  auto sys =
      SystemTopology(50, frequencies, SystemNodeList{n1, n2, n3, n4, n5},
                     SystemComponentList{inv, r1, l1, r2, l2, c1, rc, grid});

  Simulation sim(simName, level);
  sim.setSystem(sys);
  sim.setTimeStep(timeStep);
  sim.setFinalTime(finalTime);
  sim.doFrequencyParallelization(true);

  sim.run();
  sim.logStepTimes(simName + "_step_times");
}
