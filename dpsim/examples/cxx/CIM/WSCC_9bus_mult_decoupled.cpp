/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <fstream>
#include <iostream>
#include <list>

#include <DPsim.h>
#include <dpsim/ThreadLevelScheduler.h>

using namespace DPsim;
using namespace CPS;

void multiply_decoupled(SystemTopology &sys, int copies, Real resistance,
                        Real inductance, Real capacitance) {

  sys.multiply(copies);
  std::vector<String> nodes = {"BUS5", "BUS8", "BUS6"};

  for (auto orig_node : nodes) {
    std::vector<String> nodeNames{orig_node};
    for (int i = 2; i <= copies + 1; i++) {
      nodeNames.push_back(orig_node + "_" + std::to_string(i));
    }
    nodeNames.push_back(orig_node);
    // if only a single copy is added, it does not really make sense to
    // "close the ring" by adding another line
    int nlines = copies == 1 ? 1 : copies + 1;

    for (int i = 0; i < nlines; i++) {
      auto line = Signal::DecouplingLine::make(
          "dline_" + orig_node + "_" + std::to_string(i),
          sys.node<DP::SimNode>(nodeNames[i]),
          sys.node<DP::SimNode>(nodeNames[i + 1]), resistance, inductance,
          capacitance, Logger::Level::info);
      sys.addComponent(line);
      sys.addComponents(line->getLineComponents());
    }
  }
}

void simulateDecoupled(std::list<fs::path> filenames, Int copies, Int threads,
                       Int seq = 0) {
  String simName = "WSCC_9bus_decoupled_" + std::to_string(copies) + "_" +
                   std::to_string(threads) + "_" + std::to_string(seq);
  Logger::setLogDir("logs/" + simName);

  CIM::Reader reader(simName, Logger::Level::off, Logger::Level::info);
  SystemTopology sys =
      reader.loadCIM(60, filenames, Domain::DP, PhaseType::Single,
                     CPS::GeneratorType::IdealVoltageSource);

  if (copies > 0)
    multiply_decoupled(sys, copies, 12.5, 0.16, 1e-6);

  Simulation sim(simName, Logger::Level::info);
  sim.setSystem(sys);
  sim.setTimeStep(0.0001);
  sim.setFinalTime(0.5);
  sim.setDomain(Domain::DP);
  if (threads > 0)
    sim.setScheduler(std::make_shared<OpenMPLevelScheduler>(threads));

  // Logging
  auto logger = DataLogger::make(simName);
  for (Int cop = 1; cop <= copies; cop++) {
    for (Int bus = 1; bus <= 9; bus++) {
      String attrName = "v" + std::to_string(bus) + "_" + std::to_string(cop);
      String nodeName = "BUS" + std::to_string(bus) + "_" + std::to_string(cop);
      if (cop == 1) {
        attrName = "v" + std::to_string(bus);
        nodeName = "BUS" + std::to_string(bus);
      }
      logger->logAttribute(attrName,
                           sys.node<DP::SimNode>(nodeName)->attribute("v"));
    }
  }
  sim.addLogger(logger);

  //std::ofstream of1("topology_graph.svg");
  //sys.topologyGraph().render(of1));

  sim.run();
  sim.logStepTimes(simName + "_step_times");
}

int main(int argc, char *argv[]) {
  CommandLineArgs args(argc, argv);

  std::list<fs::path> filenames;
  filenames = DPsim::Utils::findFiles(
      {"WSCC-09_RX_DI.xml", "WSCC-09_RX_EQ.xml", "WSCC-09_RX_SV.xml",
       "WSCC-09_RX_TP.xml"},
      "build/_deps/cim-data-src/WSCC-09/WSCC-09_RX", "CIMPATH");

  //for (Int copies = 0; copies < 10; copies++) {
  //	for (Int threads = 0; threads <= 12; threads = threads+2)
  //		simulateDecoupled(filenames, copies, threads);
  //}
  //simulateDecoupled(filenames, 19, 8);

  Int numCopies = 0;
  Int numThreads = 0;
  Int numSeq = 0;

  if (args.options.find("copies") != args.options.end())
    numCopies = args.getOptionInt("copies");
  if (args.options.find("threads") != args.options.end())
    numThreads = args.getOptionInt("threads");
  if (args.options.find("seq") != args.options.end())
    numSeq = args.getOptionInt("seq");

  std::cout << "Simulate with " << numCopies << " copies, " << numThreads
            << " threads, sequence number " << numSeq << std::endl;
  simulateDecoupled(filenames, numCopies, numThreads, numSeq);
}
