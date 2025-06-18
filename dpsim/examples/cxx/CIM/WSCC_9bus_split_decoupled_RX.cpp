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

String decouple_line(SystemTopology &sys, const String &lineName, const String &node1, const String node2) {
  // auto origLine = sys.component<DP::Ph1::PiLine>(lineName);
  // Real Rline = origLine->attributeTyped<Real>("R_series")->get();
  // Real Lline = origLine->attributeTyped<Real>("L_series")->get();
  // Real Cline = origLine->attributeTyped<Real>("C_parallel")->get();

    Real Rline = 12.5;
    Real Lline = 0.16;
    Real Cline = 1e-6;

  sys.removeComponent(lineName);

  String dline_name = "dline_" + node1 + "_" + node2;

  auto line = Signal::DecouplingLine::make(
    "dline_" + node1 + "_" + node2, sys.node<DP::SimNode>(node1),
    sys.node<DP::SimNode>(node2), Rline, Lline, Cline, Logger::Level::debug
  );
  sys.addComponent(line);
  sys.addComponents(line->getLineComponents());

  return dline_name;
}

std::vector<String> split_decoupled(SystemTopology &sys, Real resistance,
                        Real inductance, Real capacitance) {

  String dline_75 = decouple_line(sys, "LINE75", "BUS5", "BUS7");
  // decouple_line(system, "LINE78", "BUS7", "BUS8");
  String dline_64 = decouple_line(sys, "LINE64", "BUS6", "BUS4");
  String dline_89 = decouple_line(sys, "LINE89", "BUS8", "BUS9");

  std::vector<String> nodes = {"BUS5", "BUS8", "BUS6"};
  std::vector<String> mdlines = {dline_75};

  return mdlines;
}

void simulateDecoupled(std::list<fs::path> filenames, Int threads,
                       Int seq = 0) {
  String simName = "WSCC_9bus_split_decoupled_RX_" + std::to_string(threads) + "_" + std::to_string(seq);
  Logger::setLogDir("logs/" + simName);

  CIM::Reader reader(simName, Logger::Level::debug, Logger::Level::debug);
  SystemTopology sys =
      reader.loadCIM(60, filenames, Domain::DP, PhaseType::Single,
                     CPS::GeneratorType::IdealVoltageSource);

  std::vector<String> dlines = split_decoupled(sys, 12.5, 0.16, 1e-6);

  // Logging
  auto logger = DataLogger::make(simName);
  // logger->logAttribute("BUS5.v", sys.node<DP::SimNode>("BUS5")->attribute("v"));
  // logger->logAttribute("BUS6.v", sys.node<DP::SimNode>("BUS6")->attribute("v"));
  // logger->logAttribute("BUS8.v", sys.node<DP::SimNode>("BUS8")->attribute("v"));
  for (Int bus  = 1; bus <= 9; bus++) {
    String attrName = "v" + std::to_string(bus);
    String nodeName = "BUS" + std::to_string(bus);
    logger->logAttribute(attrName, sys.node<DP::SimNode>(nodeName)->attribute("v"));
  }

  Simulation sim(simName, Logger::Level::debug);
  sim.setSystem(sys);
  sim.setTimeStep(0.0001);
  sim.setFinalTime(5e-4);
  sim.setDomain(Domain::DP);
  sim.doSplitSubnets(true);
  sim.doInitFromNodesAndTerminals(true);
  sim.addLogger(logger);
  if (threads > 0)
    sim.setScheduler(std::make_shared<OpenMPLevelScheduler>(threads));

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
      "/workspaces/dpsim_su/build/_deps/cim-data-src/WSCC-09/WSCC-09_RX", "CIMPATH");

  //for (Int copies = 0; copies < 10; copies++) {
  //	for (Int threads = 0; threads <= 12; threads = threads+2)
  //		simulateDecoupled(filenames, copies, threads);
  //}
  //simulateDecoupled(filenames, 19, 8);

  Int numThreads = 0;
  Int numSeq = 0;

  if (args.options.find("threads") != args.options.end())
    numThreads = args.getOptionInt("threads");
  if (args.options.find("seq") != args.options.end())
    numSeq = args.getOptionInt("seq");

  std::cout << "Simulate with " << numThreads
            << " threads, sequence number " << numSeq << std::endl;
  simulateDecoupled(filenames, numThreads, numSeq);
}
