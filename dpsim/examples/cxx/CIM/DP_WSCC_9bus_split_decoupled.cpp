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

String decoupleLine(SystemTopology &sys, const String &lineName, const String &node1, const String node2) {
  auto origLine = sys.component<DP::Ph1::PiLine>(lineName);
  Real Rline = origLine->attributeTyped<Real>("R_series")->get();
  Real Lline = origLine->attributeTyped<Real>("L_series")->get();
  Real Cline = origLine->attributeTyped<Real>("C_parallel")->get();

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

void doSim(String &name, SystemTopology &sys, Int threads) {

  // Logging
  auto logger = DataLogger::make(name);
  // logger->logAttribute("BUS5.v", sys.node<DP::SimNode>("BUS5")->attribute("v"));
  // logger->logAttribute("BUS6.v", sys.node<DP::SimNode>("BUS6")->attribute("v"));
  // logger->logAttribute("BUS8.v", sys.node<DP::SimNode>("BUS8")->attribute("v"));
  for (Int bus  = 1; bus <= 9; bus++) {
    String attrName = "v" + std::to_string(bus);
    String nodeName = "BUS" + std::to_string(bus);
    logger->logAttribute(attrName, sys.node<DP::SimNode>(nodeName)->attribute("v"));
  }

  Simulation sim(name, Logger::Level::debug);
  sim.setSystem(sys);
  sim.setTimeStep(0.0001);
  sim.setFinalTime(0.5);
  sim.setDomain(Domain::DP);
  sim.doSplitSubnets(true);
  sim.doInitFromNodesAndTerminals(true);
  sim.addLogger(logger);
  if (threads > 0)
    sim.setScheduler(std::make_shared<OpenMPLevelScheduler>(threads));

  //std::ofstream of1("topology_graph.svg");
  //sys.topologyGraph().render(of1));

  sim.run();
  sim.logStepTimes(name + "_step_times");
}

int main(int argc, char *argv[]) {
  CommandLineArgs args(argc, argv);

  std::list<fs::path> filenames;
  filenames = DPsim::Utils::findFiles(
      {"WSCC-09_DI.xml", "WSCC-09_EQ.xml", "WSCC-09_SV.xml",
       "WSCC-09_TP.xml"},
      "build/_deps/cim-data-src/WSCC-09/WSCC-09", "CIMPATH");

  Int numThreads = 0;
  Int numSeq = 0;

  if (args.options.find("threads") != args.options.end())
    numThreads = args.getOptionInt("threads");
  if (args.options.find("seq") != args.options.end())
    numSeq = args.getOptionInt("seq");

  std::cout << "Simulate with " << numThreads
            << " threads, sequence number " << numSeq << std::endl;

  // Monolithic Simulation
  String simNameMonolithic = "WSCC-9bus_monolithic_DP";
  Logger::setLogDir("logs/" + simNameMonolithic);
  CIM::Reader readerMonolithic(simNameMonolithic, Logger::Level::debug, Logger::Level::debug);
  SystemTopology systemMonolithic =
      readerMonolithic.loadCIM(60, filenames, Domain::DP, PhaseType::Single,
                     CPS::GeneratorType::IdealVoltageSource);

  doSim(simNameMonolithic, systemMonolithic, 0);

  // Decoupled Simulation
  String simNameDecoupled = "WSCC_9bus_split_decoupled_DP_" + std::to_string(numThreads) + "_" + std::to_string(numSeq);
  Logger::setLogDir("logs/" + simNameDecoupled);
  CIM::Reader readerDecoupled(simNameDecoupled, Logger::Level::debug, Logger::Level::debug);
  SystemTopology systemDecoupled = readerDecoupled.loadCIM(60, filenames, Domain::DP, PhaseType::Single,
                                   CPS::GeneratorType::IdealVoltageSource);

  String dline_75 = decoupleLine(systemDecoupled, "LINE75", "BUS5", "BUS7");
  // decouple_line(system, "LINE78", "BUS7", "BUS8");
  String dline_64 = decoupleLine(systemDecoupled, "LINE64", "BUS6", "BUS4");
  String dline_89 = decoupleLine(systemDecoupled, "LINE89", "BUS8", "BUS9");

  doSim(simNameDecoupled, systemDecoupled, numThreads);
}
