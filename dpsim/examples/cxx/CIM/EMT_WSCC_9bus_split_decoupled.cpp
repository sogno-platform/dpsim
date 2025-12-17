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

String decoupleLine(SystemTopology &sys, const String &lineName,
                    const String &node1, const String node2) {
  auto origLine = sys.component<EMT::Ph3::PiLine>(lineName);
  Matrix Rline = origLine->attributeTyped<Matrix>("R_series")->get();
  Matrix Lline = origLine->attributeTyped<Matrix>("L_series")->get();
  Matrix Cline = origLine->attributeTyped<Matrix>("C_parallel")->get();

  sys.removeComponent(lineName);

  String dline_name = "dline_" + node1 + "_" + node2;

  auto line = Signal::DecouplingLineEMT_Ph3::make(
      "dline_" + node1 + "_" + node2, Logger::Level::debug);

  line->setParameters(sys.node<EMT::SimNode>(node1),
                      sys.node<EMT::SimNode>(node2), Rline, Lline, Cline);
  sys.addComponent(line);
  sys.addComponents(line->getLineComponents());

  return dline_name;
}

void doSim(String &name, SystemTopology &sys, Int threads) {

  // Logging
  auto logger = DataLogger::make(name);
  logger->logAttribute("BUS5.v",
                       sys.node<EMT::SimNode>("BUS5")->attribute("v"));
  logger->logAttribute("BUS6.v",
                       sys.node<EMT::SimNode>("BUS6")->attribute("v"));
  logger->logAttribute("BUS8.v",
                       sys.node<EMT::SimNode>("BUS8")->attribute("v"));
  for (Int bus = 1; bus <= 9; bus++) {
    String attrName = "v" + std::to_string(bus);
    String nodeName = "BUS" + std::to_string(bus);
    logger->logAttribute(attrName,
                         sys.node<EMT::SimNode>(nodeName)->attribute("v"));
  }

  Simulation sim(name, Logger::Level::debug);
  sim.setSystem(sys);
  sim.setTimeStep(0.0001);
  sim.setFinalTime(0.5);
  sim.setDomain(Domain::EMT);
  sim.doSplitSubnets(true);
  sim.doInitFromNodesAndTerminals(true);
  sim.addLogger(logger);
  if (threads > 0)
#ifdef WITH_OPENMP
    sim.setScheduler(std::make_shared<OpenMPLevelScheduler>(threads));
#else
    sim.setScheduler(std::make_shared<ThreadLevelScheduler>(threads));
#endif
  //std::ofstream of1("topology_graph.svg");
  //sys.topologyGraph().render(of1));

  sim.run();
  sim.logStepTimes(name + "_step_times");
}

int main(int argc, char *argv[]) {
  CommandLineArgs args(argc, argv);

  std::list<fs::path> filenames;
  filenames = DPsim::Utils::findFiles(
      {"WSCC-09_DI.xml", "WSCC-09_EQ.xml", "WSCC-09_SV.xml", "WSCC-09_TP.xml"},
      "build/_deps/cim-data-src/WSCC-09/WSCC-09", "CIMPATH");

  Int numThreads = 0;
  Int numSeq = 0;

  if (args.options.find("threads") != args.options.end())
    numThreads = args.getOptionInt("threads");
  if (args.options.find("seq") != args.options.end())
    numSeq = args.getOptionInt("seq");

  std::cout << "Simulate with " << numThreads << " threads, sequence number "
            << numSeq << std::endl;

  // Monolithic Simulation
  String simNameMonolithic = "WSCC-9bus_monolithic_EMT";
  Logger::setLogDir("logs/" + simNameMonolithic);
  CIM::Reader readerMonolithic(simNameMonolithic, Logger::Level::debug,
                               Logger::Level::debug);
  SystemTopology systemMonolithic =
      readerMonolithic.loadCIM(60, filenames, Domain::EMT, PhaseType::ABC,
                               CPS::GeneratorType::IdealVoltageSource);

  doSim(simNameMonolithic, systemMonolithic, 0);

  // Decoupled Simulation
  String simNameDecoupled = "WSCC_9bus_split_decoupled_EMT_" +
                            std::to_string(numThreads) + "_" +
                            std::to_string(numSeq);
  Logger::setLogDir("logs/" + simNameDecoupled);
  CIM::Reader readerDecoupled(simNameDecoupled, Logger::Level::debug,
                              Logger::Level::debug);
  SystemTopology systemDecoupled =
      readerDecoupled.loadCIM(60, filenames, Domain::EMT, PhaseType::ABC,
                              CPS::GeneratorType::IdealVoltageSource);

  String dline_75 = decoupleLine(systemDecoupled, "LINE75", "BUS5", "BUS7");
  // decouple_line(system, "LINE78", "BUS7", "BUS8");
  String dline_64 = decoupleLine(systemDecoupled, "LINE64", "BUS6", "BUS4");
  String dline_89 = decoupleLine(systemDecoupled, "LINE89", "BUS8", "BUS9");

  doSim(simNameDecoupled, systemDecoupled, numThreads);
}
