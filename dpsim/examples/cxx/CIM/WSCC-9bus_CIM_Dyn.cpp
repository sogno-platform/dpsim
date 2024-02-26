/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <iostream>
#include <list>

#include <DPsim.h>

using namespace DPsim;
using namespace CPS::DP;

int main(int argc, char *argv[]) {

  // Find CIM files
  std::list<fs::path> filenames;
  if (argc <= 1) {
    filenames = Utils::findFiles(
        {"WSCC-09_RX_DI.xml", "WSCC-09_RX_EQ.xml", "WSCC-09_RX_SV.xml",
         "WSCC-09_RX_TP.xml"},
        "build/_deps/cim-data-src/WSCC-09/WSCC-09_RX_Dyn", "CIMPATH");
  } else {
    filenames = std::list<fs::path>(argv + 1, argv + argc);
  }

  String simName = "WSCC-9bus_dyn";
  Logger::setLogDir("logs/" + simName);

  CPS::CIM::Reader reader(simName, Logger::Level::debug, Logger::Level::debug);
  SystemTopology sys =
      reader.loadCIM(60, filenames, Domain::DP, PhaseType::Single,
                     CPS::GeneratorType::TransientStability);

  // Logging
  auto logger = DataLogger::make(simName);
  logger->logAttribute("v1", sys.node<SimNode>("BUS1")->attribute("v"));
  logger->logAttribute("v2", sys.node<SimNode>("BUS2")->attribute("v"));
  logger->logAttribute("v3", sys.node<SimNode>("BUS3")->attribute("v"));
  logger->logAttribute("v4", sys.node<SimNode>("BUS4")->attribute("v"));
  logger->logAttribute("v5", sys.node<SimNode>("BUS5")->attribute("v"));
  logger->logAttribute("v6", sys.node<SimNode>("BUS6")->attribute("v"));
  logger->logAttribute("v7", sys.node<SimNode>("BUS7")->attribute("v"));
  logger->logAttribute("v8", sys.node<SimNode>("BUS8")->attribute("v"));
  logger->logAttribute("v9", sys.node<SimNode>("BUS9")->attribute("v"));
  logger->logAttribute(
      "wr_1",
      sys.component<Ph1::SynchronGeneratorTrStab>("GEN1")->attribute("w_r"));
  logger->logAttribute(
      "wr_2",
      sys.component<Ph1::SynchronGeneratorTrStab>("GEN2")->attribute("w_r"));
  logger->logAttribute(
      "wr_3",
      sys.component<Ph1::SynchronGeneratorTrStab>("GEN3")->attribute("w_r"));

  Simulation sim(simName, Logger::Level::info);
  sim.setSystem(sys);
  sim.setDomain(Domain::DP);
  sim.setTimeStep(0.0001);
  sim.setFinalTime(2);
  sim.doSteadyStateInit(true);
  sim.addLogger(logger);
  sim.run();

  //std::ofstream ofTopo("topology_graph.svg");
  //sys.topologyGraph().render(ofTopo);
  //std::ofstream ofDep("task_dependencies.svg");
  //sim.dependencyGraph().render(ofDep);

  return 0;
}
