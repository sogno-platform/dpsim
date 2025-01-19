/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include "dpsim-models/EMT/EMT_Ph3_PiLine.h"
#include "dpsim-models/EMT/EMT_Ph3_RXLoad.h"
#include "dpsim-models/IdentifiedObject.h"
#include "dpsim-models/Signal/DecouplingIdealTransformerEMT.h"
#include <fstream>
#include <iostream>
#include <list>

#include <DPsim.h>
#include <dpsim/ThreadLevelScheduler.h>

using namespace DPsim;
using namespace CPS;

void decoupleNode(SystemTopology &sys, const String &nodeName, const IdentifiedObject::List &componentsAt1,
                    const IdentifiedObject::List &componentsAt2) {
  SimNode<Real>::List newNodes;
  SimPowerComp<Real>::List newComponents;

  auto intfNode = sys.node<EMT::SimNode>(nodeName);
  std::shared_ptr<TopologicalNode> nodeCopy1Topo = intfNode->clone(nodeName + "_1");
  std::shared_ptr<TopologicalNode> nodeCopy2Topo = intfNode->clone(nodeName + "_2");

  auto nodeCopy1 = std::dynamic_pointer_cast<SimNode<Real>>(nodeCopy1Topo);
  auto nodeCopy2 = std::dynamic_pointer_cast<SimNode<Real>>(nodeCopy2Topo);

  newNodes.push_back(nodeCopy1);
  newNodes.push_back(nodeCopy2);

  for (auto genComp : componentsAt1) {
    auto comp = std::dynamic_pointer_cast<SimPowerComp<Real>>(genComp);
    std::cout << "Cloning component: " << comp->name() << std::endl;
    auto compCopy = comp->clone(comp->name());

    SimNode<Real>::List nodeCopies;
    for (UInt nNode = 0; nNode < comp->terminalNumber(); nNode++) {
      String origNodeName = comp->node(nNode)->name();
      std::cout << "Processing components' node " << origNodeName << std::endl;
      if (origNodeName == nodeName) {
        nodeCopies.push_back(nodeCopy1);
      } else {
        nodeCopies.push_back(comp->node(nNode));
      }
    }

    compCopy->connect(nodeCopies);

    // update the terminal powers for powerflow initialization
    for (UInt nTerminal = 0; nTerminal < comp->terminalNumber();
          nTerminal++) {
      compCopy->terminal(nTerminal)->setPower(comp->terminal(nTerminal)->power());
    }
    newComponents.push_back(compCopy);
    sys.removeComponent(comp->name());
  }

  for (auto genComp : componentsAt2) {
    auto comp = std::dynamic_pointer_cast<SimPowerComp<Real>>(genComp);
    std::cout << "Cloning component: " << comp->name() << std::endl;
    auto compCopy = comp->clone(comp->name());

    SimNode<Real>::List nodeCopies;
    for (UInt nNode = 0; nNode < comp->terminalNumber(); nNode++) {
      String origNodeName = comp->node(nNode)->name();
      std::cout << "Processing components' node " << origNodeName << std::endl;
      if (origNodeName == nodeName) {
        nodeCopies.push_back(nodeCopy2);
      } else {
        nodeCopies.push_back(comp->node(nNode));
      }
    }

    compCopy->connect(nodeCopies);

    // update the terminal powers for powerflow initialization
    for (UInt nTerminal = 0; nTerminal < comp->terminalNumber();
          nTerminal++) {
      compCopy->terminal(nTerminal)->setPower(comp->terminal(nTerminal)->power());
    }
    newComponents.push_back(compCopy);
    sys.removeComponent(comp->name());
  }

  sys.removeNode(nodeName);

  for (auto node : newNodes)
    sys.addNode(node);
  for (auto comp : newComponents)
    sys.addComponent(comp);

  Eigen::MatrixXd i_0(1,1);
  i_0(0,0) = 0;

  auto idealTrafo = Signal::DecouplingIdealTransformerEMT::make("itm_" + nodeName,
                                                                Logger::Level::debug);
  idealTrafo->setParameters(nodeCopy1, nodeCopy2, 0, i_0);
  sys.addComponent(idealTrafo);
  sys.addComponents(idealTrafo->getComponents());
}

void doSim(String &name, SystemTopology &sys, Int threads, bool isDecoupled = false) {

  // Logging
  auto logger = DataLogger::make(name);
  for (Int bus = 1; bus <= 9; bus++) {
    String attrName = "v" + std::to_string(bus);
    String nodeName;
    if (isDecoupled && (bus==5 || bus==6 || bus==8)) {
      continue;
    } else {
      nodeName = "BUS" + std::to_string(bus);
    }
    logger->logAttribute(attrName, sys.node<EMT::SimNode>(nodeName)->attribute("v"));
  }

  if (isDecoupled) {
    logger->logAttribute("v5_1", sys.node<EMT::SimNode>("BUS5_1")->attribute("v"));
    logger->logAttribute("v5_2", sys.node<EMT::SimNode>("BUS5_2")->attribute("v"));
    logger->logAttribute("v6_1", sys.node<EMT::SimNode>("BUS5_1")->attribute("v"));
    logger->logAttribute("v6_2", sys.node<EMT::SimNode>("BUS5_2")->attribute("v"));
    logger->logAttribute("v8_1", sys.node<EMT::SimNode>("BUS8_1")->attribute("v"));
    logger->logAttribute("v8_2", sys.node<EMT::SimNode>("BUS8_2")->attribute("v"));
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
  String simNameMonolithic = "WSCC-9bus_monolithic_EMT";
  Logger::setLogDir("logs/" + simNameMonolithic);
  CIM::Reader readerMonolithic(simNameMonolithic, Logger::Level::debug, Logger::Level::debug);
  SystemTopology systemMonolithic =
      readerMonolithic.loadCIM(60, filenames, Domain::EMT, PhaseType::ABC,
                     CPS::GeneratorType::IdealVoltageSource);

  doSim(simNameMonolithic, systemMonolithic, 0);

  // Decoupled Simulation
  String simNameDecoupled = "WSCC_9bus_split_decoupled_node_EMT_" + std::to_string(numThreads) + "_" + std::to_string(numSeq);
  Logger::setLogDir("logs/" + simNameDecoupled);
  CIM::Reader readerDecoupled(simNameDecoupled, Logger::Level::debug, Logger::Level::debug);
  SystemTopology systemDecoupled = readerDecoupled.loadCIM(60, filenames, Domain::EMT, PhaseType::ABC,
                                   CPS::GeneratorType::IdealVoltageSource);

  IdentifiedObject::List components5_1;
  auto line75 = systemDecoupled.component<EMT::Ph3::PiLine>("LINE75");
  components5_1.push_back(line75);

  IdentifiedObject::List components5_2;
  auto line54 = systemDecoupled.component<EMT::Ph3::PiLine>("LINE54");
  components5_2.push_back(line54);
  auto load5 = systemDecoupled.component<EMT::Ph3::RXLoad>("LOAD5");
  components5_2.push_back(load5);

  IdentifiedObject::List components6_1;
  auto line96 = systemDecoupled.component<EMT::Ph3::PiLine>("LINE96");
  components6_1.push_back(line96);

  IdentifiedObject::List components6_2;
  auto line64 = systemDecoupled.component<EMT::Ph3::PiLine>("LINE64");
  components6_2.push_back(line64);
  auto load6 = systemDecoupled.component<EMT::Ph3::RXLoad>("LOAD6");
  components6_2.push_back(load6);

  IdentifiedObject::List components8_1;
  auto line78 = systemDecoupled.component<EMT::Ph3::PiLine>("LINE78");
  components8_1.push_back(line78);

  IdentifiedObject::List components8_2;
  auto line89 = systemDecoupled.component<EMT::Ph3::PiLine>("LINE89");
  components8_2.push_back(line89);
  auto load8 = systemDecoupled.component<EMT::Ph3::RXLoad>("LOAD8");
  components8_2.push_back(load8);

  decoupleNode(systemDecoupled, "BUS5", components5_1, components5_2);
  decoupleNode(systemDecoupled, "BUS6", components6_1, components6_2);
  decoupleNode(systemDecoupled, "BUS8", components8_1, components8_2);
  doSim(simNameDecoupled, systemDecoupled, numThreads, true);
}
