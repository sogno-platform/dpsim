/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include "dpsim-models/Definitions.h"
#include "dpsim-models/EMT/EMT_Ph3_Capacitor.h"
#include "dpsim-models/EMT/EMT_Ph3_Resistor.h"
#include "dpsim-models/Signal/DecouplingIdealTransformer_EMT_Ph3.h"
#include "dpsim-models/SimNode.h"
#include <iostream>

#include <DPsim.h>
#include <dpsim/ThreadLevelScheduler.h>

using namespace DPsim;
using namespace CPS;

void decoupleNode(SystemTopology &sys, const String &nodeName,
                  const IdentifiedObject::List &componentsAt1,
                  const IdentifiedObject::List &componentsAt2, Real ITMDelay,
                  String method, Matrix irLine_0, Matrix i_inf_0) {

  CouplingMethod cosimMethod = CouplingMethod::DELAY;

  if (method == "extrapolation-zoh")
    cosimMethod = CouplingMethod::EXTRAPOLATION_ZOH;
  else if (method == "extrapolation-linear")
    cosimMethod = CouplingMethod::EXTRAPOLATION_LINEAR;

  SimNode<Real>::List newNodes;
  SimPowerComp<Real>::List newComponents;

  auto intfNode = sys.node<EMT::SimNode>(nodeName);
  std::shared_ptr<TopologicalNode> nodeCopy1Topo =
      intfNode->clone(nodeName + "_1");
  std::shared_ptr<TopologicalNode> nodeCopy2Topo =
      intfNode->clone(nodeName + "_2");

  auto nodeCopy1 = std::dynamic_pointer_cast<SimNode<Real>>(nodeCopy1Topo);
  auto nodeCopy2 = std::dynamic_pointer_cast<SimNode<Real>>(nodeCopy2Topo);

  newNodes.push_back(nodeCopy1);
  newNodes.push_back(nodeCopy2);

  for (auto genComp : componentsAt1) {
    auto comp = std::dynamic_pointer_cast<SimPowerComp<Real>>(genComp);
    auto compCopy = comp->clone(comp->name());

    SimNode<Real>::List nodeCopies;
    for (UInt nNode = 0; nNode < comp->terminalNumber(); nNode++) {
      String origNodeName = comp->node(nNode)->name();
      if (origNodeName == nodeName) {
        nodeCopies.push_back(nodeCopy1);
      } else {
        nodeCopies.push_back(comp->node(nNode));
      }
    }

    compCopy->connect(nodeCopies);

    // update the terminal powers for powerflow initialization
    for (UInt nTerminal = 0; nTerminal < comp->terminalNumber(); nTerminal++) {
      compCopy->terminal(nTerminal)->setPower(
          comp->terminal(nTerminal)->power());
    }
    newComponents.push_back(compCopy);
    sys.removeComponent(comp->name());
  }

  for (auto genComp : componentsAt2) {
    auto comp = std::dynamic_pointer_cast<SimPowerComp<Real>>(genComp);
    auto compCopy = comp->clone(comp->name());

    SimNode<Real>::List nodeCopies;
    for (UInt nNode = 0; nNode < comp->terminalNumber(); nNode++) {
      String origNodeName = comp->node(nNode)->name();
      if (origNodeName == nodeName) {
        nodeCopies.push_back(nodeCopy2);
      } else {
        nodeCopies.push_back(comp->node(nNode));
      }
    }

    compCopy->connect(nodeCopies);

    // update the terminal powers for powerflow initialization
    for (UInt nTerminal = 0; nTerminal < comp->terminalNumber(); nTerminal++) {
      compCopy->terminal(nTerminal)->setPower(
          comp->terminal(nTerminal)->power());
    }
    newComponents.push_back(compCopy);
    sys.removeComponent(comp->name());
  }

  sys.removeNode(nodeName);

  for (auto node : newNodes)
    sys.addNode(node);
  for (auto comp : newComponents)
    sys.addComponent(comp);

  auto idealTrafo = Signal::DecouplingIdealTransformer_EMT_Ph3::make(
      "itm_" + nodeName, Logger::Level::debug);
  idealTrafo->setParameters(nodeCopy1, nodeCopy2, ITMDelay, irLine_0.real(),
                            i_inf_0, cosimMethod);
  sys.addComponent(idealTrafo);
  sys.addComponents(idealTrafo->getComponents());
  sys.addNode(idealTrafo->getVirtualNode());
}

void doSim(String &name, SystemTopology &sys, Int threads, Real ts,
           bool isDecoupled = false) {

  // Logging
  auto logger = DataLogger::make(name);
  logger->logAttribute("v_0", sys.node<EMT::SimNode>("n0")->attribute("v"));
  logger->logAttribute("v_1", sys.node<EMT::SimNode>("n1")->attribute("v"));
  logger->logAttribute(
      "i_rline", sys.component<EMT::Ph3::Resistor>("r_line")->mIntfCurrent);

  if (isDecoupled) {
    logger->logAttribute("v_2_1",
                         sys.node<EMT::SimNode>("n2_1")->attribute("v"));
    logger->logAttribute("v_2_2",
                         sys.node<EMT::SimNode>("n2_2")->attribute("v"));
    logger->logAttribute(
        "i_intf",
        sys.component<Signal::DecouplingIdealTransformer_EMT_Ph3>("itm_n2")
            ->attribute("i_intf"));
    logger->logAttribute(
        "i_ref",
        sys.component<Signal::DecouplingIdealTransformer_EMT_Ph3>("itm_n2")
            ->attribute("i_ref"));
    logger->logAttribute(
        "v_intf",
        sys.component<Signal::DecouplingIdealTransformer_EMT_Ph3>("itm_n2")
            ->attribute("v_intf"));
    logger->logAttribute(
        "v_ref",
        sys.component<Signal::DecouplingIdealTransformer_EMT_Ph3>("itm_n2")
            ->attribute("v_ref"));
  } else {
    logger->logAttribute("v_2", sys.node<EMT::SimNode>("n2")->attribute("v"));
  }

  Simulation sim(name, Logger::Level::debug);
  sim.setSystem(sys);
  sim.setTimeStep(ts);
  sim.setFinalTime(1.2);
  sim.setDomain(Domain::EMT);
  sim.doSplitSubnets(true);
  sim.doInitFromNodesAndTerminals(true);
  sim.addLogger(logger);
  if (threads > 0)
    sim.setScheduler(std::make_shared<OpenMPLevelScheduler>(threads));

  sim.run();
  sim.logStepTimes(name + "_step_times");
}

SystemTopology buildTopology(String &name, Matrix r1_r, Matrix c1_c,
                             Matrix rLine_r, Matrix r3_r, Matrix c2_c,
                             MatrixComp n1_v0, MatrixComp n2_v0) {
  CPS::Logger::setLogDir("logs/" + name);

  // Nodes
  auto gnd = EMT::SimNode::GND;
  auto n0 = EMT::SimNode::make("n0", PhaseType::ABC);
  auto n1 = EMT::SimNode::make("n1", PhaseType::ABC);
  auto n2 = EMT::SimNode::make("n2", PhaseType::ABC);

  // Components
  auto vs = EMT::Ph3::VoltageSource::make("vs");
  vs->setParameters(
      Math::singlePhaseVariableToThreePhase(Complex(1 * PEAK1PH_TO_RMS3PH, 0)),
      50);
  auto r1 = EMT::Ph3::Resistor::make("r_1");
  r1->setParameters(r1_r);
  auto c1 = EMT::Ph3::Capacitor::make("c_1");
  c1->setParameters(c1_c);
  auto rLine = EMT::Ph3::Resistor::make("r_line");
  rLine->setParameters(rLine_r);
  auto r3 = EMT::Ph3::Resistor::make("r_3");
  r3->setParameters(r3_r);
  auto c2 = EMT::Ph3::Capacitor::make("c_2");
  c2->setParameters(c2_c);

  n1->setInitialVoltage(n1_v0 * PEAK1PH_TO_RMS3PH);
  n2->setInitialVoltage(n2_v0 * PEAK1PH_TO_RMS3PH);

  // Topology
  vs->connect({gnd, n0});
  r1->connect({n0, n1});
  rLine->connect({n2, n1});
  c1->connect({n1, gnd});
  r3->connect({n2, gnd});
  c2->connect({n2, gnd});

  auto sys = SystemTopology(50, SystemNodeList{gnd, n0, n1, n2},
                            SystemComponentList{vs, r1, c1, rLine, c2, r3});

  return sys;
}

int main(int argc, char *argv[]) {
  CommandLineArgs args(argc, argv);

  Int numThreads = 0;
  Int numSeq = 0;
  Real timeStep = 0.00005;
  Real delay = 0.0001;
  MatrixComp i_intf_0 = MatrixComp::Zero(3, 1);
  String cosimMethod = "delay";
  String prefix = "1e-4";

  if (args.options.find("threads") != args.options.end())
    numThreads = args.getOptionInt("threads");
  if (args.timeStep)
    timeStep = args.timeStep;
  if (args.options.find("delay") != args.options.end())
    delay = args.getOptionReal("delay");
  if (args.options.find("i-intf-0") != args.options.end())
    i_intf_0 = Math::singlePhaseVariableToThreePhase(
        Complex(args.getOptionReal("i-intf-0"), 0));
  if (args.options.find("method") != args.options.end())
    cosimMethod = args.getOptionString("method");
  if (args.options.find("prefix") != args.options.end())
    prefix = args.getOptionString("prefix");

  std::cout << "Simulate with " << numThreads << " threads, sequence number "
            << numSeq << ", co-simulation method " << cosimMethod << std::endl;

  Matrix r1_r_1 = Math::singlePhaseParameterToThreePhase(0.1);
  Matrix c1_c_1 = Math::singlePhaseParameterToThreePhase(1);
  Matrix rLine_r_1 = Math::singlePhaseParameterToThreePhase(0.1);
  Matrix r3_r_1 = Math::singlePhaseParameterToThreePhase(1);
  Matrix c2_c_1 = Math::singlePhaseParameterToThreePhase(1);

  // Initial conditions, given by the problem
  MatrixComp n1_v0_1 = Math::singlePhaseVariableToThreePhase(Complex(0.0, 0));
  MatrixComp n2_v0_1 = Math::singlePhaseVariableToThreePhase(Complex(0.0, 0));

  MatrixComp irLine_0_1 = rLine_r_1.inverse() * (n1_v0_1 - n2_v0_1);

  // Monolithic Simulation
  String simNameMonolithic = "EMT_RC_monolithic_Ph3";
  Logger::setLogDir("logs/" + simNameMonolithic);
  SystemTopology systemMonolithic =
      buildTopology(simNameMonolithic, r1_r_1, c1_c_1, rLine_r_1, r3_r_1,
                    c2_c_1, n1_v0_1, n2_v0_1);

  doSim(simNameMonolithic, systemMonolithic, 0, timeStep);

  // Decoupled Simulation
  String simNameDecoupled = "EMT_RC_split_decoupled_Ph3_" + prefix + "_" +
                            std::to_string(numThreads) + "_" +
                            std::to_string(numSeq);
  Logger::setLogDir("logs/" + simNameDecoupled);
  SystemTopology systemDecoupled =
      buildTopology(simNameDecoupled, r1_r_1, c1_c_1, rLine_r_1, r3_r_1, c2_c_1,
                    n1_v0_1, n2_v0_1);

  IdentifiedObject::List components1;

  auto rLine = systemDecoupled.component<EMT::Ph3::Resistor>("r_line");
  components1.push_back(rLine);

  IdentifiedObject::List components2;
  auto c2 = systemDecoupled.component<EMT::Ph3::Capacitor>("c_2");
  components2.push_back(c2);
  auto r3 = systemDecoupled.component<EMT::Ph3::Resistor>("r_3");
  components2.push_back(r3);

  decoupleNode(systemDecoupled, "n2", components1, components2, delay,
               cosimMethod, irLine_0_1.real(), i_intf_0.real());
  doSim(simNameDecoupled, systemDecoupled, numThreads, timeStep, true);
}
