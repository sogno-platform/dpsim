/* Author: Christoph Wirtz <christoph.wirtz@fgh-ma.de>
 * SPDX-FileCopyrightText: 2026 FGH e.V.
 * SPDX-License-Identifier: MPL-2.0
 */

#include <iostream>
#include <list>
#include <string>
#include <vector>

#include <DPsim.h>

using namespace DPsim;
using namespace CPS::DP;

void EMT_IEEE_39Bus_3ph_Multi(Real timeStep, Real finalTime) {

  // Simulation parameters
  String simName = "IEEE-39bus_multi";
  int numGrids = 4;

  // Find CIM files
  std::list<fs::path> filenames;
  filenames = Utils::findFiles({"Rootnet_FULL_NE_29J10h_SV.xml",
                                "Rootnet_FULL_NE_29J10h_EQ.xml",
                                "Rootnet_FULL_NE_29J10h_TP.xml"},
                               "build/_deps/cim-data-src/IEEE-39", "CIMPATH");

  // ----- DYNAMIC SIMULATION -----
  Logger::setLogDir("logs/" + simName);

  // Load numGrids copies of IEEE-39 and merge them into one system
  SystemTopology sys(60);
  std::vector<CPS::EMT::SimNode::Ptr> couplingNodes;
  std::shared_ptr<CPS::EMT::Ph3::Switch> switch05_g_area0;
  std::shared_ptr<CPS::EMT::Ph3::Switch> switch05_g_area1;

  for (int i = 0; i < numGrids; ++i) {
    CPS::CIM::Reader reader(simName + "_area" + std::to_string(i),
                            Logger::Level::debug, Logger::Level::off);
    SystemTopology area =
        reader.loadCIM(60, filenames, Domain::EMT, PhaseType::ABC,
                       CPS::GeneratorType::IdealVoltageSource);

    couplingNodes.push_back(
        area.component<CPS::EMT::Ph3::PiLine>("L-01-39")->node(0));

    if (i == 0) {
      switch05_g_area0 = CPS::EMT::Ph3::Switch::make("switch-05-ground-area0");
      switch05_g_area0->setParameters(
          CPS::Math::singlePhaseParameterToThreePhase(9999),
          CPS::Math::singlePhaseParameterToThreePhase(0.1), false);
      switch05_g_area0->connect(
          {area.component<CPS::EMT::Ph3::PiLine>("L-05-06")->node(0),
           CPS::EMT::SimNode::GND});
      area.addComponent(switch05_g_area0);
    }

    // Second fault in a different area, fired at the same time, so two subnets
    // recompute concurrently (exercises the parallel diakoptics Schur rebuild).
    if (i == 1) {
      switch05_g_area1 = CPS::EMT::Ph3::Switch::make("switch-05-ground-area1");
      switch05_g_area1->setParameters(
          CPS::Math::singlePhaseParameterToThreePhase(9999),
          CPS::Math::singlePhaseParameterToThreePhase(0.1), false);
      switch05_g_area1->connect(
          {area.component<CPS::EMT::Ph3::PiLine>("L-05-06")->node(0),
           CPS::EMT::SimNode::GND});
      area.addComponent(switch05_g_area1);
    }

    sys.addNodes(area.mNodes);
    sys.addComponents(area.mComponents);
  }

  // Couple the grids in a ring with resistors
  for (int i = 0; i < numGrids; ++i) {
    int j = (i + 1) % numGrids;
    auto coupler = CPS::EMT::Ph3::Resistor::make("couple-" + std::to_string(i) +
                                                 "-" + std::to_string(j));
    coupler->setParameters(CPS::Math::singlePhaseParameterToThreePhase(1));
    coupler->connect({couplingNodes[i], couplingNodes[j]});
    sys.addComponent(coupler);
  }

  // Logging
  auto logger = DataLogger::make(simName);
  for (UInt i = 0; i < couplingNodes.size(); ++i)
    logger->logAttribute("area" + std::to_string(i) + ".V",
                         couplingNodes[i]->attribute("v"));

  auto start = std::chrono::high_resolution_clock::now();

  Simulation sim(simName, Logger::Level::off);
  // Events
  Real startTimeFault = 0.005;
  auto sw1 = SwitchEvent3Ph::make(startTimeFault, switch05_g_area0, true);
  sim.addEvent(sw1);
  auto sw2 = SwitchEvent3Ph::make(startTimeFault, switch05_g_area1, true);
  sim.addEvent(sw2);
  sim.setDomain(CPS::Domain::EMT);
  sim.setSystem(sys);
  sim.setTimeStep(timeStep);
  sim.setFinalTime(finalTime);
  sim.doSteadyStateInit(false);
  sim.doSystemMatrixRecomputation(true);
  sim.addLogger(logger);
  sim.run();

  auto end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = end - start;
  std::cout << "IEEE-39 multi: Zeit: " << std::fixed << std::setprecision(2)
            << elapsed.count() << " Sekunden\n";

  sim.logStepTimes(simName + "_step_times");
}

void EMT_IEEE_39Bus_3ph_Multi_openMP(Real timeStep, Real finalTime) {

  // Simulation parameters
  String simName = "IEEE-39bus_multi_openMP";
  int numGrids = 4;

  // Find CIM files
  std::list<fs::path> filenames;
  filenames = Utils::findFiles({"Rootnet_FULL_NE_29J10h_SV.xml",
                                "Rootnet_FULL_NE_29J10h_EQ.xml",
                                "Rootnet_FULL_NE_29J10h_TP.xml"},
                               "build/_deps/cim-data-src/IEEE-39", "CIMPATH");

  // ----- DYNAMIC SIMULATION -----
  Logger::setLogDir("logs/" + simName);

  // Load numGrids copies of IEEE-39 and merge them into one system
  SystemTopology sys(60);
  std::vector<CPS::EMT::SimNode::Ptr> couplingNodes;
  std::shared_ptr<CPS::EMT::Ph3::Switch> switch05_g_area0;
  std::shared_ptr<CPS::EMT::Ph3::Switch> switch05_g_area1;

  for (int i = 0; i < numGrids; ++i) {
    CPS::CIM::Reader reader(simName + "_area" + std::to_string(i),
                            Logger::Level::debug, Logger::Level::off);
    SystemTopology area =
        reader.loadCIM(60, filenames, Domain::EMT, PhaseType::ABC,
                       CPS::GeneratorType::IdealVoltageSource);

    couplingNodes.push_back(
        area.component<CPS::EMT::Ph3::PiLine>("L-01-39")->node(0));

    if (i == 0) {
      switch05_g_area0 = CPS::EMT::Ph3::Switch::make("switch-05-ground-area0");
      switch05_g_area0->setParameters(
          CPS::Math::singlePhaseParameterToThreePhase(9999),
          CPS::Math::singlePhaseParameterToThreePhase(0.1), false);
      switch05_g_area0->connect(
          {area.component<CPS::EMT::Ph3::PiLine>("L-05-06")->node(0),
           CPS::EMT::SimNode::GND});
      area.addComponent(switch05_g_area0);
    }

    // Second fault in a different area, fired at the same time, so two subnets
    // recompute concurrently (exercises the parallel diakoptics Schur rebuild).
    if (i == 1) {
      switch05_g_area1 = CPS::EMT::Ph3::Switch::make("switch-05-ground-area1");
      switch05_g_area1->setParameters(
          CPS::Math::singlePhaseParameterToThreePhase(9999),
          CPS::Math::singlePhaseParameterToThreePhase(0.1), false);
      switch05_g_area1->connect(
          {area.component<CPS::EMT::Ph3::PiLine>("L-05-06")->node(0),
           CPS::EMT::SimNode::GND});
      area.addComponent(switch05_g_area1);
    }

    sys.addNodes(area.mNodes);
    sys.addComponents(area.mComponents);
  }

  // Couple the grids in a ring with resistors
  for (int i = 0; i < numGrids; ++i) {
    int j = (i + 1) % numGrids;
    auto coupler = CPS::EMT::Ph3::Resistor::make("couple-" + std::to_string(i) +
                                                 "-" + std::to_string(j));
    coupler->setParameters(CPS::Math::singlePhaseParameterToThreePhase(1));
    coupler->connect({couplingNodes[i], couplingNodes[j]});
    sys.addComponent(coupler);
  }

  // Logging
  auto logger = DataLogger::make(simName);
  for (UInt i = 0; i < couplingNodes.size(); ++i)
    logger->logAttribute("area" + std::to_string(i) + ".V",
                         couplingNodes[i]->attribute("v"));

  auto start = std::chrono::high_resolution_clock::now();

  Int threads = numGrids;
  Simulation sim(simName, Logger::Level::off);
  // Events
  Real startTimeFault = 0.005;
  auto sw1 = SwitchEvent3Ph::make(startTimeFault, switch05_g_area0, true);
  sim.addEvent(sw1);
  auto sw2 = SwitchEvent3Ph::make(startTimeFault, switch05_g_area1, true);
  sim.addEvent(sw2);
  sim.setDomain(CPS::Domain::EMT);
  sim.setSystem(sys);
  sim.setTimeStep(timeStep);
  sim.setFinalTime(finalTime);
  sim.doSteadyStateInit(false);
  sim.doSystemMatrixRecomputation(true);
  sim.setScheduler(std::make_shared<OpenMPLevelScheduler>(threads));
  sim.addLogger(logger);
  sim.run();

  auto end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = end - start;
  std::cout << "IEEE-39 multi openMP: Zeit: " << std::fixed
            << std::setprecision(2) << elapsed.count() << " Sekunden\n";

  sim.logStepTimes(simName + "_step_times");
}

void EMT_IEEE_39Bus_3ph_Multi_Diakoptics(Real timeStep, Real finalTime) {

  // Simulation parameters
  String simName = "IEEE-39bus_multi_diakoptics";
  int numGrids = 4;

  // Find CIM files
  std::list<fs::path> filenames;
  filenames = Utils::findFiles({"Rootnet_FULL_NE_29J10h_SV.xml",
                                "Rootnet_FULL_NE_29J10h_EQ.xml",
                                "Rootnet_FULL_NE_29J10h_TP.xml"},
                               "build/_deps/cim-data-src/IEEE-39", "CIMPATH");

  // ----- DYNAMIC SIMULATION -----
  Logger::setLogDir("logs/" + simName);

  // Load numGrids copies of IEEE-39 and merge them into one system
  SystemTopology sys(60);
  std::vector<CPS::EMT::SimNode::Ptr> couplingNodes;
  std::shared_ptr<CPS::EMT::Ph3::Switch> switch05_g_area0;
  std::shared_ptr<CPS::EMT::Ph3::Switch> switch05_g_area1;

  for (int i = 0; i < numGrids; ++i) {
    CPS::CIM::Reader reader(simName + "_area" + std::to_string(i),
                            Logger::Level::debug, Logger::Level::off);
    SystemTopology area =
        reader.loadCIM(60, filenames, Domain::EMT, PhaseType::ABC,
                       CPS::GeneratorType::IdealVoltageSource);

    couplingNodes.push_back(
        area.component<CPS::EMT::Ph3::PiLine>("L-01-39")->node(0));

    if (i == 0) {
      switch05_g_area0 = CPS::EMT::Ph3::Switch::make("switch-05-ground-area0");
      switch05_g_area0->setParameters(
          CPS::Math::singlePhaseParameterToThreePhase(9999),
          CPS::Math::singlePhaseParameterToThreePhase(0.1), false);
      switch05_g_area0->connect(
          {area.component<CPS::EMT::Ph3::PiLine>("L-05-06")->node(0),
           CPS::EMT::SimNode::GND});
      area.addComponent(switch05_g_area0);
    }

    // Second fault in a different area, fired at the same time, so two subnets
    // recompute concurrently (exercises the parallel diakoptics Schur rebuild).
    if (i == 1) {
      switch05_g_area1 = CPS::EMT::Ph3::Switch::make("switch-05-ground-area1");
      switch05_g_area1->setParameters(
          CPS::Math::singlePhaseParameterToThreePhase(9999),
          CPS::Math::singlePhaseParameterToThreePhase(0.1), false);
      switch05_g_area1->connect(
          {area.component<CPS::EMT::Ph3::PiLine>("L-05-06")->node(0),
           CPS::EMT::SimNode::GND});
      area.addComponent(switch05_g_area1);
    }

    sys.addNodes(area.mNodes);
    sys.addComponents(area.mComponents);
  }

  // Couple the grids in a ring with tear components to create numGrids subsystems
  for (int i = 0; i < numGrids; ++i) {
    int j = (i + 1) % numGrids;
    auto coupler = CPS::EMT::Ph3::Resistor::make("couple-" + std::to_string(i) +
                                                 "-" + std::to_string(j));
    coupler->setParameters(CPS::Math::singlePhaseParameterToThreePhase(1));
    coupler->connect({couplingNodes[i], couplingNodes[j]});
    sys.addTearComponent(coupler);
  }

  // Logging
  auto logger = DataLogger::make(simName);
  for (UInt i = 0; i < couplingNodes.size(); ++i)
    logger->logAttribute("area" + std::to_string(i) + ".V",
                         couplingNodes[i]->attribute("v"));

  auto start = std::chrono::high_resolution_clock::now();

  Simulation sim(simName, Logger::Level::off);
  // Events
  Real startTimeFault = 0.005;
  auto sw1 = SwitchEvent3Ph::make(startTimeFault, switch05_g_area0, true);
  sim.addEvent(sw1);
  auto sw2 = SwitchEvent3Ph::make(startTimeFault, switch05_g_area1, true);
  sim.addEvent(sw2);
  sim.setDomain(CPS::Domain::EMT);
  sim.setSystem(sys);
  sim.setTearingComponents(sys.mTearComponents);
  sim.setTimeStep(timeStep);
  sim.setFinalTime(finalTime);
  sim.doSteadyStateInit(false);
  sim.doSystemMatrixRecomputation(true);
  sim.addLogger(logger);
  sim.run();

  auto end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = end - start;
  std::cout << "IEEE-39 multi diakoptics: Zeit: " << std::fixed
            << std::setprecision(2) << elapsed.count() << " Sekunden\n";

  sim.logStepTimes(simName + "_step_times");
}

void EMT_IEEE_39Bus_3ph_Multi_Diakoptics_openMP(Real timeStep, Real finalTime) {

  // Simulation parameters
  String simName = "IEEE-39bus_multi_diakoptics_openMP";
  int numGrids = 4;

  // Find CIM files
  std::list<fs::path> filenames;
  filenames = Utils::findFiles({"Rootnet_FULL_NE_29J10h_SV.xml",
                                "Rootnet_FULL_NE_29J10h_EQ.xml",
                                "Rootnet_FULL_NE_29J10h_TP.xml"},
                               "build/_deps/cim-data-src/IEEE-39", "CIMPATH");

  // ----- DYNAMIC SIMULATION -----
  Logger::setLogDir("logs/" + simName);

  // Load numGrids copies of IEEE-39 and merge them into one system
  SystemTopology sys(60);
  std::vector<CPS::EMT::SimNode::Ptr> couplingNodes;
  std::shared_ptr<CPS::EMT::Ph3::Switch> switch05_g_area0;
  std::shared_ptr<CPS::EMT::Ph3::Switch> switch05_g_area1;

  for (int i = 0; i < numGrids; ++i) {
    CPS::CIM::Reader reader(simName + "_area" + std::to_string(i),
                            Logger::Level::debug, Logger::Level::off);
    SystemTopology area =
        reader.loadCIM(60, filenames, Domain::EMT, PhaseType::ABC,
                       CPS::GeneratorType::IdealVoltageSource);

    couplingNodes.push_back(
        area.component<CPS::EMT::Ph3::PiLine>("L-01-39")->node(0));

    if (i == 0) {
      switch05_g_area0 = CPS::EMT::Ph3::Switch::make("switch-05-ground-area0");
      switch05_g_area0->setParameters(
          CPS::Math::singlePhaseParameterToThreePhase(9999),
          CPS::Math::singlePhaseParameterToThreePhase(0.1), false);
      switch05_g_area0->connect(
          {area.component<CPS::EMT::Ph3::PiLine>("L-05-06")->node(0),
           CPS::EMT::SimNode::GND});
      area.addComponent(switch05_g_area0);
    }

    // Second fault in a different area, fired at the same time, so two subnets
    // recompute concurrently (exercises the parallel diakoptics Schur rebuild).
    if (i == 1) {
      switch05_g_area1 = CPS::EMT::Ph3::Switch::make("switch-05-ground-area1");
      switch05_g_area1->setParameters(
          CPS::Math::singlePhaseParameterToThreePhase(9999),
          CPS::Math::singlePhaseParameterToThreePhase(0.1), false);
      switch05_g_area1->connect(
          {area.component<CPS::EMT::Ph3::PiLine>("L-05-06")->node(0),
           CPS::EMT::SimNode::GND});
      area.addComponent(switch05_g_area1);
    }

    sys.addNodes(area.mNodes);
    sys.addComponents(area.mComponents);
  }

  // Couple the grids in a ring with tear components to create numGrids subsystems
  for (int i = 0; i < numGrids; ++i) {
    int j = (i + 1) % numGrids;
    auto coupler = CPS::EMT::Ph3::Resistor::make("couple-" + std::to_string(i) +
                                                 "-" + std::to_string(j));
    coupler->setParameters(CPS::Math::singlePhaseParameterToThreePhase(1));
    coupler->connect({couplingNodes[i], couplingNodes[j]});
    sys.addTearComponent(coupler);
  }

  // Logging
  auto logger = DataLogger::make(simName);
  for (UInt i = 0; i < couplingNodes.size(); ++i)
    logger->logAttribute("area" + std::to_string(i) + ".V",
                         couplingNodes[i]->attribute("v"));

  auto start = std::chrono::high_resolution_clock::now();

  Simulation sim(simName, Logger::Level::off);
  // Events
  Real startTimeFault = 0.005;
  auto sw1 = SwitchEvent3Ph::make(startTimeFault, switch05_g_area0, true);
  sim.addEvent(sw1);
  auto sw2 = SwitchEvent3Ph::make(startTimeFault, switch05_g_area1, true);
  sim.addEvent(sw2);
  Int threads = numGrids;
  if (threads > 0)
    sim.setScheduler(std::make_shared<OpenMPLevelScheduler>(threads));
  sim.setDomain(CPS::Domain::EMT);
  sim.setSystem(sys);
  sim.setTearingComponents(sys.mTearComponents);
  sim.setTimeStep(timeStep);
  sim.setFinalTime(finalTime);
  sim.doSteadyStateInit(false);
  sim.doSystemMatrixRecomputation(true);
  sim.addLogger(logger);
  sim.run();

  auto end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = end - start;
  std::cout << "IEEE-39 multi diakoptics_openMP: Zeit: " << std::fixed
            << std::setprecision(2) << elapsed.count() << " Sekunden\n";

  sim.logStepTimes(simName + "_step_times");
}

int main(int argc, char *argv[]) {

  Real timeStep = 10e-6;
  Real finalTime = 0.02;

  EMT_IEEE_39Bus_3ph_Multi(timeStep, finalTime);
  EMT_IEEE_39Bus_3ph_Multi_openMP(timeStep, finalTime);
  EMT_IEEE_39Bus_3ph_Multi_Diakoptics(timeStep, finalTime);
  EMT_IEEE_39Bus_3ph_Multi_Diakoptics_openMP(timeStep, finalTime);
  return 0;
}
