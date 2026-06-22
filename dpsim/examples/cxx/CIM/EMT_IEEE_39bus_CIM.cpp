/* Author: Christoph Wirtz <christoph.wirtz@fgh-ma.de>
 * SPDX-FileCopyrightText: 2026 FGH e.V.
 * SPDX-License-Identifier: MPL-2.0
 */

#include <iostream>
#include <list>

#include <DPsim.h>

using namespace DPsim;
using namespace CPS::DP;

void EMT_IEEE_39Bus_3ph(Real timeStep, Real finalTime) {

  // Simulation parameters
  String simName = "IEEE-39bus";

  // Find CIM files
  std::list<fs::path> filenames;
  filenames = Utils::findFiles({"Rootnet_FULL_NE_29J10h_SV.xml",
                                "Rootnet_FULL_NE_29J10h_EQ.xml",
                                "Rootnet_FULL_NE_29J10h_TP.xml"},
                               "build/_deps/cim-data-src/IEEE-39", "CIMPATH");

  // ----- DYNAMIC SIMULATION -----
  Logger::setLogDir("logs/" + simName);

  CPS::CIM::Reader reader2(simName, Logger::Level::debug, Logger::Level::off);
  SystemTopology sys =
      reader2.loadCIM(60, filenames, Domain::EMT, PhaseType::ABC,
                      CPS::GeneratorType::IdealVoltageSource);

  // Replace lines with resistors for comparability:
  auto r16_17 = CPS::EMT::Ph3::Resistor::make("resistor-16-17");
  r16_17->setParameters(CPS::Math::singlePhaseParameterToThreePhase(1));
  auto comp16_17 = sys.component<CPS::EMT::Ph3::PiLine>("L-16-17");
  r16_17->connect({comp16_17->node(0), comp16_17->node(1)});

  auto r15_16 = CPS::EMT::Ph3::Resistor::make("resistor-15-16");
  r15_16->setParameters(CPS::Math::singlePhaseParameterToThreePhase(1));
  auto comp15_16 = sys.component<CPS::EMT::Ph3::PiLine>("L-15-16");
  r15_16->connect({comp15_16->node(0), comp15_16->node(1)});

  auto r01_39 = CPS::EMT::Ph3::Resistor::make("resistor-01-39");
  r01_39->setParameters(CPS::Math::singlePhaseParameterToThreePhase(1));
  auto comp01_39 = sys.component<CPS::EMT::Ph3::PiLine>("L-01-39");
  r01_39->connect({comp01_39->node(0), comp01_39->node(1)});

  auto r03_04 = CPS::EMT::Ph3::Resistor::make("resistor-03-04");
  r03_04->setParameters(CPS::Math::singlePhaseParameterToThreePhase(1));
  auto comp03_04 = sys.component<CPS::EMT::Ph3::PiLine>("L-03-04");
  r03_04->connect({comp03_04->node(0), comp03_04->node(1)});

  auto switch05_g = CPS::EMT::Ph3::Switch::make("switch-05-ground");
  switch05_g->setParameters(CPS::Math::singlePhaseParameterToThreePhase(9999),
                            CPS::Math::singlePhaseParameterToThreePhase(0.1),
                            false);
  auto comp05_06 = sys.component<CPS::EMT::Ph3::PiLine>("L-05-06");
  switch05_g->connect({comp05_06->node(0), CPS::EMT::SimNode::GND});

  sys.removeComponent("L-16-17");
  sys.removeComponent("L-15-16");
  sys.removeComponent("L-01-39");
  sys.removeComponent("L-03-04");
  sys.addComponent(r16_17);
  sys.addComponent(r15_16);
  sys.addComponent(r01_39);
  sys.addComponent(r03_04);
  sys.addComponent(switch05_g);

  // Logging
  auto logger = DataLogger::make(simName);
  for (auto node : sys.mNodes) {
    logger->logAttribute(node->name() + ".V", node->attribute("v"));
  }

  auto start = std::chrono::high_resolution_clock::now();

  Simulation sim(simName, Logger::Level::off);
  // Events
  Real startTimeFault = 0.1;
  auto sw1 = SwitchEvent3Ph::make(startTimeFault, switch05_g, true);
  sim.addEvent(sw1);
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
  std::cout << "IEEE-39: Zeit: " << std::fixed << std::setprecision(2)
            << elapsed.count() << " Sekunden\n";
}

void EMT_IEEE_39Bus_3ph_openMP(Real timeStep, Real finalTime) {

  // Simulation parameters
  String simName = "IEEE-39bus_openMP";

  // Find CIM files
  std::list<fs::path> filenames;

  filenames = Utils::findFiles({"Rootnet_FULL_NE_29J10h_SV.xml",
                                "Rootnet_FULL_NE_29J10h_EQ.xml",
                                "Rootnet_FULL_NE_29J10h_TP.xml"},
                               "build/_deps/cim-data-src/IEEE-39", "CIMPATH");

  // ----- DYNAMIC SIMULATION -----
  Logger::setLogDir("logs/" + simName);

  CPS::CIM::Reader reader2(simName, Logger::Level::debug, Logger::Level::off);
  SystemTopology sys =
      reader2.loadCIM(60, filenames, Domain::EMT, PhaseType::ABC,
                      CPS::GeneratorType::IdealVoltageSource);

  // Replace lines with resistors for comparability:
  auto r16_17 = CPS::EMT::Ph3::Resistor::make("resistor-16-17");
  r16_17->setParameters(CPS::Math::singlePhaseParameterToThreePhase(1));
  auto comp16_17 = sys.component<CPS::EMT::Ph3::PiLine>("L-16-17");
  r16_17->connect({comp16_17->node(0), comp16_17->node(1)});

  auto r15_16 = CPS::EMT::Ph3::Resistor::make("resistor-15-16");
  r15_16->setParameters(CPS::Math::singlePhaseParameterToThreePhase(1));
  auto comp15_16 = sys.component<CPS::EMT::Ph3::PiLine>("L-15-16");
  r15_16->connect({comp15_16->node(0), comp15_16->node(1)});

  auto r01_39 = CPS::EMT::Ph3::Resistor::make("resistor-01-39");
  r01_39->setParameters(CPS::Math::singlePhaseParameterToThreePhase(1));
  auto comp01_39 = sys.component<CPS::EMT::Ph3::PiLine>("L-01-39");
  r01_39->connect({comp01_39->node(0), comp01_39->node(1)});

  auto r03_04 = CPS::EMT::Ph3::Resistor::make("resistor-03-04");
  r03_04->setParameters(CPS::Math::singlePhaseParameterToThreePhase(1));
  auto comp03_04 = sys.component<CPS::EMT::Ph3::PiLine>("L-03-04");
  r03_04->connect({comp03_04->node(0), comp03_04->node(1)});

  auto switch05_g = CPS::EMT::Ph3::Switch::make("switch-05-ground");
  switch05_g->setParameters(CPS::Math::singlePhaseParameterToThreePhase(9999),
                            CPS::Math::singlePhaseParameterToThreePhase(0.1),
                            false);
  auto comp05_06 = sys.component<CPS::EMT::Ph3::PiLine>("L-05-06");
  switch05_g->connect({comp05_06->node(0), CPS::EMT::SimNode::GND});

  sys.removeComponent("L-16-17");
  sys.removeComponent("L-15-16");
  sys.removeComponent("L-01-39");
  sys.removeComponent("L-03-04");
  sys.addComponent(r16_17);
  sys.addComponent(r15_16);
  sys.addComponent(r01_39);
  sys.addComponent(r03_04);
  sys.addComponent(switch05_g);

  // Logging
  auto logger = DataLogger::make(simName);
  for (auto node : sys.mNodes) {
    logger->logAttribute(node->name() + ".V", node->attribute("v"));
  }

  auto start = std::chrono::high_resolution_clock::now();

  Int threads = 2;
  Simulation sim(simName, Logger::Level::off);
  // Events
  Real startTimeFault = 0.1;
  auto sw1 = SwitchEvent3Ph::make(startTimeFault, switch05_g, true);
  sim.addEvent(sw1);
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
  std::cout << "IEEE-39 openMP: Zeit: " << std::fixed << std::setprecision(2)
            << elapsed.count() << " Sekunden\n";
}

void EMT_IEEE_39Bus_3ph_Diakoptics(Real timeStep, Real finalTime) {
  // Simulation parameters
  String simName = "IEEE-39bus_diakoptics";

  // Find CIM files
  std::list<fs::path> filenames;
  filenames = Utils::findFiles({"Rootnet_FULL_NE_29J10h_SV.xml",
                                "Rootnet_FULL_NE_29J10h_EQ.xml",
                                "Rootnet_FULL_NE_29J10h_TP.xml"},
                               "build/_deps/cim-data-src/IEEE-39", "CIMPATH");

  // ----- DYNAMIC SIMULATION -----
  Logger::setLogDir("logs/" + simName);

  CPS::CIM::Reader reader2(simName, Logger::Level::debug, Logger::Level::off);
  SystemTopology sys =
      reader2.loadCIM(60, filenames, Domain::EMT, PhaseType::ABC,
                      CPS::GeneratorType::IdealVoltageSource);

  // Replace lines with tear component to create 2 subsystems
  auto r16_17 = CPS::EMT::Ph3::Resistor::make("resistor-16-17");
  r16_17->setParameters(CPS::Math::singlePhaseParameterToThreePhase(1));
  auto comp16_17 = sys.component<CPS::EMT::Ph3::PiLine>("L-16-17");
  r16_17->connect({comp16_17->node(0), comp16_17->node(1)});

  auto r15_16 = CPS::EMT::Ph3::Resistor::make("resistor-15-16");
  r15_16->setParameters(CPS::Math::singlePhaseParameterToThreePhase(1));
  auto comp15_16 = sys.component<CPS::EMT::Ph3::PiLine>("L-15-16");
  r15_16->connect({comp15_16->node(0), comp15_16->node(1)});

  auto r01_39 = CPS::EMT::Ph3::Resistor::make("resistor-01-39");
  r01_39->setParameters(CPS::Math::singlePhaseParameterToThreePhase(1));
  auto comp01_39 = sys.component<CPS::EMT::Ph3::PiLine>("L-01-39");
  r01_39->connect({comp01_39->node(0), comp01_39->node(1)});

  auto r03_04 = CPS::EMT::Ph3::Resistor::make("resistor-03-04");
  r03_04->setParameters(CPS::Math::singlePhaseParameterToThreePhase(1));
  auto comp03_04 = sys.component<CPS::EMT::Ph3::PiLine>("L-03-04");
  r03_04->connect({comp03_04->node(0), comp03_04->node(1)});

  auto switch05_g = CPS::EMT::Ph3::Switch::make("switch-05-ground");
  switch05_g->setParameters(CPS::Math::singlePhaseParameterToThreePhase(9999),
                            CPS::Math::singlePhaseParameterToThreePhase(0.1),
                            false);
  auto comp05_06 = sys.component<CPS::EMT::Ph3::PiLine>("L-05-06");
  switch05_g->connect({comp05_06->node(0), CPS::EMT::SimNode::GND});

  sys.removeComponent("L-16-17");
  sys.removeComponent("L-15-16");
  sys.removeComponent("L-01-39");
  sys.removeComponent("L-03-04");
  sys.addTearComponent(r16_17);
  sys.addTearComponent(r15_16);
  sys.addTearComponent(r01_39);
  sys.addTearComponent(r03_04);
  sys.addComponent(switch05_g);

  // Logging
  auto logger = DataLogger::make(simName);
  for (auto node : sys.mNodes) {
    logger->logAttribute(node->name() + ".V", node->attribute("v"));
  }

  auto start = std::chrono::high_resolution_clock::now();

  Simulation sim(simName, Logger::Level::off);
  // Events
  Real startTimeFault = 0.1;
  auto sw1 = SwitchEvent3Ph::make(startTimeFault, switch05_g, true);
  sim.addEvent(sw1);
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
  std::cout << "IEEE-39 diakoptics: Zeit: " << std::fixed
            << std::setprecision(2) << elapsed.count() << " Sekunden\n";
}

void EMT_IEEE_39Bus_3ph_Diakoptics_openMP(Real timeStep, Real finalTime) {
  // Simulation parameters
  String simName = "IEEE-39bus_diakoptics_openMP";

  // Find CIM files
  std::list<fs::path> filenames;
  filenames = Utils::findFiles({"Rootnet_FULL_NE_29J10h_SV.xml",
                                "Rootnet_FULL_NE_29J10h_EQ.xml",
                                "Rootnet_FULL_NE_29J10h_TP.xml"},
                               "build/_deps/cim-data-src/IEEE-39", "CIMPATH");

  // ----- DYNAMIC SIMULATION -----
  Logger::setLogDir("logs/" + simName);

  CPS::CIM::Reader reader2(simName, Logger::Level::debug, Logger::Level::off);
  SystemTopology sys =
      reader2.loadCIM(60, filenames, Domain::EMT, PhaseType::ABC,
                      CPS::GeneratorType::IdealVoltageSource);

  // Replace lines with tear component to create 2 subsystems
  auto r16_17 = CPS::EMT::Ph3::Resistor::make("resistor-16-17");
  r16_17->setParameters(CPS::Math::singlePhaseParameterToThreePhase(1));
  auto comp16_17 = sys.component<CPS::EMT::Ph3::PiLine>("L-16-17");
  r16_17->connect({comp16_17->node(0), comp16_17->node(1)});

  auto r15_16 = CPS::EMT::Ph3::Resistor::make("resistor-15-16");
  r15_16->setParameters(CPS::Math::singlePhaseParameterToThreePhase(1));
  auto comp15_16 = sys.component<CPS::EMT::Ph3::PiLine>("L-15-16");
  r15_16->connect({comp15_16->node(0), comp15_16->node(1)});

  auto r01_39 = CPS::EMT::Ph3::Resistor::make("resistor-01-39");
  r01_39->setParameters(CPS::Math::singlePhaseParameterToThreePhase(1));
  auto comp01_39 = sys.component<CPS::EMT::Ph3::PiLine>("L-01-39");
  r01_39->connect({comp01_39->node(0), comp01_39->node(1)});

  auto r03_04 = CPS::EMT::Ph3::Resistor::make("resistor-03-04");
  r03_04->setParameters(CPS::Math::singlePhaseParameterToThreePhase(1));
  auto comp03_04 = sys.component<CPS::EMT::Ph3::PiLine>("L-03-04");
  r03_04->connect({comp03_04->node(0), comp03_04->node(1)});

  auto switch05_g = CPS::EMT::Ph3::Switch::make("switch-05-ground");
  switch05_g->setParameters(CPS::Math::singlePhaseParameterToThreePhase(9999),
                            CPS::Math::singlePhaseParameterToThreePhase(0.1),
                            false);
  auto comp05_06 = sys.component<CPS::EMT::Ph3::PiLine>("L-05-06");
  switch05_g->connect({comp05_06->node(0), CPS::EMT::SimNode::GND});

  sys.removeComponent("L-16-17");
  sys.removeComponent("L-15-16");
  sys.removeComponent("L-01-39");
  sys.removeComponent("L-03-04");
  sys.addTearComponent(r16_17);
  sys.addTearComponent(r15_16);
  sys.addTearComponent(r01_39);
  sys.addTearComponent(r03_04);
  sys.addComponent(switch05_g);

  // Logging
  auto logger = DataLogger::make(simName);
  for (auto node : sys.mNodes) {
    logger->logAttribute(node->name() + ".V", node->attribute("v"));
  }

  auto start = std::chrono::high_resolution_clock::now();

  Simulation sim(simName, Logger::Level::off);
  // Events
  Real startTimeFault = 0.1;
  auto sw1 = SwitchEvent3Ph::make(startTimeFault, switch05_g, true);
  sim.addEvent(sw1);
  Int threads = 2;
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
  std::cout << "IEEE-39 diakoptics_openMP: Zeit: " << std::fixed
            << std::setprecision(2) << elapsed.count() << " Sekunden\n";
}

int main(int argc, char *argv[]) {

  Real timeStep = 10e-6;
  Real finalTime = 1.0;

  EMT_IEEE_39Bus_3ph(timeStep, finalTime);
  EMT_IEEE_39Bus_3ph_openMP(timeStep, finalTime);
  EMT_IEEE_39Bus_3ph_Diakoptics(timeStep, finalTime);
  EMT_IEEE_39Bus_3ph_Diakoptics_openMP(timeStep, finalTime);
  return 0;
}
