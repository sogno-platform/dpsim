/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <DPsim.h>
#include <dpsim-models/CIM/Reader.h>

using namespace std;
using namespace DPsim;
using namespace CPS;
using namespace CPS::CIM;

/*
 * This example runs the powerflow for the CIGRE MV benchmark system (neglecting the tap changers of the transformers)
 */
int main(int argc, char **argv) {

  // Find CIM files
  std::list<fs::path> filenames;

  filenames = DPsim::Utils::findFiles(
      {"20260401__DL_.xml", "20260401__EQ_.xml", "20260401__SV_.xml",
       "20260401__TP_.xml", "20260401__SSH_.xml",
       "20260401__GL_.xml"}, // brauchst du nicht ändern, die Namen passen

      "/home/mms/CIM_files/Working_CIM", "CIMPATH");
  // hier für das erste Argument den Pfad zu den CIM-Dateien angeben, die du einlesen willst --> ändern zu dem Pfad wo du die CIM-Dateien gespeichert hast
  // Das zweite Argument ist der Name der Umgebungsvariable, die den Pfad zu den CIM-Dateien enthält --> nicht ändern

  std::cout << "Found " << filenames.size() << " CIM files." << std::endl;

  String simName = "CIM_Import_Test_PF";
  CPS::Real system_freq = 50;

  Logger::setLogDir("logs/" + simName);
  CIM::Reader reader(simName, Logger::Level::debug, Logger::Level::info);

  // Hier haben wir den neuen Reader der exakt für unsere CIM passt
  reader.setMappingMode(CPS::CIM::MappingMode::CgmesPowerFlow);

  SystemTopology system =
      reader.loadCIM(system_freq, filenames, CPS::Domain::SP,
                     CPS::PhaseType::Single, CPS::GeneratorType::PVNode);

  std::cout << "System topology loaded from CIM files." << std::endl;
  auto logger = DPsim::DataLogger::make(simName);

  // log node voltages
  for (auto node : system.mNodes) {
    logger->logAttribute(node->name() + ".V", node->attribute("v"));
  }

  // log line currents
  for (auto comp : system.mComponents) {
    logger->logAttribute(comp->name() + ".I", comp->attribute("i_intf"));
  }

  Simulation sim(simName, Logger::Level::debug);
  sim.setSystem(system);
  sim.setTimeStep(1);
  sim.setFinalTime(10);
  sim.setDomain(Domain::SP);
  sim.setSolverType(Solver::Type::NRP);
  sim.setSolverAndComponentBehaviour(Solver::Behaviour::Initialization);
  sim.doInitFromNodesAndTerminals(true);
  sim.addLogger(logger);

  sim.run();

  return 0;
}
