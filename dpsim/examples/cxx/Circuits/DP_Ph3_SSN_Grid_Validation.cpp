// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <DPsim.h>
#include <dpsim-models/DP/DP_Ph3_SSN_ImpedanceLoad.h>
#include <dpsim-models/DP/DP_Ph3_SSN_PiLine.h>
#include <dpsim-models/DP/DP_Ph3_SSN_Transformer.h>

using namespace CPS;
using namespace DPsim;

int main() {
  constexpr Real frequency = 50.0;
  constexpr Real timeStep = 100e-6;
  constexpr Real finalTime = 0.08;
  constexpr Real primaryVoltage = 400.0;
  constexpr Real secondaryVoltage = 230.0;
  constexpr Real loadActivePower = 30e3;
  constexpr Real loadReactivePower = 5e3;
  const String simName = "DP_Ph3_SSN_Grid_Validation";

  auto sourceBus = DP::SimNode::make("source_bus", PhaseType::ABC);
  auto transformerBus = DP::SimNode::make("transformer_bus", PhaseType::ABC);
  auto loadBus = DP::SimNode::make("load_bus", PhaseType::ABC);

  auto source = DP::Ph3::VoltageSource::make("grid_source");
  source->setParameters(Math::singlePhaseVariableToThreePhase(
      Complex(primaryVoltage / std::sqrt(3.0), 0.0)));
  source->connect({DP::SimNode::GND, sourceBus});

  auto transformer = DP::Ph3::SSN::Transformer::make("ssn_transformer");
  transformer->setParameters(
      primaryVoltage, secondaryVoltage, 50e3, primaryVoltage / secondaryVoltage,
      0.0, Matrix::Identity(3, 3) * 0.02, Matrix::Identity(3, 3) * 0.0004);
  transformer->setMagnetizingBranch(0.01, 0.001);
  transformer->setReferenceWinding(WindingReference::Primary);
  transformer->connect({sourceBus, transformerBus});

  auto line = DP::Ph3::SSN::PiLine::make("ssn_line");
  line->setParameters(
      Matrix::Identity(3, 3) * 0.05, Matrix::Identity(3, 3) * 0.0002,
      Matrix::Identity(3, 3) * 4e-6, Matrix::Identity(3, 3) * 1e-7);
  line->connect({transformerBus, loadBus});

  auto load = DP::Ph3::SSN::ImpedanceLoad::make("ssn_load");
  load->setParameters(loadActivePower, loadReactivePower, secondaryVoltage);
  load->connect({loadBus});

  auto system = SystemTopology(
      frequency, SystemNodeList{sourceBus, transformerBus, loadBus},
      SystemComponentList{source, transformer, line, load});

  Logger::setLogDir("logs/" + simName);
  auto logger = DataLogger::make(simName);
  logger->logAttribute("source_bus_v", sourceBus->attribute("v"));
  logger->logAttribute("transformer_bus_v", transformerBus->attribute("v"));
  logger->logAttribute("load_bus_v", loadBus->attribute("v"));
  logger->logAttribute("load_i", load->attribute("i_intf"));

  Simulation simulation(simName, Logger::Level::off);
  simulation.setSystem(system);
  simulation.setDomain(Domain::DP);
  simulation.setTimeStep(timeStep);
  simulation.setFinalTime(finalTime);
  simulation.addLogger(logger);
  simulation.run();
  return 0;
}
