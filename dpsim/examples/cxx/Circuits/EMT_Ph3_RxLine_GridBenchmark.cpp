// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <DPsim.h>
#include <dpsim-models/EMT/EMT_Ph3_Resistor.h>
#include <dpsim-models/EMT/EMT_Ph3_RxLine.h>
#include <dpsim-models/EMT/EMT_Ph3_VoltageSource.h>

#include <chrono>
#include <iostream>
#include <vector>

using namespace CPS;
using namespace DPsim;

int main(int argc, char *argv[]) {
  constexpr UInt lineCount = 64;
  constexpr Real timeStep = 50e-6;
  constexpr Real finalTime = 0.5;
  const String runTag = argc > 1 ? argv[1] : "run";
  const String simName = "EMT_Ph3_RxLine_Grid_" + runTag;
  const std::vector<Complex> initialVoltage{
      Complex(100e3, 0), Complex(100e3, 0), Complex(100e3, 0)};
  auto sourceNode =
      EMT::SimNode::make("source", PhaseType::ABC, initialVoltage);
  auto loadNode = EMT::SimNode::make("load", PhaseType::ABC, initialVoltage);

  auto source = EMT::Ph3::VoltageSource::make("source_voltage");
  source->setParameters(Math::singlePhaseVariableToThreePhase(
                            Complex(100e3 * PEAK1PH_TO_RMS3PH, 0)),
                        50.0);
  source->connect({EMT::SimNode::GND, sourceNode});

  std::vector<std::shared_ptr<EMT::Ph3::RxLine>> lines;
  lines.reserve(lineCount);
  SystemComponentList components;
  components.reserve(lineCount + 2);
  components.push_back(source);
  for (UInt index = 0; index < lineCount; ++index) {
    auto line = EMT::Ph3::RxLine::make("line" + std::to_string(index), false);
    line->setParameters(Matrix::Identity(3, 3) * 5.0,
                        Matrix::Identity(3, 3) * 0.16);
    line->connect({sourceNode, loadNode});
    lines.push_back(line);
    components.push_back(line);
  }

  auto load = EMT::Ph3::Resistor::make("load_resistor");
  load->setParameters(Matrix::Identity(3, 3) * 10000.0);
  load->connect({EMT::SimNode::GND, loadNode});
  components.push_back(load);

  SystemTopology system(50.0, {sourceNode, loadNode}, components);
  Logger::setLogDir("logs/" + simName);
  auto logger = DataLogger::make(simName);
  logger->logAttribute("source_voltage", sourceNode->attribute("v"));
  logger->logAttribute("load_voltage", loadNode->attribute("v"));
  logger->logAttribute("line0_current", lines.front()->attribute("i_intf"));
  logger->logAttribute("line_last_current", lines.back()->attribute("i_intf"));
  logger->logAttribute("load_current", load->attribute("i_intf"));

  Simulation simulation(simName, Logger::Level::off);
  simulation.setSystem(system);
  simulation.setDomain(Domain::EMT);
  simulation.setTimeStep(timeStep);
  simulation.setFinalTime(finalTime);
  simulation.addLogger(logger);
  const auto start = std::chrono::steady_clock::now();
  simulation.run();
  const Real wallTime =
      std::chrono::duration<Real>(std::chrono::steady_clock::now() - start)
          .count();

  for (UInt index = 0; index < lineCount; ++index) {
    const Matrix &current = **lines[index]->attributeTyped<Matrix>("i_intf");
    if (!current.allFinite()) {
      std::cerr << "RxLine grid produced a non-finite result at line " << index
                << '\n';
      return 1;
    }
  }
  if (!sourceNode->voltage().allFinite() || !loadNode->voltage().allFinite() ||
      !(**load->attributeTyped<Matrix>("i_intf")).allFinite()) {
    std::cerr << "RxLine grid produced a non-finite network result.\n";
    return 1;
  }

  std::cout << "lines=" << lineCount << '\n'
            << "steps=" << static_cast<UInt>(finalTime / timeStep) + 1 << '\n'
            << "simulation_wall_s=" << wallTime << '\n';
  return 0;
}
