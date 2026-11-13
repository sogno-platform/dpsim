// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <DPsim.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <vector>

using namespace CPS;
using namespace DPsim;

int main(int argc, char *argv[]) {
  constexpr UInt branchCount = 128;
  constexpr Real voltage = 20000.0;
  constexpr Real seriesResistance = 0.5;
  constexpr Real seriesInductance = 20e-3;
  constexpr Real parallelCapacitance = 2e-3;
  constexpr Real loadResistance = 20.0;
  constexpr Real timeStep = 10e-6;
  constexpr Real finalTime = 0.5;
  const String variant = argc > 1 ? argv[1] : "run";
  const String simName = "EMT_DC_PiLine_Grid_" + variant;
  const Real initialCurrent = voltage / (seriesResistance + loadResistance);
  const Real receivingVoltage = loadResistance * initialCurrent;

  std::vector<EMT::SimNode::Ptr> sendingNodes;
  std::vector<EMT::SimNode::Ptr> receivingNodes;
  std::vector<std::shared_ptr<EMT::DC::PiLine>> lines;
  SystemComponentList components;
  sendingNodes.reserve(branchCount);
  receivingNodes.reserve(branchCount);
  lines.reserve(branchCount);
  components.reserve(3 * branchCount);

  for (UInt index = 0; index < branchCount; ++index) {
    auto sending =
        EMT::SimNode::make("sending" + std::to_string(index), PhaseType::DC);
    auto receiving =
        EMT::SimNode::make("receiving" + std::to_string(index), PhaseType::DC);
    sending->setInitialVoltage(Complex(voltage, 0.0));
    receiving->setInitialVoltage(Complex(receivingVoltage, 0.0));

    auto source =
        EMT::DC::VoltageSource::make("source" + std::to_string(index));
    source->setParameters(voltage);
    source->connect({EMT::SimNode::GND, sending});

    auto line = EMT::DC::PiLine::make("line" + std::to_string(index), false);
    line->setParameters(seriesResistance, seriesInductance, parallelCapacitance,
                        0.0, initialCurrent);
    line->connect({receiving, sending});

    auto load = EMT::DC::Resistor::make("load" + std::to_string(index));
    load->setParameters(loadResistance);
    load->connect({EMT::SimNode::GND, receiving});

    sendingNodes.push_back(sending);
    receivingNodes.push_back(receiving);
    lines.push_back(line);
    components.push_back(source);
    components.push_back(line);
    components.push_back(load);
  }

  std::vector<EMT::SimNode::Ptr> nodes;
  nodes.reserve(2 * branchCount);
  nodes.insert(nodes.end(), sendingNodes.begin(), sendingNodes.end());
  nodes.insert(nodes.end(), receivingNodes.begin(), receivingNodes.end());
  SystemTopology system(0.0, SystemNodeList(nodes.begin(), nodes.end()),
                        components);

  Logger::setLogDir("logs/" + simName);
  auto logger = DataLogger::make(simName);
  for (UInt index : {0u, branchCount / 2, branchCount - 1}) {
    logger->logAttribute("sending" + std::to_string(index) + ".v",
                         sendingNodes[index]->attribute("v"));
    logger->logAttribute("receiving" + std::to_string(index) + ".v",
                         receivingNodes[index]->attribute("v"));
    logger->logAttribute("line" + std::to_string(index) + ".i",
                         lines[index]->attribute("i_intf"));
  }

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

  Real maxVoltageError = 0.0;
  Real maxCurrentError = 0.0;
  for (UInt index = 0; index < branchCount; ++index) {
    const Real actualSending = sendingNodes[index]->voltage()(0, 0);
    const Real actualReceiving = receivingNodes[index]->voltage()(0, 0);
    const Real actualCurrent = lines[index]->intfCurrent()(0, 0);
    if (!Math::isFinite(actualSending) || !Math::isFinite(actualReceiving) ||
        !Math::isFinite(actualCurrent)) {
      std::cerr << "DC PiLine grid produced a non-finite value at branch "
                << index << '\n';
      return 1;
    }
    maxVoltageError =
        std::max(maxVoltageError, std::abs(actualSending - voltage));
    maxVoltageError =
        std::max(maxVoltageError, std::abs(actualReceiving - receivingVoltage));
    maxCurrentError =
        std::max(maxCurrentError, std::abs(actualCurrent - initialCurrent));
  }
  if (maxVoltageError > 1e-6 || maxCurrentError > 1e-6) {
    std::cerr << "DC PiLine grid regression: voltage error=" << maxVoltageError
              << ", current error=" << maxCurrentError << '\n';
    return 1;
  }

  std::cout << "branches=" << branchCount << '\n'
            << "steps=" << static_cast<UInt>(finalTime / timeStep) + 1 << '\n'
            << "max_voltage_error=" << maxVoltageError << '\n'
            << "max_current_error=" << maxCurrentError << '\n'
            << "simulation_wall_s=" << wallTime << '\n';
  return 0;
}
