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
  constexpr Real timeStep = 10e-6;
  constexpr Real finalTime = 0.5;
  const String variant = argc > 1 ? argv[1] : "run";
  const String simName = "EMT_DC_CurrentSource_Grid_" + variant;

  std::vector<EMT::SimNode::Ptr> nodes;
  std::vector<std::shared_ptr<EMT::DC::CurrentSource>> sources;
  std::vector<std::shared_ptr<EMT::DC::Resistor>> resistors;
  SystemComponentList components;
  nodes.reserve(branchCount);
  sources.reserve(branchCount);
  resistors.reserve(branchCount);
  components.reserve(2 * branchCount);

  for (UInt index = 0; index < branchCount; ++index) {
    const Real resistance = 20.0 + index;
    const Real current = 10.0 + index;
    const Real voltage = -resistance * current;
    auto node =
        EMT::SimNode::make("bus" + std::to_string(index), PhaseType::DC);
    node->setInitialVoltage(Complex(voltage, 0.0));

    auto source =
        EMT::DC::CurrentSource::make("source" + std::to_string(index));
    source->setParameters(current);
    source->connect({EMT::SimNode::GND, node});

    auto resistor = EMT::DC::Resistor::make("resistor" + std::to_string(index));
    resistor->setParameters(resistance);
    resistor->connect({EMT::SimNode::GND, node});

    nodes.push_back(node);
    sources.push_back(source);
    resistors.push_back(resistor);
    components.push_back(source);
    components.push_back(resistor);
  }

  SystemTopology system(0.0, SystemNodeList(nodes.begin(), nodes.end()),
                        components);
  Logger::setLogDir("logs/" + simName);
  auto logger = DataLogger::make(simName);
  for (UInt index : {0u, branchCount / 2, branchCount - 1}) {
    logger->logAttribute("bus" + std::to_string(index) + ".v",
                         nodes[index]->attribute("v"));
    logger->logAttribute("source" + std::to_string(index) + ".v",
                         sources[index]->attribute("v_intf"));
    logger->logAttribute("source" + std::to_string(index) + ".i",
                         sources[index]->attribute("i_intf"));
    logger->logAttribute("resistor" + std::to_string(index) + ".i",
                         resistors[index]->attribute("i_intf"));
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
    const Real expectedResistance = 20.0 + index;
    const Real expectedSourceCurrent = 10.0 + index;
    const Real expectedVoltage = -expectedResistance * expectedSourceCurrent;
    const Real actualVoltage = nodes[index]->voltage()(0, 0);
    const Real actualSourceCurrent = sources[index]->intfCurrent()(0, 0);
    const Real actualSourceVoltage = sources[index]->intfVoltage()(0, 0);
    const Real actualLoadCurrent = resistors[index]->intfCurrent()(0, 0);
    if (!Math::isFinite(actualVoltage) ||
        !Math::isFinite(actualSourceCurrent) ||
        !Math::isFinite(actualSourceVoltage) ||
        !Math::isFinite(actualLoadCurrent)) {
      std::cerr << "DC current-source grid produced a non-finite value at "
                   "branch "
                << index << '\n';
      return 1;
    }
    maxVoltageError =
        std::max(maxVoltageError, std::abs(actualVoltage - expectedVoltage));
    maxCurrentError =
        std::max(maxCurrentError,
                 std::max(std::abs(actualSourceCurrent - expectedSourceCurrent),
                          std::abs(actualLoadCurrent + expectedSourceCurrent)));
  }
  if (maxVoltageError > 1e-8 || maxCurrentError > 1e-8) {
    std::cerr << "DC current-source grid regression: voltage error="
              << maxVoltageError << ", current error=" << maxCurrentError
              << '\n';
    return 1;
  }

  std::cout << "branches=" << branchCount << '\n'
            << "steps=" << static_cast<UInt>(finalTime / timeStep) + 1 << '\n'
            << "max_voltage_error=" << maxVoltageError << '\n'
            << "max_current_error=" << maxCurrentError << '\n'
            << "simulation_wall_s=" << wallTime << '\n';
  return 0;
}
