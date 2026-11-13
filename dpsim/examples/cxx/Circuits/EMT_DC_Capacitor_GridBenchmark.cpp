// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <DPsim.h>
#include <dpsim-models/EMT/EMT_DC_Capacitor.h>
#include <dpsim-models/EMT/EMT_DC_CurrentSource.h>
#include <dpsim-models/EMT/EMT_DC_Resistor.h>

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
  const String runTag = argc > 1 ? argv[1] : "run";
  const String simName = "EMT_DC_Capacitor_Grid_" + runTag;

  std::vector<EMT::SimNode::Ptr> nodes;
  std::vector<std::shared_ptr<EMT::DC::Capacitor>> capacitors;
  SystemComponentList components;
  nodes.reserve(branchCount);
  capacitors.reserve(branchCount);
  components.reserve(3 * branchCount);

  for (UInt index = 0; index < branchCount; ++index) {
    auto node =
        EMT::SimNode::make("bus" + std::to_string(index), PhaseType::DC);
    node->setInitialVoltage(Complex(0.0, 0.0));

    auto source =
        EMT::DC::CurrentSource::make("source" + std::to_string(index));
    const Real current = 1.0 + 0.001 * index;
    source->setParameters(current);
    source->connect({node, EMT::SimNode::GND});

    auto resistor = EMT::DC::Resistor::make("resistor" + std::to_string(index));
    const Real resistance = 10.0 + 0.05 * index;
    resistor->setParameters(resistance);
    resistor->connect({EMT::SimNode::GND, node});

    auto capacitor =
        EMT::DC::Capacitor::make("capacitor" + std::to_string(index));
    const Real capacitance = 0.01 + 0.0001 * index;
    capacitor->setParameters(capacitance);
    capacitor->connect({EMT::SimNode::GND, node});

    nodes.push_back(node);
    capacitors.push_back(capacitor);
    components.push_back(source);
    components.push_back(resistor);
    components.push_back(capacitor);
  }

  SystemTopology system(0.0, SystemNodeList(nodes.begin(), nodes.end()),
                        components);
  Logger::setLogDir("logs/" + simName);
  auto logger = DataLogger::make(simName);
  for (UInt index : {0u, branchCount / 2, branchCount - 1}) {
    logger->logAttribute("bus" + std::to_string(index) + ".v",
                         nodes[index]->attribute("v"));
    logger->logAttribute("capacitor" + std::to_string(index) + ".i",
                         capacitors[index]->attribute("i_intf"));
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
  for (UInt index = 0; index < branchCount; ++index) {
    const Real actualVoltage = nodes[index]->voltage()(0, 0);
    const Real resistance = 10.0 + 0.05 * index;
    const Real capacitance = 0.01 + 0.0001 * index;
    const Real current = 1.0 + 0.001 * index;
    const Real expectedVoltage =
        current * resistance *
        (1.0 - std::exp(-finalTime / (resistance * capacitance)));
    if (!Math::isFinite(actualVoltage) ||
        !Math::isFinite(
            (**capacitors[index]->attributeTyped<Matrix>("i_intf"))(0, 0))) {
      std::cerr << "DC capacitor grid produced a non-finite value at branch "
                << index << '\n';
      return 1;
    }
    maxVoltageError =
        std::max(maxVoltageError, std::abs(actualVoltage - expectedVoltage));
  }
  if (maxVoltageError > 2e-4) {
    std::cerr << "DC capacitor grid analytic voltage error=" << maxVoltageError
              << '\n';
    return 1;
  }

  std::cout << "branches=" << branchCount << '\n'
            << "steps=" << static_cast<UInt>(finalTime / timeStep) + 1 << '\n'
            << "max_voltage_error=" << maxVoltageError << '\n'
            << "simulation_wall_s=" << wallTime << '\n';
  return 0;
}
