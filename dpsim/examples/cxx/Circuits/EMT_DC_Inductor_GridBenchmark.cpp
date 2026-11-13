// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <DPsim.h>
#include <dpsim-models/EMT/EMT_DC_CurrentSource.h>
#include <dpsim-models/EMT/EMT_DC_Inductor.h>
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
  const String simName = "EMT_DC_Inductor_Grid_" + runTag;

  std::vector<EMT::SimNode::Ptr> nodes;
  std::vector<std::shared_ptr<EMT::DC::Inductor>> inductors;
  SystemComponentList components;
  nodes.reserve(branchCount);
  inductors.reserve(branchCount);
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

    auto inductor = EMT::DC::Inductor::make("inductor" + std::to_string(index));
    const Real inductance = 0.2 + 0.001 * index;
    inductor->setParameters(inductance, 0.0);
    inductor->connect({EMT::SimNode::GND, node});

    nodes.push_back(node);
    inductors.push_back(inductor);
    components.push_back(source);
    components.push_back(resistor);
    components.push_back(inductor);
  }

  SystemTopology system(0.0, SystemNodeList(nodes.begin(), nodes.end()),
                        components);
  Logger::setLogDir("logs/" + simName);
  auto logger = DataLogger::make(simName);
  for (UInt index : {0u, branchCount / 2, branchCount - 1}) {
    logger->logAttribute("bus" + std::to_string(index) + ".v",
                         nodes[index]->attribute("v"));
    logger->logAttribute("inductor" + std::to_string(index) + ".i",
                         inductors[index]->attribute("i_intf"));
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
    const Real actualVoltage = nodes[index]->voltage()(0, 0);
    const Real actualCurrent =
        (**inductors[index]->attributeTyped<Matrix>("i_intf"))(0, 0);
    const Real resistance = 10.0 + 0.05 * index;
    const Real inductance = 0.2 + 0.001 * index;
    const Real sourceCurrent = 1.0 + 0.001 * index;
    const Real exponential = std::exp(-resistance * finalTime / inductance);
    const Real expectedVoltage = sourceCurrent * resistance * exponential;
    const Real expectedCurrent = sourceCurrent * (1.0 - exponential);
    if (!Math::isFinite(actualVoltage) || !Math::isFinite(actualCurrent)) {
      std::cerr << "DC inductor grid produced a non-finite value at branch "
                << index << '\n';
      return 1;
    }
    maxVoltageError =
        std::max(maxVoltageError, std::abs(actualVoltage - expectedVoltage));
    maxCurrentError =
        std::max(maxCurrentError, std::abs(actualCurrent - expectedCurrent));
  }
  if (maxVoltageError > 1e-5 || maxCurrentError > 1e-6) {
    std::cerr << "DC inductor grid analytic errors: voltage=" << maxVoltageError
              << ", current=" << maxCurrentError << '\n';
    return 1;
  }

  std::cout << "branches=" << branchCount << '\n'
            << "steps=" << static_cast<UInt>(finalTime / timeStep) + 1 << '\n'
            << "max_voltage_error=" << maxVoltageError << '\n'
            << "max_current_error=" << maxCurrentError << '\n'
            << "simulation_wall_s=" << wallTime << '\n';
  return 0;
}
