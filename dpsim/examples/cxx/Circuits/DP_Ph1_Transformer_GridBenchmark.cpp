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
  constexpr UInt branchCount = 64;
  constexpr Real frequency = 50.0;
  constexpr Real voltage = 230.0;
  constexpr Real timeStep = 20e-6;
  constexpr Real finalTime = 0.1;
  const String variant = argc > 1 ? argv[1] : "run";
  const String simName = "DP_Ph1_Transformer_Grid_" + variant;

  std::vector<SimNode<Complex>::Ptr> primaryNodes;
  std::vector<SimNode<Complex>::Ptr> secondaryNodes;
  std::vector<std::shared_ptr<DP::Ph1::Transformer>> transformers;
  SystemComponentList components;
  primaryNodes.reserve(branchCount);
  secondaryNodes.reserve(branchCount);
  transformers.reserve(branchCount);
  components.reserve(3 * branchCount);

  for (UInt index = 0; index < branchCount; ++index) {
    auto primary = SimNode<Complex>::make("primary" + std::to_string(index),
                                          PhaseType::Single);
    auto secondary = SimNode<Complex>::make("secondary" + std::to_string(index),
                                            PhaseType::Single);
    primary->setInitialVoltage(Complex(voltage, 0.0));
    secondary->setInitialVoltage(Complex(voltage, 0.0));

    auto source =
        DP::Ph1::NetworkInjection::make("source" + std::to_string(index));
    source->setParameters(Complex(voltage, 0.0), frequency);
    source->connect({primary});

    auto transformer = DP::Ph1::Transformer::make(
        "transformer" + std::to_string(index), false, Logger::Level::off, true);
    transformer->setReferenceWinding(WindingReference::Primary);
    transformer->setParameters(230.0, 230.0, 10000.0, 1.0, 0.0, 0.1, 1e-3);
    transformer->connect({primary, secondary});

    auto load = DP::Ph1::Resistor::make("load" + std::to_string(index));
    load->setParameters(100.0);
    load->connect({secondary, SimNode<Complex>::GND});

    primaryNodes.push_back(primary);
    secondaryNodes.push_back(secondary);
    transformers.push_back(transformer);
    components.push_back(source);
    components.push_back(transformer);
    components.push_back(load);
  }

  std::vector<SimNode<Complex>::Ptr> nodes;
  nodes.reserve(2 * branchCount);
  nodes.insert(nodes.end(), primaryNodes.begin(), primaryNodes.end());
  nodes.insert(nodes.end(), secondaryNodes.begin(), secondaryNodes.end());
  SystemTopology system(frequency, SystemNodeList(nodes.begin(), nodes.end()),
                        components);

  Logger::setLogDir("logs/" + simName);
  auto logger = DataLogger::make(simName);
  for (UInt index : {0u, branchCount / 2, branchCount - 1}) {
    logger->logAttribute("primary" + std::to_string(index) + ".v",
                         primaryNodes[index]->attribute("v"));
    logger->logAttribute("secondary" + std::to_string(index) + ".v",
                         secondaryNodes[index]->attribute("v"));
    logger->logAttribute("transformer" + std::to_string(index) + ".v",
                         transformers[index]->attribute("v_intf"));
    logger->logAttribute("transformer" + std::to_string(index) + ".i",
                         transformers[index]->attribute("i_intf"));
  }

  Simulation simulation(simName, Logger::Level::off);
  simulation.setSystem(system);
  simulation.setDomain(Domain::DP);
  simulation.setTimeStep(timeStep);
  simulation.setFinalTime(finalTime);
  simulation.addLogger(logger);

  const auto start = std::chrono::steady_clock::now();
  simulation.run();
  const Real wallTime =
      std::chrono::duration<Real>(std::chrono::steady_clock::now() - start)
          .count();

  for (UInt index = 0; index < branchCount; ++index) {
    const auto primaryVoltage = primaryNodes[index]->voltage()(0, 0);
    const auto secondaryVoltage = secondaryNodes[index]->voltage()(0, 0);
    const auto transformerVoltage = transformers[index]->intfVoltage()(0, 0);
    const auto transformerCurrent = transformers[index]->intfCurrent()(0, 0);
    if (!Math::isFinite(primaryVoltage) || !Math::isFinite(secondaryVoltage) ||
        !Math::isFinite(transformerVoltage) ||
        !Math::isFinite(transformerCurrent)) {
      std::cerr << "DP transformer grid produced a non-finite result at branch "
                << index << '\n';
      return 1;
    }
  }

  std::cout << "branches=" << branchCount << '\n'
            << "steps=" << static_cast<UInt>(finalTime / timeStep) + 1 << '\n'
            << "simulation_wall_s=" << wallTime << '\n';
  return 0;
}
