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
  constexpr Real voltage = 400.0;
  constexpr Real fundamental = 50.0;
  constexpr Real timeStep = 20e-6;
  constexpr Real finalTime = 0.1;
  const Bool useSubcomponents = argc > 1 && String(argv[1]) == "legacy";
  const String mode = useSubcomponents ? "Legacy" : "Direct";
  const String simName = "DP_Ph1_RXLoad_MultiFrequency_" + mode;

  Matrix frequencies(9, 1);
  frequencies << 50.0, 150.0, 250.0, 350.0, 450.0, 550.0, 650.0, 750.0, 850.0;

  std::vector<DP::SimNode::Ptr> nodes;
  std::vector<std::shared_ptr<DP::Ph1::RXLoad>> loads;
  SystemComponentList components;
  nodes.reserve(branchCount);
  loads.reserve(branchCount);
  components.reserve(2 * branchCount);

  for (UInt index = 0; index < branchCount; ++index) {
    auto node = DP::SimNode::make("bus" + std::to_string(index));
    node->setInitialVoltage(Complex(voltage, 0.0));
    auto source =
        DP::Ph1::VoltageSource::make("source" + std::to_string(index));
    source->setParameters(Complex(voltage, 0.0), fundamental);
    source->connect({DP::SimNode::GND, node});

    auto load =
        DP::Ph1::RXLoad::make("load" + std::to_string(index), useSubcomponents);
    load->setParameters(1000.0 + index, 500.0 + index, voltage);
    load->connect({node});

    nodes.push_back(node);
    loads.push_back(load);
    components.push_back(source);
    components.push_back(load);
  }

  SystemTopology system(fundamental, frequencies,
                        SystemNodeList(nodes.begin(), nodes.end()), components);
  Logger::setLogDir("logs/" + simName);
  auto logger = DataLogger::make(simName);
  logger->logAttribute("bus0_voltage", nodes[0]->attribute("v"), 1, 9);
  logger->logAttribute("load0_current", loads[0]->attribute("i_intf"), 1, 9);

  Simulation simulation(simName, Logger::Level::off);
  simulation.setSystem(system);
  simulation.setDomain(Domain::DP);
  simulation.setSolverType(Solver::Type::MNA);
  simulation.setTimeStep(timeStep);
  simulation.setFinalTime(finalTime);
  simulation.addLogger(logger);
  const auto start = std::chrono::steady_clock::now();
  simulation.run();
  const Real wallTime =
      std::chrono::duration<Real>(std::chrono::steady_clock::now() - start)
          .count();

  Real maxVoltageError = 0.0;
  Real maxFundamentalCurrentError = 0.0;
  Real maxHigherFrequencyCurrent = 0.0;
  for (UInt index = 0; index < branchCount; ++index) {
    const auto &busVoltage = nodes[index]->voltage();
    const auto &loadCurrent =
        **loads[index]->attributeTyped<MatrixComp>("i_intf");
    if (!busVoltage.allFinite() || !loadCurrent.allFinite() ||
        loadCurrent.cols() != frequencies.rows()) {
      std::cerr << "Multi-frequency RXLoad grid produced invalid values at "
                << "branch " << index << " (bus shape " << busVoltage.rows()
                << 'x' << busVoltage.cols() << ", current shape "
                << loadCurrent.rows() << 'x' << loadCurrent.cols()
                << ", finite=" << loadCurrent.allFinite() << ")\n";
      return 1;
    }

    maxVoltageError =
        std::max(maxVoltageError, std::abs(busVoltage(0, 0) - voltage));
    const Complex expectedCurrent((1000.0 + index) / voltage,
                                  -(500.0 + index) / voltage);
    maxFundamentalCurrentError =
        std::max(maxFundamentalCurrentError,
                 std::abs(loadCurrent(0, 0) - expectedCurrent));
    for (UInt freq = 1; freq < static_cast<UInt>(frequencies.rows()); ++freq)
      maxHigherFrequencyCurrent =
          std::max(maxHigherFrequencyCurrent, std::abs(loadCurrent(0, freq)));
  }
  if (maxVoltageError > 1e-6 || maxFundamentalCurrentError > 1e-4 ||
      maxHigherFrequencyCurrent > 1e-8) {
    std::cerr << "Multi-frequency RXLoad grid errors: voltage="
              << maxVoltageError
              << ", fundamental current=" << maxFundamentalCurrentError
              << ", higher-frequency current=" << maxHigherFrequencyCurrent
              << '\n';
    return 1;
  }

  std::cout << "branches=" << branchCount << '\n'
            << "frequencies=" << frequencies.rows() << '\n'
            << "steps=" << static_cast<UInt>(finalTime / timeStep) + 1 << '\n'
            << "max_voltage_error=" << maxVoltageError << '\n'
            << "max_fundamental_current_error=" << maxFundamentalCurrentError
            << '\n'
            << "max_higher_frequency_current=" << maxHigherFrequencyCurrent
            << '\n'
            << "simulation_wall_s=" << wallTime << '\n';
  return 0;
}
