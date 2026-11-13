// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <DPsim.h>

#include <chrono>
#include <cmath>
#include <iostream>
#include <vector>

using namespace CPS;
using namespace DPsim;

int main(int argc, char *[]) {
  constexpr UInt branchCount = 128;
  constexpr Real voltage = 400.0;
  constexpr Real frequency = 50.0;
  constexpr Real conductance = 0.1;
  constexpr Real susceptance = 0.05;
  constexpr Real timeStep = 20e-6;
  constexpr Real finalTime = 0.1;
  const Bool useSubcomponents = argc > 1;
  const String simName = useSubcomponents ? "DP_Ph1_Shunt_Grid_Legacy"
                                          : "DP_Ph1_Shunt_Grid_Direct";

  std::vector<DP::SimNode::Ptr> nodes;
  std::vector<std::shared_ptr<DP::Ph1::Shunt>> shunts;
  SystemComponentList components;
  nodes.reserve(branchCount);
  shunts.reserve(branchCount);
  components.reserve(2 * branchCount);

  for (UInt index = 0; index < branchCount; ++index) {
    auto node = DP::SimNode::make("bus" + std::to_string(index));
    node->setInitialVoltage(Complex(voltage, 0.0));
    auto source =
        DP::Ph1::VoltageSource::make("source" + std::to_string(index));
    source->setParameters(Complex(voltage, 0.0), frequency);
    source->connect({DP::SimNode::GND, node});

    auto shunt =
        DP::Ph1::Shunt::make("shunt" + std::to_string(index), useSubcomponents);
    shunt->setParameters(conductance, susceptance);
    shunt->connect({node});

    nodes.push_back(node);
    shunts.push_back(shunt);
    components.push_back(source);
    components.push_back(shunt);
  }

  SystemTopology system(frequency, SystemNodeList(nodes.begin(), nodes.end()),
                        components);
  Simulation simulation(simName, Logger::Level::off);
  simulation.setSystem(system);
  simulation.setDomain(Domain::DP);
  simulation.setSolverType(Solver::Type::MNA);
  simulation.setTimeStep(timeStep);
  simulation.setFinalTime(finalTime);
  const auto start = std::chrono::steady_clock::now();
  simulation.run();
  const Real wallTime =
      std::chrono::duration<Real>(std::chrono::steady_clock::now() - start)
          .count();

  Real maxVoltageError = 0.0;
  Real maxCurrentError = 0.0;
  Real currentMagnitudeSum = 0.0;
  for (UInt index = 0; index < branchCount; ++index) {
    const Complex busVoltage = nodes[index]->voltage()(0, 0);
    const Complex shuntCurrent =
        (**shunts[index]->attributeTyped<MatrixComp>("i_intf"))(0, 0);
    if (!Math::isFinite(busVoltage) || !Math::isFinite(shuntCurrent)) {
      std::cerr << "Shunt grid produced a non-finite value at branch " << index
                << '\n';
      return 1;
    }
    maxVoltageError = std::max(maxVoltageError, std::abs(busVoltage - voltage));
    maxCurrentError =
        std::max(maxCurrentError,
                 std::abs(shuntCurrent - Complex(conductance * voltage,
                                                 susceptance * voltage)));
    currentMagnitudeSum += std::abs(shuntCurrent);
  }
  if (maxVoltageError > 1e-6 || maxCurrentError > 1e-4 ||
      !std::isfinite(currentMagnitudeSum)) {
    std::cerr << "Shunt grid regression: max voltage error=" << maxVoltageError
              << ", max current error=" << maxCurrentError << '\n';
    return 1;
  }

  std::cout << "branches=" << branchCount << '\n'
            << "max_bus_voltage_error=" << maxVoltageError << '\n'
            << "max_current_error=" << maxCurrentError << '\n'
            << "sum_abs_shunt_current=" << currentMagnitudeSum << '\n'
            << "simulation_wall_s=" << wallTime << '\n';
  return 0;
}
