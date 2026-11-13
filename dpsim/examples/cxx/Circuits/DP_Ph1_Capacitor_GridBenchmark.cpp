// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <DPsim.h>

#include <chrono>
#include <cmath>
#include <iostream>
#include <vector>

using namespace CPS;
using namespace DPsim;

int main(int argc, char *argv[]) {
  constexpr UInt branchCount = 128;
  constexpr Real voltage = 230.0;
  constexpr Real frequency = 50.0;
  constexpr Real capacitance = 100e-6;
  constexpr Real resistance = 100.0;
  constexpr Real timeStep = 20e-6;
  constexpr Real finalTime = 0.1;
  const String runName = argc > 1 ? argv[1] : "run";
  const String simName = "DP_Ph1_Capacitor_Grid_" + runName;

  std::vector<DP::SimNode::Ptr> nodes;
  std::vector<std::shared_ptr<DP::Ph1::Capacitor>> capacitors;
  SystemComponentList components;
  nodes.reserve(branchCount);
  capacitors.reserve(branchCount);
  components.reserve(3 * branchCount);

  for (UInt index = 0; index < branchCount; ++index) {
    auto node = DP::SimNode::make("bus" + std::to_string(index));
    node->setInitialVoltage(Complex(voltage, 0.0));

    auto source =
        DP::Ph1::VoltageSource::make("source" + std::to_string(index));
    source->setParameters(Complex(voltage, 0.0), frequency);
    source->connect({DP::SimNode::GND, node});

    auto capacitor =
        DP::Ph1::Capacitor::make("capacitor" + std::to_string(index));
    capacitor->setParameters(capacitance);
    capacitor->connect({node, DP::SimNode::GND});

    auto load = DP::Ph1::Resistor::make("load" + std::to_string(index));
    load->setParameters(resistance);
    load->connect({node, DP::SimNode::GND});

    nodes.push_back(node);
    capacitors.push_back(capacitor);
    components.push_back(source);
    components.push_back(capacitor);
    components.push_back(load);
  }

  SystemTopology system(frequency, SystemNodeList(nodes.begin(), nodes.end()),
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

  for (UInt index = 0; index < branchCount; ++index) {
    const auto nodeVoltage = nodes[index]->voltage()(0, 0);
    const auto capacitorCurrent = capacitors[index]->intfCurrent()(0, 0);
    if (!Math::isFinite(nodeVoltage) || !Math::isFinite(capacitorCurrent)) {
      std::cerr << "DP capacitor grid produced a non-finite result at branch "
                << index << '\n';
      return 1;
    }
  }

  std::cout << "branches=" << branchCount << '\n'
            << "steps=" << static_cast<UInt>(finalTime / timeStep) + 1 << '\n'
            << "simulation_wall_s=" << wallTime << '\n';
  return 0;
}
