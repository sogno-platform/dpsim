// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <stdexcept>
#include <vector>

#include <DPsim.h>

using namespace CPS;
using namespace DPsim;

int main() {
  constexpr UInt branchCount = 512;
  constexpr Real resistance = 10.0;
  constexpr Real timeStep = 100e-6;
  constexpr Real finalTime = 1.0;
  const String simName = "DP_Ph1_Resistor_GridBenchmark";

  std::vector<DP::SimNode::Ptr> nodes;
  SystemComponentList components;
  nodes.reserve(branchCount);
  components.reserve(2 * branchCount);

  for (UInt idx = 0; idx < branchCount; ++idx) {
    const Complex current(0.01 + 0.0001 * idx, 0.002 * (idx % 5));
    auto node = DP::SimNode::make("bus" + std::to_string(idx));
    node->setInitialVoltage(current * resistance);

    auto source = DP::Ph1::CurrentSource::make("source" + std::to_string(idx));
    source->setParameters(current);
    source->connect({DP::SimNode::GND, node});

    auto load = DP::Ph1::Resistor::make("load" + std::to_string(idx));
    load->setParameters(resistance);
    load->connect({node, DP::SimNode::GND});

    nodes.push_back(std::move(node));
    components.push_back(std::move(source));
    components.push_back(std::move(load));
  }

  SystemTopology system(50.0, SystemNodeList(nodes.begin(), nodes.end()),
                        components);
  Logger::setLogDir("logs/" + simName);
  auto logger = DataLogger::make(simName);
  for (UInt idx : {0u, branchCount / 2, branchCount - 1})
    logger->logAttribute("bus" + std::to_string(idx) + ".v",
                         nodes[idx]->attribute("v"));

  Simulation simulation(simName);
  simulation.setSystem(system);
  simulation.setDomain(Domain::DP);
  simulation.setSolverType(Solver::Type::MNA);
  simulation.setTimeStep(timeStep);
  simulation.setFinalTime(finalTime);
  simulation.addLogger(logger);
  simulation.run();

  for (UInt idx : {0u, branchCount / 2, branchCount - 1})
    if (!Math::isFinite(nodes[idx]->voltage()(0, 0)))
      throw std::runtime_error(
          "DP resistor grid produced a non-finite voltage.");
}
