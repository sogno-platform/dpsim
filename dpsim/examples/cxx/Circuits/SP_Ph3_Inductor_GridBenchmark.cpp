// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <stdexcept>
#include <vector>

#include <DPsim.h>

using namespace CPS;
using namespace DPsim;

int main() {
  constexpr UInt branchCount = 64;
  constexpr Real timeStep = 50e-6;
  constexpr Real finalTime = 0.5;
  const String simName = "SP_Ph3_Inductor_GridBenchmark";

  std::vector<SimNode<Complex>::Ptr> nodes;
  SystemComponentList components;
  nodes.reserve(branchCount);
  components.reserve(2 * branchCount);

  for (UInt idx = 0; idx < branchCount; ++idx) {
    const Complex voltage(100. + idx, 0.1 * idx);
    auto node =
        SimNode<Complex>::make("bus" + std::to_string(idx), PhaseType::ABC);
    node->setInitialVoltage(voltage);

    auto source = SP::Ph3::VoltageSource::make("source" + std::to_string(idx));
    source->setParameters(voltage);
    source->connect({SimNode<Complex>::GND, node});

    auto inductor = SP::Ph3::Inductor::make("inductor" + std::to_string(idx));
    inductor->setParameters(
        Math::singlePhaseParameterToThreePhase(1e-3 * (1. + 0.001 * idx)));
    inductor->connect({node, SimNode<Complex>::GND});

    nodes.push_back(std::move(node));
    components.push_back(std::move(source));
    components.push_back(std::move(inductor));
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
  simulation.setDomain(Domain::SP);
  simulation.setSolverType(Solver::Type::MNA);
  simulation.setTimeStep(timeStep);
  simulation.setFinalTime(finalTime);
  simulation.addLogger(logger);
  simulation.run();

  for (UInt idx : {0u, branchCount / 2, branchCount - 1})
    if (!Math::isFinite(nodes[idx]->voltage()(0, 0)))
      throw std::runtime_error(
          "SP inductor grid produced a non-finite voltage.");
}
