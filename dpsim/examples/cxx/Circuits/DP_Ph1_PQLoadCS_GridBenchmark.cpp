// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <stdexcept>
#include <vector>

#include <DPsim.h>

using namespace CPS;
using namespace DPsim;

int main() {
  constexpr UInt branchCount = 256;
  constexpr Real voltage = 400.0;
  constexpr Real timeStep = 20e-6;
  constexpr Real finalTime = 0.1;
  const String simName = "DP_Ph1_PQLoadCS_GridBenchmark";

  std::vector<DP::SimNode::Ptr> nodes;
  std::vector<std::shared_ptr<DP::Ph1::PQLoadCS>> loads;
  SystemComponentList components;
  nodes.reserve(branchCount);
  loads.reserve(branchCount);
  components.reserve(2 * branchCount);

  for (UInt idx = 0; idx < branchCount; ++idx) {
    auto node = DP::SimNode::make("bus" + std::to_string(idx));
    node->setInitialVoltage(Complex(voltage, 0.));

    auto source = DP::Ph1::VoltageSource::make("source" + std::to_string(idx));
    source->setParameters(Complex(voltage, 0.), 50.0);
    source->connect({DP::SimNode::GND, node});

    auto load = DP::Ph1::PQLoadCS::make("load" + std::to_string(idx), false);
    load->setParameters(1000. + idx, 500. + idx, voltage);
    load->connect({node});

    nodes.push_back(std::move(node));
    loads.push_back(load);
    components.push_back(std::move(source));
    components.push_back(std::move(load));
  }

  SystemTopology system(50.0, SystemNodeList(nodes.begin(), nodes.end()),
                        components);
  Logger::setLogDir("logs/" + simName);
  auto logger = DataLogger::make(simName);
  logger->logAttribute("load0.i", loads[0]->attribute("i_intf"));
  for (UInt idx : {0u, branchCount / 2, branchCount - 1})
    logger->logAttribute("bus" + std::to_string(idx) + ".v",
                         nodes[idx]->attribute("v"));

  Simulation simulation(simName);
  simulation.setSystem(system);
  simulation.setDomain(Domain::DP);
  simulation.setSolverType(Solver::Type::MNA);
  simulation.setTimeStep(timeStep);
  simulation.setFinalTime(finalTime);
  simulation.addEvent(
      AttributeEvent<Real>::make(0.05, loads[0]->mActivePower, 2000.));
  simulation.addLogger(logger);
  simulation.run();

  for (UInt idx : {0u, branchCount / 2, branchCount - 1})
    if (!Math::isFinite(nodes[idx]->voltage()(0, 0)))
      throw std::runtime_error(
          "DP PQLoadCS grid produced a non-finite voltage.");
}
