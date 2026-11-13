// SPDX-FileCopyrightText: 2026 Institute for Automation of Power Systems, EONERC
// SPDX-License-Identifier: MPL-2.0

#include <DPsim.h>

#include <chrono>
#include <ctime>
#include <iostream>
#include <vector>

using namespace CPS;
using namespace DPsim;

int main(int argc, char *argv[]) {
  constexpr UInt branchCount = 128;
  constexpr Real frequency = 50.0;
  constexpr Real timeStep = 20e-6;
  constexpr Real finalTime = 0.05;
  const Bool useSubcomponents = argc > 1 && String(argv[1]) == "legacy";
  const Bool logTrace = argc > 1 && String(argv[argc - 1]) == "log";
  const String mode = useSubcomponents ? "Legacy" : "Direct";
  const String simName =
      "EMT_Ph1_PiLine_Grid_" + mode + (logTrace ? "_Trace" : "_Benchmark");

  std::vector<SimNode<Real>::Ptr> sourceBuses;
  std::vector<SimNode<Real>::Ptr> loadBuses;
  std::vector<std::shared_ptr<EMT::Ph1::VoltageSource>> sources;
  std::vector<std::shared_ptr<EMT::Ph1::PiLine>> lines;
  std::vector<std::shared_ptr<EMT::Ph1::Resistor>> loads;
  SystemComponentList components;
  sourceBuses.reserve(branchCount);
  loadBuses.reserve(branchCount);
  sources.reserve(branchCount);
  lines.reserve(branchCount);
  loads.reserve(branchCount);
  components.reserve(3 * branchCount);

  for (UInt index = 0; index < branchCount; ++index) {
    auto sourceBus = SimNode<Real>::make("source_bus" + std::to_string(index),
                                         PhaseType::Single);
    auto loadBus = SimNode<Real>::make("load_bus" + std::to_string(index),
                                       PhaseType::Single);
    auto source =
        EMT::Ph1::VoltageSource::make("source" + std::to_string(index));
    source->setParameters(Complex(230.0, 0.0), frequency);
    auto line = EMT::Ph1::PiLine::make("line" + std::to_string(index),
                                       useSubcomponents);
    line->setParameters(0.1, 1e-3, 2e-6, 1e-6);
    auto load = EMT::Ph1::Resistor::make("load" + std::to_string(index));
    load->setParameters(10.0);
    source->connect({SimNode<Real>::GND, sourceBus});
    line->connect({sourceBus, loadBus});
    load->connect({loadBus, SimNode<Real>::GND});
    sourceBuses.push_back(sourceBus);
    loadBuses.push_back(loadBus);
    sources.push_back(source);
    lines.push_back(line);
    loads.push_back(load);
    components.push_back(source);
    components.push_back(line);
    components.push_back(load);
  }

  SystemNodeList nodes;
  nodes.reserve(2 * branchCount);
  for (const auto &node : sourceBuses)
    nodes.push_back(node);
  for (const auto &node : loadBuses)
    nodes.push_back(node);
  auto system = SystemTopology(frequency, nodes, components);
  Simulation simulation(simName, Logger::Level::off);
  simulation.setSystem(system);
  simulation.setDomain(Domain::EMT);
  simulation.setTimeStep(timeStep);
  simulation.setFinalTime(finalTime);
  if (logTrace) {
    Logger::setLogDir("logs/" + simName);
    auto logger = DataLogger::make(simName);
    logger->logAttribute("line0_voltage", lines.front()->attribute("v_intf"));
    logger->logAttribute("line0_current", lines.front()->attribute("i_intf"));
    logger->logAttribute("load0_voltage", loadBuses.front()->attribute("v"));
    simulation.addLogger(logger);
  }

  const auto wallStart = std::chrono::steady_clock::now();
  const std::clock_t cpuStart = std::clock();
  simulation.run();
  const Real wallTime =
      std::chrono::duration<Real>(std::chrono::steady_clock::now() - wallStart)
          .count();
  const Real cpuTime =
      static_cast<Real>(std::clock() - cpuStart) / CLOCKS_PER_SEC;

  for (UInt index : {0u, branchCount / 2, branchCount - 1}) {
    if (!Math::isFinite(loadBuses[index]->voltage()(0, 0)))
      throw std::runtime_error("EMT PiLine grid produced non-finite voltage.");
  }
  std::cout << "mode=" << mode << '\n'
            << "branches=" << branchCount << '\n'
            << "steps=" << static_cast<UInt>(finalTime / timeStep) + 1 << '\n'
            << "simulation_wall_s=" << wallTime << '\n'
            << "simulation_cpu_s=" << cpuTime << '\n';
  return 0;
}
