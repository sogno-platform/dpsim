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
  const Bool logTrace = String(argv[argc > 1 ? argc - 1 : 0]) == "log";
  const String mode = useSubcomponents ? "Legacy" : "Direct";
  const String simName = "DP_Ph3_NetworkInjection_Grid_" + mode +
                         (logTrace ? "_Trace" : "_Benchmark");

  std::vector<SimNode<Complex>::Ptr> buses;
  std::vector<std::shared_ptr<DP::Ph3::NetworkInjection>> sources;
  SystemComponentList components;
  buses.reserve(branchCount);
  sources.reserve(branchCount);
  components.reserve(branchCount);
  const MatrixComp reference =
      Math::singlePhaseVariableToThreePhase(Complex(230.0, 0.0));
  for (UInt index = 0; index < branchCount; ++index) {
    auto bus =
        SimNode<Complex>::make("bus" + std::to_string(index), PhaseType::ABC);
    auto source = DP::Ph3::NetworkInjection::make(
        "source" + std::to_string(index), useSubcomponents);
    source->setParameters(reference, frequency);
    source->connect({bus});
    buses.push_back(bus);
    sources.push_back(source);
    components.push_back(source);
  }

  auto system = SystemTopology(
      frequency, SystemNodeList(buses.begin(), buses.end()), components);
  Simulation simulation(simName, Logger::Level::off);
  simulation.setSystem(system);
  simulation.setDomain(Domain::DP);
  simulation.setTimeStep(timeStep);
  simulation.setFinalTime(finalTime);
  if (logTrace) {
    Logger::setLogDir("logs/" + simName);
    auto logger = DataLogger::make(simName);
    logger->logAttribute("bus0_voltage", buses.front()->attribute("v"));
    logger->logAttribute("source0_voltage",
                         sources.front()->attribute("v_intf"));
    logger->logAttribute("source0_current",
                         sources.front()->attribute("i_intf"));
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
    if (!Math::isFinite(buses[index]->voltage()(0, 0)))
      throw std::runtime_error(
          "DP network-injection grid produced non-finite voltage.");
  }
  std::cout << "mode=" << mode << '\n'
            << "branches=" << branchCount << '\n'
            << "steps=" << static_cast<UInt>(finalTime / timeStep) + 1 << '\n'
            << "simulation_wall_s=" << wallTime << '\n'
            << "simulation_cpu_s=" << cpuTime << '\n';
  return 0;
}
