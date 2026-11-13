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
  constexpr UInt branchCount = 512;
  constexpr Real frequency = 50.0;
  constexpr Real timeStep = 20e-6;
  constexpr Real finalTime = 0.05;
  const Bool logTrace = argc > 1 && String(argv[1]) == "log";
  const String simName = "EMT_Ph1_CurrentSource_Grid_" +
                         (logTrace ? String("Trace") : String("Benchmark"));

  std::vector<SimNode<Real>::Ptr> buses;
  std::vector<std::shared_ptr<EMT::Ph1::CurrentSource>> sources;
  std::vector<std::shared_ptr<EMT::Ph1::Resistor>> loads;
  SystemComponentList components;
  buses.reserve(branchCount);
  sources.reserve(branchCount);
  loads.reserve(branchCount);
  components.reserve(2 * branchCount);

  for (UInt index = 0; index < branchCount; ++index) {
    auto bus =
        SimNode<Real>::make("bus" + std::to_string(index), PhaseType::Single);
    auto source =
        EMT::Ph1::CurrentSource::make("source" + std::to_string(index));
    source->setParameters(Complex(5.0, 2.0), frequency);
    auto load = EMT::Ph1::Resistor::make("load" + std::to_string(index));
    load->setParameters(10.0);
    source->connect({SimNode<Real>::GND, bus});
    load->connect({bus, SimNode<Real>::GND});
    buses.push_back(bus);
    sources.push_back(source);
    loads.push_back(load);
    components.push_back(source);
    components.push_back(load);
  }

  auto system = SystemTopology(
      frequency, SystemNodeList(buses.begin(), buses.end()), components);
  Simulation simulation(simName, Logger::Level::off);
  simulation.setSystem(system);
  simulation.setDomain(Domain::EMT);
  simulation.setTimeStep(timeStep);
  simulation.setFinalTime(finalTime);
  // Exercise the public/live reference path so a cache must invalidate.
  simulation.addEvent(AttributeEvent<Complex>::make(
      finalTime / 2.0, sources.front()->mCurrentRef, Complex(2.0, -1.0)));
  simulation.addEvent(AttributeEvent<Real>::make(
      3.0 * finalTime / 4.0, sources.front()->mSrcFreq, 60.0));
  if (logTrace) {
    Logger::setLogDir("logs/" + simName);
    auto logger = DataLogger::make(simName);
    logger->logAttribute("bus0_voltage", buses.front()->attribute("v"));
    logger->logAttribute("load0_current", loads.front()->attribute("i_intf"));
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
          "EMT current-source grid produced non-finite voltage.");
  }
  std::cout << "branches=" << branchCount << '\n'
            << "steps=" << static_cast<UInt>(finalTime / timeStep) + 1 << '\n'
            << "simulation_wall_s=" << wallTime << '\n'
            << "simulation_cpu_s=" << cpuTime << '\n';
  return 0;
}
