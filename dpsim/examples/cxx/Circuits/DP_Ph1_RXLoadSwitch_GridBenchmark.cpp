// SPDX-FileCopyrightText: 2026 Institute for Automation of Power Systems, EONERC
// SPDX-License-Identifier: MPL-2.0

#include <DPsim.h>

#include <chrono>
#include <ctime>
#include <iostream>
#include <utility>
#include <vector>

using namespace CPS;
using namespace DPsim;

class RXLoadSwitchOpenEvent : public Event {
  std::shared_ptr<DP::Ph1::RXLoadSwitch> mLoad;

public:
  RXLoadSwitchOpenEvent(Real time, std::shared_ptr<DP::Ph1::RXLoadSwitch> load)
      : Event(time), mLoad(std::move(load)) {}

  void execute() override { mLoad->setSwitchParameters(1e9, 1e-6, false); }
};

int main(int argc, char *argv[]) {
  constexpr UInt branchCount = 128;
  constexpr Real frequency = 50.0;
  constexpr Real timeStep = 100e-6;
  constexpr Real explicitSwitchFinalTime = 0.1;
  constexpr Real automaticTripFinalTime = 1.1;
  constexpr Real nominalVoltage = 400.0;
  constexpr Real switchOpenTime = 0.05;

  bool useSubcomponents = false;
  bool logTrace = false;
  bool automaticTrip = false;
  for (int arg = 1; arg < argc; ++arg) {
    const String option(argv[arg]);
    useSubcomponents = useSubcomponents || option == "legacy";
    logTrace = logTrace || option == "log";
    automaticTrip = automaticTrip || option == "trip";
  }
  const Real finalTime =
      automaticTrip ? automaticTripFinalTime : explicitSwitchFinalTime;
  const String mode = useSubcomponents ? "Legacy" : "Direct";
  const String simName =
      "DP_Ph1_RXLoadSwitch_Grid_" + mode +
      (automaticTrip ? "_AutoTrip" : (logTrace ? "_Trace" : "_Benchmark"));

  std::vector<SimNode<Complex>::Ptr> buses;
  std::vector<std::shared_ptr<DP::Ph1::VoltageSource>> sources;
  std::vector<std::shared_ptr<DP::Ph1::RXLoadSwitch>> loads;
  SystemComponentList components;
  buses.reserve(branchCount);
  sources.reserve(branchCount);
  loads.reserve(branchCount);
  components.reserve(2 * branchCount);

  for (UInt index = 0; index < branchCount; ++index) {
    auto bus = SimNode<Complex>::make("bus" + std::to_string(index),
                                      PhaseType::Single);
    bus->setInitialVoltage(Complex(nominalVoltage, 0.0));
    auto source =
        DP::Ph1::VoltageSource::make("source" + std::to_string(index));
    source->setParameters(Complex(nominalVoltage, 0.0), frequency);
    source->connect({SimNode<Complex>::GND, bus});

    auto load = DP::Ph1::RXLoadSwitch::make("load" + std::to_string(index),
                                            useSubcomponents);
    load->setParameters(1000.0, 0.0,
                        automaticTrip ? nominalVoltage - 100.0 : nominalVoltage,
                        1e9, 1e-6, true);
    load->connect({bus});

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
  simulation.setDomain(Domain::DP);
  simulation.setTimeStep(timeStep);
  simulation.setFinalTime(finalTime);
  if (!automaticTrip) {
    for (const auto &load : loads)
      simulation.addEvent(
          std::make_shared<RXLoadSwitchOpenEvent>(switchOpenTime, load));
  }
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
  UInt openSwitches = 0;
  for (const auto &load : loads)
    openSwitches += !load->mnaIsClosed();
  std::cout << "mode=" << mode << '\n'
            << "branches=" << branchCount << '\n'
            << "steps=" << static_cast<UInt>(finalTime / timeStep) + 1 << '\n'
            << "switch_open_time_s=" << switchOpenTime << '\n'
            << "open_switches=" << openSwitches << '\n'
            << "simulation_wall_s=" << wallTime << '\n'
            << "simulation_cpu_s=" << cpuTime << '\n';
  return 0;
}
