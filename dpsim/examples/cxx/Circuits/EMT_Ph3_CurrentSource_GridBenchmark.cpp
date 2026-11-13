// SPDX-FileCopyrightText: 2026 Institute for Automation of Power Systems, EONERC
// SPDX-License-Identifier: MPL-2.0

#include <DPsim.h>

#include <algorithm>
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
  constexpr Real resistance = 10.0;

  bool logTrace = false;
  bool controlledSource = false;
  for (int arg = 1; arg < argc; ++arg) {
    const String option(argv[arg]);
    logTrace = logTrace || option == "log";
    controlledSource = controlledSource || option == "controlled";
  }
  const String sourceMode =
      controlledSource ? "ControlledCurrentSource" : "CurrentSource";
  const String simName =
      "EMT_Ph3_" + sourceMode + "_Grid" + (logTrace ? "Trace" : "Benchmark");

  std::vector<EMT::SimNode::Ptr> buses;
  std::vector<SimPowerComp<Real>::Ptr> sources;
  SystemComponentList components;
  buses.reserve(branchCount);
  sources.reserve(branchCount);
  components.reserve(2 * branchCount);

  const MatrixComp currentReference =
      Math::singlePhaseVariableToThreePhase(Complex(1.0, 0.0));
  for (UInt index = 0; index < branchCount; ++index) {
    auto bus =
        EMT::SimNode::make("bus" + std::to_string(index), PhaseType::ABC);
    SimPowerComp<Real>::Ptr source;
    if (controlledSource) {
      auto controlled = EMT::Ph3::ControlledCurrentSource::make(
          "source" + std::to_string(index));
      controlled->setParameters(RMS_TO_PEAK * currentReference.real());
      source = controlled;
    } else {
      auto sinusoidal =
          EMT::Ph3::CurrentSource::make("source" + std::to_string(index));
      sinusoidal->setParameters(currentReference, frequency);
      source = sinusoidal;
    }
    source->connect({EMT::SimNode::GND, bus});

    auto load = EMT::Ph3::Resistor::make("load" + std::to_string(index));
    load->setParameters(resistance * Matrix::Identity(3, 3));
    load->connect({bus, EMT::SimNode::GND});

    buses.push_back(bus);
    sources.push_back(source);
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

  const auto start = std::chrono::steady_clock::now();
  const std::clock_t cpuStart = std::clock();
  simulation.run();
  const Real wallTime =
      std::chrono::duration<Real>(std::chrono::steady_clock::now() - start)
          .count();
  const Real cpuTime =
      static_cast<Real>(std::clock() - cpuStart) / CLOCKS_PER_SEC;

  Real maxVoltage = 0.0;
  Real maxCurrent = 0.0;
  for (const auto &source : sources) {
    const auto &voltage = **source->attributeTyped<Matrix>("v_intf");
    const auto &current = **source->attributeTyped<Matrix>("i_intf");
    if (!voltage.allFinite() || !current.allFinite()) {
      std::cerr << "Non-finite source output in " << source->name() << '\n';
      return 1;
    }
    maxVoltage = std::max(maxVoltage, voltage.cwiseAbs().maxCoeff());
    maxCurrent = std::max(maxCurrent, current.cwiseAbs().maxCoeff());
  }

  std::cout << "source_type=" << sourceMode << '\n'
            << "branches=" << branchCount << '\n'
            << "steps=" << static_cast<UInt>(finalTime / timeStep) + 1 << '\n'
            << "max_source_voltage=" << maxVoltage << '\n'
            << "max_source_current=" << maxCurrent << '\n'
            << "simulation_wall_s=" << wallTime << '\n'
            << "simulation_cpu_s=" << cpuTime << '\n';
  return 0;
}
