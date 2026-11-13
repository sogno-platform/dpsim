// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <DPsim.h>

#include <algorithm>
#include <chrono>
#include <iostream>
#include <vector>

using namespace DPsim;
using namespace CPS::DP;

int main(int argc, char *argv[]) {
  constexpr UInt branchCount = 64;
  constexpr Real voltage = 400.0;
  constexpr Real frequency = 50.0;
  constexpr Real timeStep = 10e-6;
  constexpr Real finalTime = 0.04;
  const Bool legacy = argc > 1 && String(argv[1]) == "legacy";
  const String mode = legacy ? "Legacy" : "Direct";
  const String simName = "DP_Ph1_SVC_GridBenchmark_" + mode;

  std::vector<SimNode::Ptr> buses;
  std::vector<std::shared_ptr<Ph1::SVC>> svcs;
  SystemComponentList components;
  buses.reserve(branchCount);
  svcs.reserve(branchCount);
  components.reserve(3 * branchCount);

  for (UInt index = 0; index < branchCount; ++index) {
    auto bus = SimNode::make("bus" + std::to_string(index));
    bus->setInitialVoltage(Complex(voltage, 0.0));
    auto source = Ph1::VoltageSource::make("source" + std::to_string(index));
    source->setParameters(Complex(voltage, 0.0), frequency);
    source->connect({SimNode::GND, bus});

    auto svc = Ph1::SVC::make("svc" + std::to_string(index), legacy);
    svc->setParameters(1.0, -1.0, 1e5, voltage, voltage);
    svc->setControllerParameters(0.01, 1.0);
    svc->connect({bus});

    auto load = Ph1::Resistor::make("load" + std::to_string(index));
    load->setParameters(voltage * voltage / 1000.0);
    load->connect({bus, SimNode::GND});

    buses.push_back(bus);
    svcs.push_back(svc);
    components.push_back(source);
    components.push_back(svc);
    components.push_back(load);
  }

  SystemTopology system(frequency, SystemNodeList(buses.begin(), buses.end()),
                        components);
  Logger::setLogDir("logs/" + simName);
  auto logger = DataLogger::make(simName);
  for (UInt index = 0; index < branchCount; ++index) {
    const String prefix = "svc" + std::to_string(index) + "_";
    logger->logAttribute(prefix + "voltage", svcs[index]->attribute("v_intf"));
    logger->logAttribute(prefix + "current", svcs[index]->attribute("i_intf"));
  }

  Simulation simulation(simName, Logger::Level::off);
  simulation.setSystem(system);
  simulation.setDomain(Domain::DP);
  simulation.setTimeStep(timeStep);
  simulation.setFinalTime(finalTime);
  simulation.addLogger(logger);
  const auto start = std::chrono::steady_clock::now();
  simulation.run();
  const Real wallTime =
      std::chrono::duration<Real>(std::chrono::steady_clock::now() - start)
          .count();

  Real maxVoltage = 0.0;
  Real maxCurrent = 0.0;
  for (const auto &svc : svcs) {
    const auto &svcVoltage = **svc->attributeTyped<MatrixComp>("v_intf");
    const auto &svcCurrent = **svc->attributeTyped<MatrixComp>("i_intf");
    if (!svcVoltage.allFinite() || !svcCurrent.allFinite()) {
      std::cerr << "Non-finite SVC output in " << svc->name() << '\n';
      return 1;
    }
    maxVoltage = std::max(maxVoltage, std::abs(svcVoltage(0, 0)));
    maxCurrent = std::max(maxCurrent, std::abs(svcCurrent(0, 0)));
  }

  std::cout << "mode=" << mode << '\n'
            << "branches=" << branchCount << '\n'
            << "steps=" << static_cast<UInt>(finalTime / timeStep) + 1 << '\n'
            << "max_svc_voltage=" << maxVoltage << '\n'
            << "max_svc_current=" << maxCurrent << '\n'
            << "simulation_wall_s=" << wallTime << '\n';
  return 0;
}
