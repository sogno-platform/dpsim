// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <chrono>
#include <iostream>
#include <stdexcept>
#include <vector>

#include <DPsim.h>

using namespace CPS;
using namespace DPsim;

namespace {
constexpr UInt lineCount = 32;
constexpr Real timeStep = 10e-6;
constexpr Real finalTime = 0.5;
constexpr Real frequency = 50.0;

Real runCase(Bool useSubcomponents, UInt run) {
  const String mode = useSubcomponents ? "Legacy" : "Direct";
  const String name =
      "DP_Ph1_RxLine_GridBenchmark_" + mode + "_" + std::to_string(run);

  std::vector<DP::SimNode::Ptr> nodes;
  std::vector<DP::Ph1::RxLine::Ptr> lines;
  SystemComponentList components;
  nodes.reserve(2 * lineCount);
  lines.reserve(lineCount);
  components.reserve(3 * lineCount);

  constexpr Real voltage = 400.0;
  constexpr Real loadResistance = voltage * voltage / 1000.0;
  for (UInt idx = 0; idx < lineCount; ++idx) {
    auto bus0 = DP::SimNode::make("bus" + std::to_string(idx) + "_0");
    auto bus1 = DP::SimNode::make("bus" + std::to_string(idx) + "_1");
    bus0->setInitialVoltage(Complex(voltage, 0.0));
    bus1->setInitialVoltage(Complex(voltage * 0.99, -2.0));

    auto source = DP::Ph1::VoltageSource::make("source" + std::to_string(idx));
    source->setParameters(Complex(voltage, 0.0), frequency);
    source->connect({DP::SimNode::GND, bus0});

    auto line =
        DP::Ph1::RxLine::make("line" + std::to_string(idx), useSubcomponents);
    line->setParameters(0.1, 1e-3);
    line->connect({bus0, bus1});

    auto load = DP::Ph1::Resistor::make("load" + std::to_string(idx));
    load->setParameters(loadResistance);
    load->connect({bus1, DP::SimNode::GND});

    nodes.push_back(std::move(bus0));
    nodes.push_back(std::move(bus1));
    components.push_back(std::move(source));
    components.push_back(line);
    components.push_back(std::move(load));
    lines.push_back(std::move(line));
  }

  SystemTopology system(frequency, SystemNodeList(nodes.begin(), nodes.end()),
                        components);
  Logger::setLogDir("logs/" + name);
  auto logger = DataLogger::make(name);
  for (UInt idx : {0u, lineCount / 2, lineCount - 1})
    logger->logAttribute("bus" + std::to_string(idx) + ".v",
                         nodes[2 * idx + 1]->attribute("v"));
  for (UInt idx : {0u, lineCount / 2, lineCount - 1})
    logger->logAttribute("line" + std::to_string(idx) + ".i",
                         lines[idx]->attribute("i_intf"));

  Simulation simulation(name, Logger::Level::off);
  simulation.setSystem(system);
  simulation.setDomain(Domain::DP);
  simulation.setSolverType(Solver::Type::MNA);
  simulation.setTimeStep(timeStep);
  simulation.setFinalTime(finalTime);
  simulation.addLogger(logger);

  const auto start = std::chrono::steady_clock::now();
  simulation.run();
  const Real elapsed =
      std::chrono::duration<Real>(std::chrono::steady_clock::now() - start)
          .count();

  for (UInt idx : {0u, lineCount / 2, lineCount - 1})
    if (!Math::isFinite(nodes[2 * idx + 1]->voltage()(0, 0)))
      throw std::runtime_error("DP RxLine grid produced a non-finite voltage.");

  return elapsed;
}
} // namespace

int main(int argc, char *argv[]) {
  if (argc < 2 || (String(argv[1]) != "legacy" && String(argv[1]) != "direct"))
    throw std::invalid_argument(
        "Usage: DP_Ph1_RxLine_GridBenchmark legacy|direct [runs]");

  const Bool useSubcomponents = String(argv[1]) == "legacy";
  const UInt runs = argc >= 3 ? static_cast<UInt>(std::stoul(argv[2])) : 5;
  Real total = 0.;
  for (UInt run = 0; run < runs; ++run) {
    const Real elapsed = runCase(useSubcomponents, run + 1);
    total += elapsed;
    std::cout << (useSubcomponents ? "legacy" : "direct") << " run " << run + 1
              << ": " << elapsed << " s\n";
  }
  std::cout << "mean (" << runs << " runs, " << lineCount << " lines, "
            << finalTime << " s at " << timeStep << " s): " << total / runs
            << " s\n";
}
