// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <DPsim.h>

#include <chrono>
#include <iostream>

using namespace DPsim;
using namespace CPS::EMT;

int main(int argc, char *[]) {
  constexpr Real timeStep = 20e-6;
  constexpr Real finalTime = 10.0;
  const String simName = "EMT_Ph1_VoltageSourceNorton_Optimization";

  auto node = SimNode::make("n1");
  auto source = Ph1::VoltageSourceNorton::make("norton_source");
  source->setParameters(Complex(10.0, 1.0), 50.0, 1.0);
  auto load = Ph1::Resistor::make("load");
  load->setParameters(10.0);

  source->connect({SimNode::GND, node});
  load->connect({node, SimNode::GND});
  auto system = SystemTopology(50, SystemNodeList{node},
                               SystemComponentList{source, load});

  std::shared_ptr<DataLoggerInterface> logger;
  if (argc == 1) {
    Logger::setLogDir("logs/" + simName);
    auto dataLogger = DataLogger::make(simName);
    dataLogger->logAttribute("node_voltage", node->attribute("v"));
    dataLogger->logAttribute("source_current", source->attribute("i_intf"));
    dataLogger->logAttribute("source_voltage", source->attribute("v_intf"));
    logger = dataLogger;
  }

  Simulation simulation(simName, Logger::Level::off);
  simulation.setSystem(system);
  simulation.setDomain(Domain::EMT);
  simulation.setTimeStep(timeStep);
  simulation.setFinalTime(finalTime);
  if (logger)
    simulation.addLogger(logger);
  const auto start = std::chrono::steady_clock::now();
  simulation.run();
  const auto elapsed =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - start)
          .count();
  std::cout << "simulation_wall_s=" << elapsed << '\n';
  return 0;
}
