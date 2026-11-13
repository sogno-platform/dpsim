// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <DPsim.h>

#include <chrono>
#include <iostream>

using namespace DPsim;
using namespace CPS::DP;

int main(int argc, char *[]) {
  constexpr Real timeStep = 20e-6;
  constexpr Real finalTime = 3.0;
  const String simName = "DP_Ph1_varResSwitch_Optimization";

  auto node = SimNode::make("bus");
  auto source = Ph1::VoltageSource::make("source");
  source->setParameters(Complex(100.0, 0.0), 50.0);
  auto load = Ph1::Resistor::make("load");
  load->setParameters(100.0);
  auto fault = Ph1::varResSwitch::make("fault");
  fault->setParameters(1e6, 0.1);
  fault->setInitParameters(timeStep);
  fault->openSwitch();

  source->connect({SimNode::GND, node});
  load->connect({node, SimNode::GND});
  fault->connect({node, SimNode::GND});
  auto system = SystemTopology(50, SystemNodeList{node},
                               SystemComponentList{source, load, fault});

  std::shared_ptr<DataLoggerInterface> logger;
  if (argc == 1) {
    Logger::setLogDir("logs/" + simName);
    auto dataLogger = DataLogger::make(simName);
    dataLogger->logAttribute("node_voltage", node->attribute("v"));
    dataLogger->logAttribute("fault_voltage", fault->attribute("v_intf"));
    dataLogger->logAttribute("fault_current", fault->attribute("i_intf"));
    dataLogger->logAttribute("fault_resistance", fault->attribute("R_open"));
    logger = dataLogger;
  }

  Simulation simulation(simName, Logger::Level::off);
  simulation.setSystem(system);
  simulation.setDomain(Domain::DP);
  simulation.setTimeStep(timeStep);
  simulation.setFinalTime(finalTime);
  simulation.doSystemMatrixRecomputation(true);
  simulation.addEvent(SwitchEvent::make(1.0, fault, true));
  simulation.addEvent(SwitchEvent::make(2.0, fault, false));
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
