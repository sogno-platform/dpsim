// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <DPsim.h>

#include <chrono>
#include <iostream>

using namespace DPsim;
using namespace CPS::DP;

int main(int argc, char *[]) {
  constexpr Real timeStep = 20e-6;
  constexpr Real finalTime = 10.0;
  const String simName = "DP_Ph3_SeriesSwitch_Optimization";

  auto node = SimNode::make("bus", PhaseType::ABC);
  auto source = Ph3::VoltageSource::make("source");
  source->setParameters(
      CPS::Math::singlePhaseVariableToThreePhase(Complex(100.0, 0.0)), 50.0);
  auto load = Ph3::SeriesSwitch::make("series_switch");
  load->setParameters(1e6, 10.0);
  load->open();

  source->connect({SimNode::GND, node});
  load->connect({node, SimNode::GND});
  auto system = SystemTopology(50, SystemNodeList{node},
                               SystemComponentList{source, load});

  std::shared_ptr<DataLoggerInterface> logger;
  if (argc == 1) {
    Logger::setLogDir("logs/" + simName);
    auto dataLogger = DataLogger::make(simName);
    dataLogger->logAttribute("node_voltage", node->attribute("v"));
    dataLogger->logAttribute("switch_voltage", load->attribute("v_intf"));
    dataLogger->logAttribute("switch_current", load->attribute("i_intf"));
    logger = dataLogger;
  }

  Simulation simulation(simName, Logger::Level::off);
  simulation.setSystem(system);
  simulation.setDomain(Domain::DP);
  simulation.setTimeStep(timeStep);
  simulation.setFinalTime(finalTime);
  simulation.doSystemMatrixRecomputation(true);
  simulation.addEvent(SwitchEvent::make(2.0, load, true));
  simulation.addEvent(SwitchEvent::make(8.0, load, false));
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
