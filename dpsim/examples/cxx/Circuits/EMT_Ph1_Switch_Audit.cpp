// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <DPsim.h>

#include <chrono>
#include <cmath>
#include <iostream>

using namespace DPsim;
using namespace CPS::EMT;

int main(int argc, char *[]) {
  constexpr Real timeStep = 20e-6;
  constexpr Real finalTime = 1.0;
  const bool usePrecomputedSwitchMatrices = argc > 1;
  const String simName = usePrecomputedSwitchMatrices
                             ? "EMT_Ph1_Switch_Audit_Precomputed"
                             : "EMT_Ph1_Switch_Audit_Recompute";

  auto sourceNode = SimNode::make("source", PhaseType::Single);
  auto loadNode = SimNode::make("load", PhaseType::Single);
  auto source = Ph1::VoltageSource::make("source");
  source->setParameters(Complex(100.0, 0.0), 50.0);
  auto switchComp = Ph1::Switch::make("switch");
  switchComp->setParameters(1e6, 0.1, false);
  auto load = Ph1::Resistor::make("load");
  load->setParameters(10.0);

  source->connect({SimNode::GND, sourceNode});
  switchComp->connect({sourceNode, loadNode});
  load->connect({loadNode, SimNode::GND});
  auto system = SystemTopology(50.0, SystemNodeList{sourceNode, loadNode},
                               SystemComponentList{source, switchComp, load});

  std::shared_ptr<DataLoggerInterface> logger;
  Logger::setLogDir("logs/" + simName);
  auto dataLogger = DataLogger::make(simName);
  dataLogger->logAttribute("source_voltage", sourceNode->attribute("v"));
  dataLogger->logAttribute("load_voltage", loadNode->attribute("v"));
  dataLogger->logAttribute("switch_voltage", switchComp->attribute("v_intf"));
  dataLogger->logAttribute("switch_current", switchComp->attribute("i_intf"));
  dataLogger->logAttribute("load_current", load->attribute("i_intf"));
  dataLogger->logAttribute("switch_closed", switchComp->mIsClosed);
  logger = dataLogger;

  Simulation simulation(simName, Logger::Level::off);
  simulation.setSystem(system);
  simulation.setDomain(Domain::EMT);
  simulation.setTimeStep(timeStep);
  simulation.setFinalTime(finalTime);
  simulation.doSystemMatrixRecomputation(!usePrecomputedSwitchMatrices);
  simulation.addEvent(SwitchEvent::make(0.25, switchComp, true));
  simulation.addEvent(SwitchEvent::make(0.75, switchComp, false));
  if (logger)
    simulation.addLogger(logger);

  const auto start = std::chrono::steady_clock::now();
  simulation.run();
  const auto elapsed =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - start)
          .count();
  const Real finalLoadVoltage = (**loadNode->attributeTyped<Matrix>("v"))(0, 0);
  const Bool finalSwitchClosed =
      **switchComp->attributeTyped<Bool>("is_closed");
  if (finalSwitchClosed || std::abs(finalLoadVoltage - 1e-3) > 1e-8) {
    std::cerr << "switch-open regression: closed=" << finalSwitchClosed
              << ", load_voltage=" << finalLoadVoltage << '\n';
    return 1;
  }
  std::cout << "simulation_wall_s=" << elapsed << '\n';
  return 0;
}
