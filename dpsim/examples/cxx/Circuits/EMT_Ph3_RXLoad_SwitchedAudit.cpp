// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <DPsim.h>

#include <chrono>
#include <cmath>
#include <iostream>

using namespace CPS;
using namespace DPsim;

int main(int argc, char *argv[]) {
  constexpr Real frequency = 50.0;
  constexpr Real timeStep = 50e-6;
  constexpr Real finalTime = 0.15;
  constexpr Real nominalVoltage = 100e3;
  const Bool useSubcomponents = argc > 1 && String(argv[1]) == "legacy";
  const String variant = useSubcomponents ? "legacy" : "direct";
  const String simName = "EMT_Ph3_RXLoad_Switched_" + variant;

  const std::vector<Complex> initialVoltage{Complex(nominalVoltage, 0.0),
                                            Complex(nominalVoltage, 0.0),
                                            Complex(nominalVoltage, 0.0)};
  auto sourceNode =
      EMT::SimNode::make("source_bus", PhaseType::ABC, initialVoltage);
  auto loadNode =
      EMT::SimNode::make("load_bus", PhaseType::ABC, initialVoltage);

  auto source = EMT::Ph3::VoltageSource::make("grid_source");
  source->setParameters(Math::singlePhaseVariableToThreePhase(
                            Complex(nominalVoltage * PEAK1PH_TO_RMS3PH, 0.0)),
                        frequency);
  source->connect({EMT::SimNode::GND, sourceNode});

  auto contact = EMT::Ph3::Switch::make("load_contact");
  contact->setParameters(Math::singlePhaseParameterToThreePhase(1e8),
                         Math::singlePhaseParameterToThreePhase(1e-4));
  contact->connect({sourceNode, loadNode});

  auto load = EMT::Ph3::RXLoad::make("rx_load", !useSubcomponents);
  load->setParameters(Matrix::Identity(3, 3) * 1e6,
                      Matrix::Identity(3, 3) * 0.2e6, nominalVoltage);
  load->connect({loadNode});

  SystemTopology system(frequency, {sourceNode, loadNode},
                        {source, contact, load});
  Logger::setLogDir("logs/" + simName);
  auto logger = DataLogger::make(simName);
  logger->logAttribute("source_bus_voltage", sourceNode->attribute("v"));
  logger->logAttribute("load_bus_voltage", loadNode->attribute("v"));
  logger->logAttribute("load_voltage", load->attribute("v_intf"));
  logger->logAttribute("load_current", load->attribute("i_intf"));
  logger->logAttribute("contact_current", contact->attribute("i_intf"));

  Simulation simulation(simName, Logger::Level::off);
  simulation.setSystem(system);
  simulation.setDomain(Domain::EMT);
  simulation.setTimeStep(timeStep);
  simulation.setFinalTime(finalTime);
  simulation.addLogger(logger);
  simulation.addEvent(SwitchEvent3Ph::make(0.05, contact, false));
  simulation.addEvent(SwitchEvent3Ph::make(0.10, contact, true));

  const auto start = std::chrono::steady_clock::now();
  simulation.run();
  const Real wallTime =
      std::chrono::duration<Real>(std::chrono::steady_clock::now() - start)
          .count();

  for (const auto &node : {sourceNode, loadNode})
    for (Int phase = 0; phase < 3; ++phase)
      if (!Math::isFinite(node->voltage()(phase, 0))) {
        std::cerr
            << "Switched RX-load test produced a non-finite bus voltage.\n";
        return 1;
      }
  for (Int phase = 0; phase < 3; ++phase)
    if (!Math::isFinite(load->intfCurrent()(phase, 0)) ||
        !Math::isFinite(contact->intfCurrent()(phase, 0))) {
      std::cerr << "Switched RX-load test produced a non-finite current.\n";
      return 1;
    }

  std::cout << "realization=" << variant << '\n'
            << "steps="
            << static_cast<UInt>(std::llround(finalTime / timeStep)) + 1 << '\n'
            << "simulation_wall_s=" << wallTime << '\n';
  return 0;
}
