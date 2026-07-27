// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>

#include <DPsim.h>

using namespace CPS;
using namespace DPsim;

namespace {

void requireFinite(Real value, const String &name) {
  if (!Math::isFinite(value))
    throw std::runtime_error(name + " is NaN or Inf.");
}

Real relativeError(Real actual, Real expected) {
  const Real denominator = std::max(std::abs(expected), 1e-12);
  return std::abs(actual - expected) / denominator;
}

} // namespace

int main() {
  try {
    // -------------------------------------------------------------------------
    // Simulation settings
    // -------------------------------------------------------------------------
    constexpr Real timeStep = 1e-5;
    constexpr Real finalTime = 0.5;
    constexpr Real sourceStepTime = 0.1;
    const String simName = "EMT_DC_SSN_PiLine_SourceStep";

    // -------------------------------------------------------------------------
    // Electrical parameters in SI units
    // -------------------------------------------------------------------------
    constexpr Real initialSourceVoltage = 20e3;
    constexpr Real steppedSourceVoltage = 22e3;
    constexpr Real feederResistance = 0.2;
    constexpr Real lineResistance = 0.5;
    constexpr Real lineInductance = 20e-3;
    constexpr Real totalLineCapacitance = 2e-3;
    constexpr Real lineConductance = 0.0;
    constexpr Real loadResistance = 20.0;

    // Pre-step DC steady-state operating point. At DC steady state, the line
    // inductance is a short circuit and both shunt capacitors are open circuits.
    constexpr Real initialCurrent =
        initialSourceVoltage /
        (feederResistance + lineResistance + loadResistance);
    constexpr Real initialSendingVoltage =
        initialSourceVoltage - feederResistance * initialCurrent;
    constexpr Real initialReceivingVoltage = loadResistance * initialCurrent;

    // Expected post-step DC steady-state operating point.
    constexpr Real finalCurrentExpected =
        steppedSourceVoltage /
        (feederResistance + lineResistance + loadResistance);
    constexpr Real finalSendingVoltageExpected =
        steppedSourceVoltage - feederResistance * finalCurrentExpected;
    constexpr Real finalReceivingVoltageExpected =
        loadResistance * finalCurrentExpected;

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "Initial analytical operating point:\n"
              << "  I_dc       = " << initialCurrent << " A\n"
              << "  V_sending  = " << initialSendingVoltage << " V\n"
              << "  V_receiving= " << initialReceivingVoltage << " V\n\n";

    // -------------------------------------------------------------------------
    // Scalar DC nodes
    // -------------------------------------------------------------------------
    auto sourceNode = EMT::SimNode::make("source_node", PhaseType::DC);
    auto sendingNode = EMT::SimNode::make("sending_node", PhaseType::DC);
    auto receivingNode = EMT::SimNode::make("receiving_node", PhaseType::DC);

    sourceNode->setInitialVoltage(Complex(initialSourceVoltage, 0.0));
    sendingNode->setInitialVoltage(Complex(initialSendingVoltage, 0.0));
    receivingNode->setInitialVoltage(Complex(initialReceivingVoltage, 0.0));

    // -------------------------------------------------------------------------
    // Components
    //
    // The scalar DC component convention used by the validation example is:
    // connect({negative_terminal, positive_terminal}).
    // Positive interface current flows from positive to negative terminal.
    // -------------------------------------------------------------------------
    auto source = EMT::DC::VoltageSource::make("dc_source");
    source->setParameters(initialSourceVoltage);
    source->connect({EMT::SimNode::GND, sourceNode});

    auto feeder = EMT::DC::SSN::Resistor::make("feeder");
    feeder->setParameters(feederResistance);
    feeder->connect({sendingNode, sourceNode});

    auto line = EMT::DC::SSN::PiLine::make("dc_pi_line");
    line->setParameters(lineResistance, lineInductance, totalLineCapacitance,
                        lineConductance, initialCurrent);
    line->connect({receivingNode, sendingNode});

    auto load = EMT::DC::SSN::Resistor::make("dc_load");
    load->setParameters(loadResistance);
    load->connect({EMT::SimNode::GND, receivingNode});

    SystemTopology system(
        0.0, SystemNodeList{sourceNode, sendingNode, receivingNode},
        SystemComponentList{source, feeder, line, load});

    // -------------------------------------------------------------------------
    // Time-series logging
    // -------------------------------------------------------------------------
    Logger::setLogDir("logs/" + simName);
    auto logger = DataLogger::make(simName);

    logger->logAttribute("v_source_node", sourceNode->attribute("v"));
    logger->logAttribute("v_sending_node", sendingNode->attribute("v"));
    logger->logAttribute("v_receiving_node", receivingNode->attribute("v"));

    logger->logAttribute("v_source_intf", source->attribute("v_intf"));
    logger->logAttribute("i_source_intf", source->attribute("i_intf"));

    logger->logAttribute("v_feeder_intf", feeder->attribute("v_intf"));
    logger->logAttribute("i_feeder_intf", feeder->attribute("i_intf"));

    logger->logAttribute("v_line_intf", line->attribute("v_intf"));
    logger->logAttribute("i_line_intf", line->attribute("i_intf"));

    logger->logAttribute("v_load_intf", load->attribute("v_intf"));
    logger->logAttribute("i_load_intf", load->attribute("i_intf"));

    // -------------------------------------------------------------------------
    // EMT simulation
    // -------------------------------------------------------------------------
    Simulation simulation(simName, Logger::Level::info);
    simulation.setSystem(system);
    simulation.setDomain(Domain::EMT);
    simulation.setSolverType(Solver::Type::MNA);
    simulation.setTimeStep(timeStep);
    simulation.setFinalTime(finalTime);
    simulation.setLogStepTimes(false);
    simulation.addLogger(logger);

    simulation.initialize();
    simulation.start();

    Bool sourceChanged = false;

    while (simulation.time() < simulation.finalTime()) {
      if (!sourceChanged &&
          simulation.time() >= sourceStepTime - 0.5 * timeStep) {
        source->setParameters(steppedSourceVoltage);
        sourceChanged = true;

        std::cout << "Source-voltage step applied at t = " << simulation.time()
                  << " s: " << initialSourceVoltage << " V -> "
                  << steppedSourceVoltage << " V\n";
      }

      simulation.step();
    }

    simulation.stop();

    // -------------------------------------------------------------------------
    // Final operating point and lightweight safety checks
    // -------------------------------------------------------------------------
    const Real finalSendingVoltage = sendingNode->voltage()(0, 0);
    const Real finalReceivingVoltage = receivingNode->voltage()(0, 0);
    const Real finalLineCurrent = line->intfCurrent()(0, 0);
    const Real finalLoadCurrent = load->intfCurrent()(0, 0);
    const Real finalSourceCurrent = source->intfCurrent()(0, 0);

    requireFinite(finalSendingVoltage, "Final sending-node voltage");
    requireFinite(finalReceivingVoltage, "Final receiving-node voltage");
    requireFinite(finalLineCurrent, "Final line current");
    requireFinite(finalLoadCurrent, "Final load current");
    requireFinite(finalSourceCurrent, "Final source current");

    std::cout << "\nFinal analytical operating point:\n"
              << "  I_dc       = " << finalCurrentExpected << " A\n"
              << "  V_sending  = " << finalSendingVoltageExpected << " V\n"
              << "  V_receiving= " << finalReceivingVoltageExpected << " V\n";

    std::cout << "\nFinal simulated operating point:\n"
              << "  I_line     = " << finalLineCurrent << " A"
              << "  (relative error "
              << relativeError(finalLineCurrent, finalCurrentExpected) << ")\n"
              << "  I_load     = " << finalLoadCurrent << " A\n"
              << "  I_source   = " << finalSourceCurrent << " A\n"
              << "  V_sending  = " << finalSendingVoltage << " V"
              << "  (relative error "
              << relativeError(finalSendingVoltage, finalSendingVoltageExpected)
              << ")\n"
              << "  V_receiving= " << finalReceivingVoltage << " V"
              << "  (relative error "
              << relativeError(finalReceivingVoltage,
                               finalReceivingVoltageExpected)
              << ")\n";

    std::cout << "\nTime-series results written below:\n"
              << "  logs/" << simName << "/\n";

    if (!sourceChanged)
      throw std::runtime_error("The source-voltage step was never applied.");

    return 0;
  } catch (const std::exception &exception) {
    std::cerr << "EMT DC SSN source-step example failed: " << exception.what()
              << '\n';
    return 1;
  }
}
