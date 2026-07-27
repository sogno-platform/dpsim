// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

#include <DPsim.h>

using namespace CPS;
using namespace DPsim;

namespace {

struct Sample {
  Real voltage;
  Real current;
  Real power;
  Real energy;
};

void requireFinite(const Sample &sample, const String &name) {
  if (!Math::isFinite(sample.voltage) || !Math::isFinite(sample.current) ||
      !Math::isFinite(sample.power) || !Math::isFinite(sample.energy))
    throw std::runtime_error(name + " produced a non-finite signal.");
}

void requireNear(Real actual, Real expected, Real tolerance,
                 const String &name) {
  const Real error = std::abs(actual - expected);
  std::cout << name << ": actual=" << actual << ", expected=" << expected
            << ", abs_error=" << error << ", tolerance=" << tolerance << '\n';
  if (error > tolerance)
    throw std::runtime_error(name + " exceeded its error tolerance.");
}

template <typename Callable>
void requireInvalidParameter(Callable &&callable, const String &name) {
  try {
    callable();
  } catch (const std::invalid_argument &) {
    std::cout << name << ": correctly rejected\n";
    return;
  }
  throw std::runtime_error(name + " was not rejected.");
}

void validateInvalidParameters() {
  const Real epsilon = std::numeric_limits<Real>::epsilon();

  auto resistor = EMT::DC::SSN::Resistor::make("invalid_resistor");
  requireInvalidParameter([&] { resistor->setParameters(0.0); },
                          "Zero DC resistance");

  auto capacitor = EMT::DC::SSN::Capacitor::make("invalid_capacitor");
  requireInvalidParameter([&] { capacitor->setParameters(epsilon); },
                          "Near-zero DC capacitance");

  auto inductor = EMT::DC::SSN::Inductor::make("invalid_inductor");
  requireInvalidParameter([&] { inductor->setParameters(0.0); },
                          "Zero DC inductance");

  auto line = EMT::DC::SSN::PiLine::make("invalid_pi_line");
  requireInvalidParameter([&] { line->setParameters(1.0, 1.0, epsilon, 0.0); },
                          "Near-zero DC pi-line shunt capacitance");

  auto source = EMT::DC::VoltageSource::make("invalid_source");
  requireInvalidParameter(
      [&] { source->setParameters(std::numeric_limits<Real>::infinity()); },
      "Non-finite DC source voltage");
}

Sample runRC(Real targetTime, Real timeStep, Real sourceVoltage,
             Real resistance, Real capacitance, const String &suffix) {
  auto supply = EMT::SimNode::make("supply_" + suffix, PhaseType::DC);
  auto capacitorNode =
      EMT::SimNode::make("capacitor_node_" + suffix, PhaseType::DC);
  supply->setInitialVoltage(Complex(0.0, 0.0));
  capacitorNode->setInitialVoltage(Complex(0.0, 0.0));

  auto source = EMT::DC::VoltageSource::make("source_" + suffix);
  source->setParameters(sourceVoltage);
  source->connect({EMT::SimNode::GND, supply});

  auto resistor = EMT::DC::SSN::Resistor::make("resistor_" + suffix);
  resistor->setParameters(resistance);
  resistor->connect({capacitorNode, supply});

  auto capacitor = EMT::DC::SSN::Capacitor::make("capacitor_" + suffix);
  capacitor->setParameters(capacitance);
  capacitor->connect({EMT::SimNode::GND, capacitorNode});

  SystemTopology system(0.0, SystemNodeList{supply, capacitorNode},
                        SystemComponentList{source, resistor, capacitor});

  Simulation simulation("EMT_DC_RC_" + suffix, Logger::Level::off);
  simulation.setSystem(system);
  simulation.setDomain(Domain::EMT);
  simulation.setTimeStep(timeStep);
  simulation.setFinalTime(targetTime);
  simulation.initialize();
  requireNear(capacitor->intfVoltage()(0, 0), 0.0, 0.0,
              "RC initialized capacitor voltage");
  simulation.run();

  const Real voltage = capacitor->intfVoltage()(0, 0);
  const Real current = capacitor->intfCurrent()(0, 0);
  return {voltage, current, voltage * current,
          0.5 * capacitance * voltage * voltage};
}

Sample runRL(Real targetTime, Real timeStep, Real sourceVoltage,
             Real resistance, Real inductance, const String &suffix) {
  auto supply = EMT::SimNode::make("supply_" + suffix, PhaseType::DC);
  auto inductorNode =
      EMT::SimNode::make("inductor_node_" + suffix, PhaseType::DC);
  supply->setInitialVoltage(Complex(0.0, 0.0));
  inductorNode->setInitialVoltage(Complex(0.0, 0.0));

  auto source = EMT::DC::VoltageSource::make("source_" + suffix);
  source->setParameters(sourceVoltage);
  source->connect({EMT::SimNode::GND, supply});

  auto resistor = EMT::DC::SSN::Resistor::make("resistor_" + suffix);
  resistor->setParameters(resistance);
  resistor->connect({inductorNode, supply});

  auto inductor = EMT::DC::SSN::Inductor::make("inductor_" + suffix);
  inductor->setParameters(inductance, 0.0);
  inductor->connect({EMT::SimNode::GND, inductorNode});

  SystemTopology system(0.0, SystemNodeList{supply, inductorNode},
                        SystemComponentList{source, resistor, inductor});

  Simulation simulation("EMT_DC_RL_" + suffix, Logger::Level::off);
  simulation.setSystem(system);
  simulation.setDomain(Domain::EMT);
  simulation.setTimeStep(timeStep);
  simulation.setFinalTime(targetTime);
  simulation.initialize();
  requireNear(inductor->intfCurrent()(0, 0), 0.0, 0.0,
              "RL initialized inductor current");
  simulation.run();

  const Real voltage = inductor->intfVoltage()(0, 0);
  const Real current = inductor->intfCurrent()(0, 0);
  return {voltage, current, voltage * current,
          0.5 * inductance * current * current};
}

void validateResistorAndSources() {
  constexpr Real voltage = 12.0;
  constexpr Real resistance = 4.0;

  auto node = EMT::SimNode::make("resistor_node", PhaseType::DC);
  node->setInitialVoltage(Complex(0.0, 0.0));

  auto voltageSource = EMT::DC::VoltageSource::make("resistor_source");
  voltageSource->setParameters(voltage);
  voltageSource->connect({EMT::SimNode::GND, node});

  auto resistor = EMT::DC::SSN::Resistor::make("resistor");
  resistor->setParameters(resistance);
  resistor->connect({EMT::SimNode::GND, node});

  SystemTopology system(0.0, SystemNodeList{node},
                        SystemComponentList{voltageSource, resistor});
  Simulation simulation("EMT_DC_R", Logger::Level::off);
  simulation.setSystem(system);
  simulation.setDomain(Domain::EMT);
  simulation.setTimeStep(1e-4);
  simulation.setFinalTime(1e-4);
  simulation.run();

  const Real current = resistor->intfCurrent()(0, 0);
  requireNear(current, voltage / resistance, 1e-12,
              "DC resistor steady-state current");
  requireNear(voltageSource->intfCurrent()(0, 0), -current, 1e-12,
              "DC voltage-source/load equal-opposite current");
  const Real sourcePower =
      voltageSource->intfVoltage()(0, 0) * voltageSource->intfCurrent()(0, 0);
  const Real loadPower =
      resistor->intfVoltage()(0, 0) * resistor->intfCurrent()(0, 0);
  requireNear(sourcePower + loadPower, 0.0, 1e-12,
              "DC voltage-source/load PSC power balance");
  if (sourcePower >= 0.0 || loadPower <= 0.0)
    throw std::runtime_error(
        "DC voltage-source/load PSC power signs are inconsistent.");

  auto reversedNode =
      EMT::SimNode::make("reversed_resistor_node", PhaseType::DC);
  auto reversedSource =
      EMT::DC::VoltageSource::make("reversed_resistor_source");
  reversedSource->setParameters(-voltage);
  reversedSource->connect({EMT::SimNode::GND, reversedNode});
  auto reversedResistor = EMT::DC::SSN::Resistor::make("reversed_resistor");
  reversedResistor->setParameters(resistance);
  reversedResistor->connect({EMT::SimNode::GND, reversedNode});

  SystemTopology reversedSystem(
      0.0, SystemNodeList{reversedNode},
      SystemComponentList{reversedSource, reversedResistor});
  Simulation reversedSimulation("EMT_DC_R_REVERSED", Logger::Level::off);
  reversedSimulation.setSystem(reversedSystem);
  reversedSimulation.setDomain(Domain::EMT);
  reversedSimulation.setTimeStep(1e-4);
  reversedSimulation.setFinalTime(1e-4);
  reversedSimulation.run();

  requireNear(reversedResistor->intfCurrent()(0, 0), -voltage / resistance,
              1e-12, "DC resistor current with reversed source polarity");
  requireNear(reversedSource->intfCurrent()(0, 0),
              -reversedResistor->intfCurrent()(0, 0), 1e-12,
              "Reversed DC source/load equal-opposite current");
  const Real reversedSourcePower =
      reversedSource->intfVoltage()(0, 0) * reversedSource->intfCurrent()(0, 0);
  const Real reversedLoadPower = reversedResistor->intfVoltage()(0, 0) *
                                 reversedResistor->intfCurrent()(0, 0);
  requireNear(reversedSourcePower + reversedLoadPower, 0.0, 1e-12,
              "Reversed DC source/load PSC power balance");
  if (reversedSourcePower >= 0.0 || reversedLoadPower <= 0.0)
    throw std::runtime_error(
        "Reversed DC source/load PSC power signs are inconsistent.");

  auto currentNode = EMT::SimNode::make("current_source_node", PhaseType::DC);
  auto currentSource = EMT::DC::CurrentSource::make("current_source");
  currentSource->setParameters(2.0);
  currentSource->connect({currentNode, EMT::SimNode::GND});
  auto currentLoad = EMT::DC::SSN::Resistor::make("current_source_load");
  currentLoad->setParameters(10.0);
  currentLoad->connect({EMT::SimNode::GND, currentNode});

  SystemTopology currentSystem(0.0, SystemNodeList{currentNode},
                               SystemComponentList{currentSource, currentLoad});
  Simulation currentSimulation("EMT_DC_I_SOURCE", Logger::Level::off);
  currentSimulation.setSystem(currentSystem);
  currentSimulation.setDomain(Domain::EMT);
  currentSimulation.setTimeStep(1e-4);
  currentSimulation.setFinalTime(1e-4);
  currentSimulation.run();
  requireNear(currentLoad->intfCurrent()(0, 0), 2.0, 1e-12,
              "DC ideal-current-source load current");
  requireNear(currentSource->intfCurrent()(0, 0),
              currentLoad->intfCurrent()(0, 0), 1e-12,
              "DC current-source/load equal-opposite branch injection");
  const Real currentSourcePower =
      currentSource->intfVoltage()(0, 0) * currentSource->intfCurrent()(0, 0);
  const Real currentLoadPower =
      currentLoad->intfVoltage()(0, 0) * currentLoad->intfCurrent()(0, 0);
  requireNear(currentSourcePower + currentLoadPower, 0.0, 1e-12,
              "DC current-source/load PSC power balance");
  if (currentSourcePower >= 0.0 || currentLoadPower <= 0.0)
    throw std::runtime_error(
        "DC current-source/load PSC power signs are inconsistent.");
  std::cout << "DC source PSC powers: voltage_source=" << sourcePower
            << ", reversed_voltage_source=" << reversedSourcePower
            << ", current_source=" << currentSourcePower << '\n';
}

void validateRC() {
  constexpr Real sourceVoltage = 10.0;
  constexpr Real resistance = 2.0;
  constexpr Real capacitance = 0.01;
  constexpr Real tau = resistance * capacitance;
  constexpr Real timeStep = tau / 1000.0;
  constexpr Real nominalTolerance = 2e-3;
  constexpr Real integrationTolerance = 2e-6;
  constexpr Real finalTolerance = 2e-6;

  const Sample atTau =
      runRC(tau, timeStep, sourceVoltage, resistance, capacitance, "tau");
  requireFinite(atTau, "RC time-constant sample");
  requireNear(atTau.voltage, sourceVoltage * (1.0 - std::exp(-1.0)),
              nominalTolerance,
              "RC capacitor voltage at nominal one time constant");
  requireNear(atTau.voltage,
              sourceVoltage * (1.0 - std::exp(-(tau - 0.5 * timeStep) / tau)),
              integrationTolerance,
              "RC capacitor voltage with half-step startup alignment");
  if (atTau.power < -1e-12 || atTau.energy < 0.0)
    throw std::runtime_error(
        "RC capacitor energy or PSC power has wrong sign.");
  std::cout << "RC PSC power=" << atTau.power
            << ", stored_energy=" << atTau.energy << '\n';

  const Real finalTime = 10.0 * tau;
  const Sample final = runRC(finalTime, timeStep, sourceVoltage, resistance,
                             capacitance, "final");
  requireFinite(final, "RC final sample");
  requireNear(final.voltage, sourceVoltage * (1.0 - std::exp(-finalTime / tau)),
              finalTolerance, "RC final capacitor voltage");
}

void validateRL() {
  constexpr Real sourceVoltage = 10.0;
  constexpr Real resistance = 2.0;
  constexpr Real inductance = 0.02;
  constexpr Real tau = inductance / resistance;
  constexpr Real timeStep = tau / 1000.0;
  constexpr Real nominalTolerance = 1e-3;
  constexpr Real integrationTolerance = 1e-6;
  constexpr Real finalTolerance = 1e-6;
  constexpr Real finalCurrent = sourceVoltage / resistance;

  const Sample atTau =
      runRL(tau, timeStep, sourceVoltage, resistance, inductance, "tau");
  requireFinite(atTau, "RL time-constant sample");
  requireNear(atTau.current, finalCurrent * (1.0 - std::exp(-1.0)),
              nominalTolerance,
              "RL inductor current at nominal one time constant");
  requireNear(atTau.current,
              finalCurrent * (1.0 - std::exp(-(tau - 0.5 * timeStep) / tau)),
              integrationTolerance,
              "RL inductor current with half-step startup alignment");
  if (atTau.power < -1e-12 || atTau.energy < 0.0)
    throw std::runtime_error("RL inductor energy or PSC power has wrong sign.");
  std::cout << "RL PSC power=" << atTau.power
            << ", stored_energy=" << atTau.energy << '\n';

  const Real finalTime = 10.0 * tau;
  const Sample final = runRL(finalTime, timeStep, sourceVoltage, resistance,
                             inductance, "final");
  requireFinite(final, "RL final sample");
  requireNear(final.current, finalCurrent * (1.0 - std::exp(-finalTime / tau)),
              finalTolerance, "RL final inductor current");
}

void validatePiLineSteadyState() {
  constexpr Real sourceVoltage = 20.0;
  constexpr Real seriesResistance = 0.5;
  constexpr Real seriesInductance = 0.01;
  constexpr Real parallelCapacitance = 1e-3;
  constexpr Real loadResistance = 10.0;
  constexpr Real expectedCurrent =
      sourceVoltage / (seriesResistance + loadResistance);

  auto supply = EMT::SimNode::make("pi_supply", PhaseType::DC);
  auto loadNode = EMT::SimNode::make("pi_load_node", PhaseType::DC);
  supply->setInitialVoltage(Complex(sourceVoltage, 0.0));
  loadNode->setInitialVoltage(Complex(loadResistance * expectedCurrent, 0.0));

  auto source = EMT::DC::VoltageSource::make("pi_source");
  source->setParameters(sourceVoltage);
  source->connect({EMT::SimNode::GND, supply});

  auto line = EMT::DC::SSN::PiLine::make("pi_line");
  line->setParameters(seriesResistance, seriesInductance, parallelCapacitance,
                      0.0, expectedCurrent);
  line->connect({loadNode, supply});

  auto load = EMT::DC::SSN::Resistor::make("pi_load");
  load->setParameters(loadResistance);
  load->connect({EMT::SimNode::GND, loadNode});

  SystemTopology system(0.0, SystemNodeList{supply, loadNode},
                        SystemComponentList{source, line, load});
  Simulation simulation("EMT_DC_PI", Logger::Level::off);
  simulation.setSystem(system);
  simulation.setDomain(Domain::EMT);
  simulation.setTimeStep(1e-5);
  simulation.setFinalTime(0.2);
  simulation.run();

  const Sample lineSample{line->intfVoltage()(0, 0), line->intfCurrent()(0, 0),
                          line->intfVoltage()(0, 0) * line->intfCurrent()(0, 0),
                          0.0};
  requireFinite(lineSample, "DC pi-line steady-state test");
  if (lineSample.voltage < 0.0 || lineSample.current < 0.0)
    throw std::runtime_error(
        "DC pi-line voltage or series-current orientation is inconsistent.");
  requireNear(lineSample.current, expectedCurrent, 1e-6,
              "DC pi-line analytical steady-state series current");
  requireNear(load->intfVoltage()(0, 0), loadResistance * expectedCurrent, 1e-6,
              "DC pi-line analytical steady-state load voltage");
  requireNear(lineSample.voltage, seriesResistance * expectedCurrent, 1e-6,
              "DC pi-line analytical steady-state line drop");
  requireNear(source->intfCurrent()(0, 0), -expectedCurrent, 1e-6,
              "DC pi-line source/series equal-opposite current");

  const Real powerBalance =
      source->intfVoltage()(0, 0) * source->intfCurrent()(0, 0) +
      lineSample.power + load->intfVoltage()(0, 0) * load->intfCurrent()(0, 0);
  requireNear(powerBalance, 0.0, 2e-5,
              "DC pi-line steady-state PSC power balance");
}

void validatePiLineTransient() {
  constexpr Real sourceVoltage = 20.0;
  constexpr Real feederResistance = 1.0;
  constexpr Real seriesResistance = 0.5;
  constexpr Real seriesInductance = 0.01;
  constexpr Real parallelCapacitance = 1e-3;
  constexpr Real loadResistance = 10.0;
  constexpr Real timeStep = 1e-5;
  constexpr Int stepCount = 20000;
  constexpr Int lateWindowCount = 1000;

  auto sourceNode = EMT::SimNode::make("pi_step_source", PhaseType::DC);
  auto sendingNode = EMT::SimNode::make("pi_step_sending", PhaseType::DC);
  auto receivingNode = EMT::SimNode::make("pi_step_receiving", PhaseType::DC);

  auto source = EMT::DC::VoltageSource::make("pi_step_voltage_source");
  source->setParameters(0.0);
  source->connect({EMT::SimNode::GND, sourceNode});

  auto feeder = EMT::DC::SSN::Resistor::make("pi_step_feeder");
  feeder->setParameters(feederResistance);
  feeder->connect({sendingNode, sourceNode});

  auto line = EMT::DC::SSN::PiLine::make("pi_step_line");
  line->setParameters(seriesResistance, seriesInductance, parallelCapacitance,
                      0.0, 0.0);
  line->connect({receivingNode, sendingNode});

  auto load = EMT::DC::SSN::Resistor::make("pi_step_load");
  load->setParameters(loadResistance);
  load->connect({EMT::SimNode::GND, receivingNode});

  SystemTopology system(0.0,
                        SystemNodeList{sourceNode, sendingNode, receivingNode},
                        SystemComponentList{source, feeder, line, load});
  Simulation simulation("EMT_DC_PI_STEP", Logger::Level::off);
  simulation.setSystem(system);
  simulation.setDomain(Domain::EMT);
  simulation.setTimeStep(timeStep);
  simulation.setFinalTime(stepCount * timeStep);
  simulation.start();

  const Real currentBeforeStep = line->intfCurrent()(0, 0);
  const Real sendingCapVoltageBeforeStep = sendingNode->voltage()(0, 0);
  const Real receivingCapVoltageBeforeStep = receivingNode->voltage()(0, 0);

  source->setParameters(sourceVoltage);

  requireNear(line->intfCurrent()(0, 0), currentBeforeStep, 0.0,
              "DC pi-line series-current continuity at source step");
  requireNear(sendingNode->voltage()(0, 0), sendingCapVoltageBeforeStep, 0.0,
              "DC pi-line sending-capacitor voltage continuity at source step");
  requireNear(
      receivingNode->voltage()(0, 0), receivingCapVoltageBeforeStep, 0.0,
      "DC pi-line receiving-capacitor voltage continuity at source step");

  Real lateCurrentMin = std::numeric_limits<Real>::infinity();
  Real lateCurrentMax = -std::numeric_limits<Real>::infinity();
  Real lateSendingVoltageMin = std::numeric_limits<Real>::infinity();
  Real lateSendingVoltageMax = -std::numeric_limits<Real>::infinity();
  Real lateReceivingVoltageMin = std::numeric_limits<Real>::infinity();
  Real lateReceivingVoltageMax = -std::numeric_limits<Real>::infinity();

  for (Int step = 0; step < stepCount; ++step) {
    simulation.step();
    const Real current = line->intfCurrent()(0, 0);
    const Real sendingCapVoltage = sendingNode->voltage()(0, 0);
    const Real receivingCapVoltage = receivingNode->voltage()(0, 0);
    const Sample sample{sendingCapVoltage - receivingCapVoltage, current,
                        (sendingCapVoltage - receivingCapVoltage) * current,
                        0.0};
    requireFinite(sample, "DC pi-line transient sample");

    if (step >= stepCount - lateWindowCount) {
      lateCurrentMin = std::min(lateCurrentMin, current);
      lateCurrentMax = std::max(lateCurrentMax, current);
      lateSendingVoltageMin =
          std::min(lateSendingVoltageMin, sendingCapVoltage);
      lateSendingVoltageMax =
          std::max(lateSendingVoltageMax, sendingCapVoltage);
      lateReceivingVoltageMin =
          std::min(lateReceivingVoltageMin, receivingCapVoltage);
      lateReceivingVoltageMax =
          std::max(lateReceivingVoltageMax, receivingCapVoltage);
    }
  }
  simulation.stop();

  constexpr Real expectedCurrent =
      sourceVoltage / (feederResistance + seriesResistance + loadResistance);
  constexpr Real expectedSendingVoltage =
      sourceVoltage - feederResistance * expectedCurrent;
  constexpr Real expectedReceivingVoltage = loadResistance * expectedCurrent;
  requireNear(line->intfCurrent()(0, 0), expectedCurrent, 1e-6,
              "DC pi-line step final series current");
  requireNear(sendingNode->voltage()(0, 0), expectedSendingVoltage, 1e-6,
              "DC pi-line step final sending-capacitor voltage");
  requireNear(receivingNode->voltage()(0, 0), expectedReceivingVoltage, 1e-6,
              "DC pi-line step final receiving-capacitor voltage");

  requireNear(lateCurrentMax - lateCurrentMin, 0.0, 1e-7,
              "DC pi-line late-window current drift/oscillation");
  requireNear(lateSendingVoltageMax - lateSendingVoltageMin, 0.0, 1e-7,
              "DC pi-line late-window sending-voltage drift/oscillation");
  requireNear(lateReceivingVoltageMax - lateReceivingVoltageMin, 0.0, 1e-7,
              "DC pi-line late-window receiving-voltage drift/oscillation");
}

} // namespace

int main() {
  try {
    validateInvalidParameters();
    validateResistorAndSources();
    validateRC();
    validateRL();
    validatePiLineSteadyState();
    validatePiLineTransient();
    std::cout << "All scalar DC SSN validations passed.\n";
    return 0;
  } catch (const std::exception &exception) {
    std::cerr << "DC SSN validation failed: " << exception.what() << '\n';
    return 1;
  }
}
