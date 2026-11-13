/* SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
 * SPDX-License-Identifier: MPL-2.0
 */

#include <DPsim.h>
#include <dpsim-models/EMT/EMT_Ph1_PiLine.h>
#include <dpsim-models/EMT/EMT_Ph1_VoltageSourceRamp.h>
#include <dpsim-models/EMT/EMT_Ph3_AvVoltageSourceInverterDQ.h>
#include <dpsim-models/EMT/EMT_Ph3_HalfDecouplingLine.h>
#include <dpsim-models/EMT/EMT_Ph3_SynchronGeneratorIdeal.h>
#include <dpsim-models/EMT/EMT_Ph3_SynchronGeneratorTrStab.h>
#include <dpsim-models/EMT/EMT_Ph3_VSIVoltageControlVCO.h>
#include <dpsim-models/SP/SP_Ph1_NetworkInjection.h>
#include <dpsim-models/SP/SP_Ph1_PiLine.h>
#include <dpsim-models/SP/SP_Ph1_SynchronGenerator.h>

#include <chrono>

using namespace DPsim;
using namespace CPS::EMT;
using namespace CPS::EMT::Ph3;

static Real runCase(const String &suffix, Bool useSubcomponents) {
  constexpr Real timeStep = 50e-6;
  constexpr Real finalTime = 0.5;
  const String simName = "EMT_Ph3_RxLine_" + suffix;

  const std::vector<Complex> initialVoltage{
      Complex(100e3, 0), Complex(100e3, 0), Complex(100e3, 0)};
  auto sourceNode = SimNode::make("source", PhaseType::ABC, initialVoltage);
  auto loadNode = SimNode::make("load", PhaseType::ABC, initialVoltage);

  auto source = VoltageSource::make("source_voltage");
  source->setParameters(CPS::Math::singlePhaseVariableToThreePhase(
                            Complex(100e3 * PEAK1PH_TO_RMS3PH, 0)),
                        50.);
  auto line = RxLine::make("line", useSubcomponents);
  line->setParameters(Matrix::Identity(3, 3) * 5.,
                      Matrix::Identity(3, 3) * 0.16);
  auto load = Resistor::make("load_resistor");
  load->setParameters(Matrix::Identity(3, 3) * 10000.);

  source->connect({SimNode::GND, sourceNode});
  line->connect({sourceNode, loadNode});
  load->connect({loadNode, SimNode::GND});

  SystemTopology system(50., {sourceNode, loadNode}, {source, line, load});
  auto logger = DataLogger::make(simName);
  logger->logAttribute("v_source", sourceNode->attribute("v"));
  logger->logAttribute("v_load", loadNode->attribute("v"));
  logger->logAttribute("line_voltage", line->attribute("v_intf"));
  logger->logAttribute("line_current", line->attribute("i_intf"));

  Simulation simulation(simName, Logger::Level::off);
  simulation.setSystem(system);
  simulation.setDomain(Domain::EMT);
  simulation.setTimeStep(timeStep);
  simulation.setFinalTime(finalTime);
  simulation.addLogger(logger);

  const auto start = std::chrono::steady_clock::now();
  simulation.run();
  return std::chrono::duration<Real>(std::chrono::steady_clock::now() - start)
      .count();
}

static Real runAvInverter(const String &suffix, Bool useSubcomponents) {
  constexpr Real timeStep = 50e-6;
  constexpr Real finalTime = 0.5;
  const String simName = "EMT_AvVSI_Direct_" + suffix;
  const std::vector<Complex> initialVoltage{
      Complex(100e3, 0), Complex(100e3, 0), Complex(100e3, 0)};
  auto gridNode = SimNode::make("grid", PhaseType::ABC, initialVoltage);
  auto pccNode = SimNode::make("pcc", PhaseType::ABC, initialVoltage);
  auto source = VoltageSource::make("grid_source");
  source->setParameters(CPS::Math::singlePhaseVariableToThreePhase(
                            Complex(100e3 * PEAK1PH_TO_RMS3PH, 0)),
                        50.);
  auto line = Resistor::make("grid_resistance");
  line->setParameters(Matrix::Identity(3, 3) * 5.);
  auto inverter = AvVoltageSourceInverterDQ::make("inverter", useSubcomponents);
  inverter->setParameters(2. * PI * 50., 100e3, 0., 0.);
  inverter->setControllerParameters(0.25, 2., 0.001, 0.08, 3.77, 1400.,
                                    2. * PI * 50.);
  inverter->setFilterParameters(0.16, 10e-6, 5., 100.);
  inverter->withControl(false);
  source->connect({SimNode::GND, gridNode});
  line->connect({gridNode, pccNode});
  inverter->connect({pccNode});
  SystemTopology system(50., {gridNode, pccNode}, {source, line, inverter});
  const String logName = "logs/" + simName;
  Logger::setLogDir(logName);
  auto logger = DataLogger::make(simName);
  logger->logAttribute("pcc_voltage", pccNode->attribute("v"));
  logger->logAttribute("inverter_voltage", inverter->attribute("v_intf"));
  logger->logAttribute("inverter_current", inverter->attribute("i_intf"));
  logger->logAttribute("source_voltage", inverter->mVs);
  Simulation simulation(simName, Logger::Level::off);
  simulation.setSystem(system);
  simulation.setDomain(Domain::EMT);
  simulation.setTimeStep(timeStep);
  simulation.setFinalTime(finalTime);
  simulation.addLogger(logger);
  const auto start = std::chrono::steady_clock::now();
  simulation.run();
  return std::chrono::duration<Real>(std::chrono::steady_clock::now() - start)
      .count();
}

static Real runVcoInverter(const String &suffix, Bool useSubcomponents) {
  constexpr Real timeStep = 50e-6;
  constexpr Real finalTime = 0.5;
  const String simName = "EMT_VCO_VSI_Direct_" + suffix;
  const std::vector<Complex> initialVoltage{
      Complex(100e3, 0), Complex(100e3, 0), Complex(100e3, 0)};
  auto gridNode = SimNode::make("grid", PhaseType::ABC, initialVoltage);
  auto pccNode = SimNode::make("pcc", PhaseType::ABC, initialVoltage);
  auto source = VoltageSource::make("grid_source");
  source->setParameters(CPS::Math::singlePhaseVariableToThreePhase(
                            Complex(100e3 * PEAK1PH_TO_RMS3PH, 0)),
                        50.);
  auto line = Resistor::make("grid_resistance");
  line->setParameters(Matrix::Identity(3, 3) * 5.);
  auto inverter = VSIVoltageControlVCO::make("inverter", useSubcomponents);
  inverter->setParameters(2. * PI * 50., 100e3, 0.);
  inverter->setControllerParameters(0.001, 0.08, 3.77, 1400., 2. * PI * 50.);
  inverter->setFilterParameters(0.16, 10e-6, 5., 100.);
  inverter->withControl(false);
  source->connect({SimNode::GND, gridNode});
  line->connect({gridNode, pccNode});
  inverter->connect({pccNode});
  SystemTopology system(50., {gridNode, pccNode}, {source, line, inverter});
  Logger::setLogDir("logs/" + simName);
  auto logger = DataLogger::make(simName);
  logger->logAttribute("pcc_voltage", pccNode->attribute("v"));
  logger->logAttribute("inverter_voltage", inverter->attribute("v_intf"));
  logger->logAttribute("inverter_current", inverter->attribute("i_intf"));
  logger->logAttribute("source_voltage", inverter->mVs);
  Simulation simulation(simName, Logger::Level::off);
  simulation.setSystem(system);
  simulation.setDomain(Domain::EMT);
  simulation.setTimeStep(timeStep);
  simulation.setFinalTime(finalTime);
  simulation.addLogger(logger);
  const auto start = std::chrono::steady_clock::now();
  simulation.run();
  return std::chrono::duration<Real>(std::chrono::steady_clock::now() - start)
      .count();
}

static Real runInjection(const String &suffix, Bool useSubcomponents) {
  constexpr Real timeStep = 50e-6;
  constexpr Real finalTime = 0.5;
  const String simName = "EMT_Ph3_NetworkInjection_" + suffix;
  const std::vector<Complex> initialVoltage{
      Complex(100e3, 0), Complex(100e3, 0), Complex(100e3, 0)};
  auto bus = SimNode::make("bus", PhaseType::ABC, initialVoltage);
  auto source = NetworkInjection::make("grid_source", useSubcomponents);
  source->setParameters(CPS::Math::singlePhaseVariableToThreePhase(
                            Complex(100e3 * PEAK1PH_TO_RMS3PH, 0)),
                        50.);
  auto load = Resistor::make("load_resistor");
  load->setParameters(Matrix::Identity(3, 3) * 10000.);
  source->connect({bus});
  load->connect({bus, SimNode::GND});

  SystemTopology system(50., {bus}, {source, load});
  auto logger = DataLogger::make(simName);
  logger->logAttribute("bus_voltage", bus->attribute("v"));
  logger->logAttribute("source_voltage", source->attribute("v_intf"));
  logger->logAttribute("source_current", source->attribute("i_intf"));

  Simulation simulation(simName, Logger::Level::off);
  simulation.setSystem(system);
  simulation.setDomain(Domain::EMT);
  simulation.setTimeStep(timeStep);
  simulation.setFinalTime(finalTime);
  simulation.addLogger(logger);
  const auto start = std::chrono::steady_clock::now();
  simulation.run();
  return std::chrono::duration<Real>(std::chrono::steady_clock::now() - start)
      .count();
}

static Real runShunt(const String &suffix, Bool useSubcomponents) {
  constexpr Real timeStep = 50e-6;
  constexpr Real finalTime = 0.5;
  const String simName = "EMT_Ph3_Shunt_" + suffix;
  const std::vector<Complex> initialVoltage{
      Complex(100e3, 0), Complex(100e3, 0), Complex(100e3, 0)};
  auto bus = SimNode::make("bus", PhaseType::ABC, initialVoltage);
  auto source = NetworkInjection::make("grid_source", false);
  source->setParameters(CPS::Math::singlePhaseVariableToThreePhase(
                            Complex(100e3 * PEAK1PH_TO_RMS3PH, 0)),
                        50.);
  auto shunt = Shunt::make("shunt", useSubcomponents);
  shunt->setParameters(1e-6, 1e-4);
  source->connect({bus});
  shunt->connect({bus});

  SystemTopology system(50., {bus}, {source, shunt});
  auto logger = DataLogger::make(simName);
  logger->logAttribute("bus_voltage", bus->attribute("v"));
  logger->logAttribute("shunt_voltage", shunt->attribute("v_intf"));
  logger->logAttribute("shunt_current", shunt->attribute("i_intf"));

  Simulation simulation(simName, Logger::Level::off);
  simulation.setSystem(system);
  simulation.setDomain(Domain::EMT);
  simulation.setTimeStep(timeStep);
  simulation.setFinalTime(finalTime);
  simulation.addLogger(logger);
  const auto start = std::chrono::steady_clock::now();
  simulation.run();
  return std::chrono::duration<Real>(std::chrono::steady_clock::now() - start)
      .count();
}

static Real runPiLine(const String &suffix, Bool useSubcomponents) {
  constexpr Real timeStep = 50e-6;
  constexpr Real finalTime = 0.5;
  const String simName = "EMT_Ph3_PiLine_" + suffix;
  const std::vector<Complex> initialVoltage{
      Complex(100e3, 0), Complex(100e3, 0), Complex(100e3, 0)};
  auto sourceNode = SimNode::make("source", PhaseType::ABC, initialVoltage);
  auto loadNode = SimNode::make("load", PhaseType::ABC, initialVoltage);
  auto source = VoltageSource::make("source_voltage");
  source->setParameters(CPS::Math::singlePhaseVariableToThreePhase(
                            Complex(100e3 * PEAK1PH_TO_RMS3PH, 0)),
                        50.);
  auto line = PiLine::make("line", useSubcomponents);
  line->setParameters(
      Matrix::Identity(3, 3) * 5., Matrix::Identity(3, 3) * 0.16,
      Matrix::Identity(3, 3) * 10e-6, Matrix::Identity(3, 3) * 1e-6);
  auto load = Resistor::make("load_resistor");
  load->setParameters(Matrix::Identity(3, 3) * 10000.);
  source->connect({SimNode::GND, sourceNode});
  line->connect({sourceNode, loadNode});
  load->connect({loadNode, SimNode::GND});
  SystemTopology system(50., {sourceNode, loadNode}, {source, line, load});
  auto logger = DataLogger::make(simName);
  logger->logAttribute("load_voltage", loadNode->attribute("v"));
  logger->logAttribute("line_voltage", line->attribute("v_intf"));
  logger->logAttribute("line_current", line->attribute("i_intf"));
  Simulation simulation(simName, Logger::Level::off);
  simulation.setSystem(system);
  simulation.setDomain(Domain::EMT);
  simulation.setTimeStep(timeStep);
  simulation.setFinalTime(finalTime);
  simulation.addLogger(logger);
  const auto start = std::chrono::steady_clock::now();
  simulation.run();
  return std::chrono::duration<Real>(std::chrono::steady_clock::now() - start)
      .count();
}

static Real runRXLoad(const String &suffix, Bool useSubcomponents) {
  constexpr Real timeStep = 50e-6;
  constexpr Real finalTime = 0.5;
  const String simName = "EMT_Ph3_RXLoad_" + suffix;
  const std::vector<Complex> initialVoltage{
      Complex(100e3, 0), Complex(100e3, 0), Complex(100e3, 0)};
  auto bus = SimNode::make("bus", PhaseType::ABC, initialVoltage);
  auto source = NetworkInjection::make("grid_source", false);
  source->setParameters(CPS::Math::singlePhaseVariableToThreePhase(
                            Complex(100e3 * PEAK1PH_TO_RMS3PH, 0)),
                        50.);
  auto load = RXLoad::make("rx_load", useSubcomponents);
  load->setParameters(Matrix::Identity(3, 3) * 1e6,
                      Matrix::Identity(3, 3) * 0.2e6, 100e3);
  source->connect({bus});
  load->connect({bus});
  SystemTopology system(50., {bus}, {source, load});
  auto logger = DataLogger::make(simName);
  logger->logAttribute("bus_voltage", bus->attribute("v"));
  logger->logAttribute("load_voltage", load->attribute("v_intf"));
  logger->logAttribute("load_current", load->attribute("i_intf"));
  Simulation simulation(simName, Logger::Level::off);
  simulation.setSystem(system);
  simulation.setDomain(Domain::EMT);
  simulation.setTimeStep(timeStep);
  simulation.setFinalTime(finalTime);
  simulation.addLogger(logger);
  const auto start = std::chrono::steady_clock::now();
  simulation.run();
  return std::chrono::duration<Real>(std::chrono::steady_clock::now() - start)
      .count();
}

static Real runTransformer(const String &suffix, Bool useSubcomponents,
                           Bool withResistiveLosses = false) {
  constexpr Real timeStep = 50e-6;
  constexpr Real finalTime = 0.5;
  constexpr Real highVoltage = 100e3;
  constexpr Real lowVoltage = 10e3;
  constexpr Real ratio = highVoltage / lowVoltage;
  const String simName = "EMT_Ph3_Transformer_" + suffix;
  const std::vector<Complex> highInitial{Complex(highVoltage, 0),
                                         Complex(highVoltage, 0),
                                         Complex(highVoltage, 0)};
  const std::vector<Complex> lowInitial{
      Complex(lowVoltage, 0), Complex(lowVoltage, 0), Complex(lowVoltage, 0)};
  auto highNode = SimNode::make("high", PhaseType::ABC, highInitial);
  auto lowNode = SimNode::make("low", PhaseType::ABC, lowInitial);
  auto source = VoltageSource::make("source_voltage");
  source->setParameters(CPS::Math::singlePhaseVariableToThreePhase(
                            Complex(highVoltage * PEAK1PH_TO_RMS3PH, 0)),
                        50.);
  auto transformer = Transformer::make("transformer", useSubcomponents,
                                       Logger::Level::off, withResistiveLosses);
  transformer->setParameters(highVoltage, lowVoltage, 1e6, ratio, 0.,
                             CPS::Math::singlePhaseParameterToThreePhase(1.),
                             CPS::Math::singlePhaseParameterToThreePhase(0.1));
  auto load = Resistor::make("load_resistor");
  load->setParameters(CPS::Math::singlePhaseParameterToThreePhase(100.));
  source->connect({SimNode::GND, highNode});
  transformer->connect({highNode, lowNode});
  load->connect({lowNode, SimNode::GND});
  SystemTopology system(50., {highNode, lowNode}, {source, transformer, load});
  auto logger = DataLogger::make(simName);
  logger->logAttribute("high_voltage", highNode->attribute("v"));
  logger->logAttribute("low_voltage", lowNode->attribute("v"));
  logger->logAttribute("transformer_current", transformer->attribute("i_intf"));
  Simulation simulation(simName, Logger::Level::off);
  simulation.setSystem(system);
  simulation.setDomain(Domain::EMT);
  simulation.setTimeStep(timeStep);
  simulation.setFinalTime(finalTime);
  simulation.addLogger(logger);
  const auto start = std::chrono::steady_clock::now();
  simulation.run();
  return std::chrono::duration<Real>(std::chrono::steady_clock::now() - start)
      .count();
}

static Real runHalfDecouplingLine(const String &suffix, Bool useSubcomponents) {
  constexpr Real timeStep = 50e-6;
  constexpr Real finalTime = 0.5;
  const String simName = "EMT_Ph3_HalfDecouplingLine_" + suffix;
  const std::vector<Complex> voltage1{Complex(100e3, 0), Complex(100e3, 0),
                                      Complex(100e3, 0)};
  const std::vector<Complex> voltage2{Complex(95e3, 0), Complex(95e3, 0),
                                      Complex(95e3, 0)};
  auto node1 = SimNode::make("node1", PhaseType::ABC, voltage1);
  auto node2 = SimNode::make("node2", PhaseType::ABC, voltage2);
  auto source1 = VoltageSource::make("source1");
  source1->setParameters(CPS::Math::singlePhaseVariableToThreePhase(
                             Complex(100e3 * PEAK1PH_TO_RMS3PH, 0)),
                         50.);
  auto source2 = VoltageSource::make("source2");
  source2->setParameters(CPS::Math::singlePhaseVariableToThreePhase(
                             Complex(95e3 * PEAK1PH_TO_RMS3PH, 0)),
                         50.);
  auto end1 = CPS::EMT::Ph3::HalfDecouplingLine::make("end1", useSubcomponents);
  auto end2 = CPS::EMT::Ph3::HalfDecouplingLine::make("end2", useSubcomponents);
  const Matrix resistance = Matrix::Identity(3, 3) * 0.5;
  const Matrix inductance = Matrix::Identity(3, 3) * 0.01;
  const Matrix capacitance = Matrix::Identity(3, 3) * 1e-4;
  end1->setParameters(resistance, inductance, capacitance);
  end2->setParameters(resistance, inductance, capacitance);
  end1->setCouplingSource(end2->mSendingVolt, end2->mSendingCur);
  end2->setCouplingSource(end1->mSendingVolt, end1->mSendingCur);
  source1->connect({SimNode::GND, node1});
  source2->connect({SimNode::GND, node2});
  end1->connect({node1});
  end2->connect({node2});
  SystemTopology system(50., {node1, node2}, {source1, source2, end1, end2});
  auto logger = DataLogger::make(simName);
  logger->logAttribute("node1_voltage", node1->attribute("v"));
  logger->logAttribute("node2_voltage", node2->attribute("v"));
  logger->logAttribute("end1_current", end1->attribute("i_intf"));
  logger->logAttribute("end2_current", end2->attribute("i_intf"));
  Simulation simulation(simName, Logger::Level::off);
  simulation.setSystem(system);
  simulation.setDomain(Domain::EMT);
  simulation.setTimeStep(timeStep);
  simulation.setFinalTime(finalTime);
  simulation.addLogger(logger);
  const auto start = std::chrono::steady_clock::now();
  simulation.run();
  return std::chrono::duration<Real>(std::chrono::steady_clock::now() - start)
      .count();
}

static Real runDCPiLine(const String &suffix, Bool useSubcomponents) {
  constexpr Real timeStep = 10e-6;
  constexpr Real finalTime = 0.5;
  constexpr Real sourceStepTime = 0.01;
  constexpr Real initialVoltage = 20e3;
  constexpr Real finalVoltage = 22e3;
  constexpr Real feederResistance = 0.2;
  constexpr Real lineResistance = 0.5;
  constexpr Real lineInductance = 20e-3;
  constexpr Real lineCapacitance = 2e-3;
  constexpr Real loadResistance = 20.;
  const Real initialCurrent =
      initialVoltage / (feederResistance + lineResistance + loadResistance);
  const String simName = "EMT_DC_PiLine_" + suffix;

  auto sourceNode = SimNode::make("source", PhaseType::DC);
  auto sendingNode = SimNode::make("sending", PhaseType::DC);
  auto receivingNode = SimNode::make("receiving", PhaseType::DC);
  sourceNode->setInitialVoltage(Complex(initialVoltage, 0.));
  sendingNode->setInitialVoltage(
      Complex(initialVoltage - feederResistance * initialCurrent, 0.));
  receivingNode->setInitialVoltage(
      Complex(loadResistance * initialCurrent, 0.));

  auto source = CPS::EMT::DC::VoltageSource::make("dc_source");
  source->setParameters(initialVoltage);
  source->connect({SimNode::GND, sourceNode});
  auto feeder = CPS::EMT::DC::Resistor::make("feeder");
  feeder->setParameters(feederResistance);
  feeder->connect({sendingNode, sourceNode});
  auto line = CPS::EMT::DC::PiLine::make("line", useSubcomponents);
  line->setParameters(lineResistance, lineInductance, lineCapacitance, 0.,
                      initialCurrent);
  line->connect({receivingNode, sendingNode});
  auto load = CPS::EMT::DC::Resistor::make("load");
  load->setParameters(loadResistance);
  load->connect({SimNode::GND, receivingNode});

  SystemTopology system(0., {sourceNode, sendingNode, receivingNode},
                        {source, feeder, line, load});
  auto logger = DataLogger::make(simName);
  logger->logAttribute("sending_voltage", sendingNode->attribute("v"));
  logger->logAttribute("receiving_voltage", receivingNode->attribute("v"));
  logger->logAttribute("line_voltage", line->attribute("v_intf"));
  logger->logAttribute("line_current", line->attribute("i_intf"));

  Simulation simulation(simName, Logger::Level::off);
  simulation.setSystem(system);
  simulation.setDomain(Domain::EMT);
  simulation.setTimeStep(timeStep);
  simulation.setFinalTime(finalTime);
  simulation.addLogger(logger);
  simulation.initialize();
  const auto start = std::chrono::steady_clock::now();
  simulation.start();
  Bool sourceChanged = false;
  while (simulation.time() < simulation.finalTime()) {
    if (!sourceChanged &&
        simulation.time() >= sourceStepTime - 0.5 * timeStep) {
      source->setParameters(finalVoltage);
      sourceChanged = true;
    }
    simulation.step();
  }
  simulation.stop();
  return std::chrono::duration<Real>(std::chrono::steady_clock::now() - start)
      .count();
}

static Real runPh1PiLine(const String &suffix, Bool useSubcomponents) {
  constexpr Real timeStep = 50e-6;
  constexpr Real finalTime = 0.5;
  const String simName = "EMT_Ph1_PiLine_" + suffix;
  auto sourceNode = SimNode::make("source", PhaseType::Single);
  auto loadNode = SimNode::make("load", PhaseType::Single);
  sourceNode->setInitialVoltage(Complex(100e3, 0.));
  loadNode->setInitialVoltage(Complex(100e3, 0.));

  auto source = CPS::EMT::Ph1::VoltageSource::make("source_voltage");
  source->setParameters(Complex(100e3, 0.), 50.);
  auto line = CPS::EMT::Ph1::PiLine::make("line", useSubcomponents);
  line->setParameters(5., 0.16, 10e-6, 1e-6);
  auto load = CPS::EMT::Ph1::Resistor::make("load_resistor");
  load->setParameters(10000.);
  source->connect({SimNode::GND, sourceNode});
  line->connect({sourceNode, loadNode});
  load->connect({loadNode, SimNode::GND});

  SystemTopology system(50., {sourceNode, loadNode}, {source, line, load});
  auto logger = DataLogger::make(simName);
  logger->logAttribute("source_voltage", sourceNode->attribute("v"));
  logger->logAttribute("load_voltage", loadNode->attribute("v"));
  logger->logAttribute("line_voltage", line->attribute("v_intf"));
  logger->logAttribute("line_current", line->attribute("i_intf"));
  Simulation simulation(simName, Logger::Level::off);
  simulation.setSystem(system);
  simulation.setDomain(Domain::EMT);
  simulation.setTimeStep(timeStep);
  simulation.setFinalTime(finalTime);
  simulation.addLogger(logger);
  const auto start = std::chrono::steady_clock::now();
  simulation.run();
  return std::chrono::duration<Real>(std::chrono::steady_clock::now() - start)
      .count();
}

static Real runVoltageSourceRamp(const String &suffix, Bool useSubcomponents) {
  constexpr Real timeStep = 50e-6;
  constexpr Real finalTime = 0.5;
  const String simName = "EMT_Ph1_VoltageSourceRamp_" + suffix;
  auto bus = SimNode::make("bus", PhaseType::Single);
  bus->setInitialVoltage(Complex(100e3, 0.));
  auto source =
      CPS::EMT::Ph1::VoltageSourceRamp::make("source", useSubcomponents);
  source->setParameters(Complex(100e3, 0.), Complex(2e3, 1e3), 2. * PI * 50.,
                        2. * PI * 2., 0.01, 0.02);
  auto load = CPS::EMT::Ph1::Resistor::make("load_resistor");
  load->setParameters(10000.);
  source->connect({SimNode::GND, bus});
  load->connect({bus, SimNode::GND});
  SystemTopology system(50., {bus}, {source, load});
  auto logger = DataLogger::make(simName);
  logger->logAttribute("bus_voltage", bus->attribute("v"));
  logger->logAttribute("source_voltage", source->attribute("v_intf"));
  logger->logAttribute("source_current", source->attribute("i_intf"));
  Simulation simulation(simName, Logger::Level::off);
  simulation.setSystem(system);
  simulation.setDomain(Domain::EMT);
  simulation.setTimeStep(timeStep);
  simulation.setFinalTime(finalTime);
  simulation.addLogger(logger);
  const auto start = std::chrono::steady_clock::now();
  simulation.run();
  return std::chrono::duration<Real>(std::chrono::steady_clock::now() - start)
      .count();
}

static Real runIdealGenerator(const String &suffix, Bool useSubcomponents) {
  constexpr Real timeStep = 50e-6;
  constexpr Real finalTime = 0.5;
  const String simName = "EMT_Ph3_SynchronGeneratorIdeal_" + suffix;
  const std::vector<Complex> initialVoltage{
      Complex(100e3, 0), Complex(100e3, 0), Complex(100e3, 0)};
  auto bus = SimNode::make("bus", PhaseType::ABC, initialVoltage);
  auto generator = SynchronGeneratorIdeal::make("generator", useSubcomponents);
  auto load = Resistor::make("load_resistor");
  load->setParameters(Matrix::Identity(3, 3) * 10000.);
  generator->connect({bus});
  load->connect({bus, SimNode::GND});
  SystemTopology system(50., {bus}, {generator, load});
  auto logger = DataLogger::make(simName);
  logger->logAttribute("bus_voltage", bus->attribute("v"));
  logger->logAttribute("generator_voltage", generator->attribute("v_intf"));
  logger->logAttribute("generator_current", generator->attribute("i_intf"));
  Simulation simulation(simName, Logger::Level::off);
  simulation.setSystem(system);
  simulation.setDomain(Domain::EMT);
  simulation.setTimeStep(timeStep);
  simulation.setFinalTime(finalTime);
  simulation.addLogger(logger);
  const auto start = std::chrono::steady_clock::now();
  simulation.run();
  return std::chrono::duration<Real>(std::chrono::steady_clock::now() - start)
      .count();
}

static Real runTransientGenerator(const String &suffix, Bool useSubcomponents) {
  constexpr Real timeStep = 50e-6;
  constexpr Real finalTime = 0.5;
  constexpr Real voltageBase = 230e3;
  constexpr Real nominalPower = 500e6;
  constexpr Real machineVoltage = 22e3;
  constexpr Real frequency = 60.;
  constexpr Real reactance = 0.31;
  constexpr Real lineResistance = 6.7;
  constexpr Real lineInductance = 47. / (2. * PI * frequency);
  constexpr Real transformerRatio = voltageBase / machineVoltage;
  const String simName = "EMT_Ph3_SynchronGeneratorTrStab_" + suffix;

  auto pfGeneratorNode =
      CPS::SimNode<Complex>::make("generator_pf", PhaseType::Single);
  auto pfSlackNode = CPS::SimNode<Complex>::make("slack_pf", PhaseType::Single);
  auto pfGenerator = CPS::SP::Ph1::SynchronGenerator::make("generator_pf");
  pfGenerator->setParameters(nominalPower, machineVoltage, 300e6,
                             machineVoltage * transformerRatio,
                             CPS::PowerflowBusType::PV);
  pfGenerator->setBaseVoltage(voltageBase);
  pfGenerator->modifyPowerFlowBusType(CPS::PowerflowBusType::PV);
  auto pfLine = CPS::SP::Ph1::PiLine::make("line_pf");
  pfLine->setParameters(lineResistance, lineInductance, 0., 0.);
  pfLine->setBaseVoltage(voltageBase);
  auto pfSlack = CPS::SP::Ph1::NetworkInjection::make("slack_pf");
  pfSlack->setParameters(voltageBase);
  pfSlack->setBaseVoltage(voltageBase);
  pfSlack->modifyPowerFlowBusType(CPS::PowerflowBusType::VD);
  pfGenerator->connect({pfGeneratorNode});
  pfLine->connect({pfGeneratorNode, pfSlackNode});
  pfSlack->connect({pfSlackNode});
  SystemTopology pfSystem(frequency, {pfGeneratorNode, pfSlackNode},
                          {pfGenerator, pfLine, pfSlack});
  Simulation pfSimulation(simName + "_PF", Logger::Level::off);
  pfSimulation.setSystem(pfSystem);
  pfSimulation.setTimeStep(finalTime);
  pfSimulation.setFinalTime(2. * finalTime);
  pfSimulation.setDomain(Domain::SP);
  pfSimulation.setSolverType(Solver::Type::NRP);
  pfSimulation.setSolverAndComponentBehaviour(
      Solver::Behaviour::Initialization);
  pfSimulation.doInitFromNodesAndTerminals(false);
  pfSimulation.run();

  auto generatorNode = SimNode::make("generator", PhaseType::ABC);
  auto slackNode = SimNode::make("slack", PhaseType::ABC);
  auto generator = SynchronGeneratorTrStab::make("generator", useSubcomponents,
                                                 Logger::Level::off);
  generator->setStandardParametersPU(
      nominalPower, machineVoltage, frequency,
      reactance * transformerRatio * transformerRatio, 5., 0., 50.);
  auto line = PiLine::make("line");
  line->setParameters(Matrix::Identity(3, 3) * lineResistance,
                      Matrix::Identity(3, 3) * lineInductance,
                      Matrix::Zero(3, 3), Matrix::Zero(3, 3));
  auto slack = NetworkInjection::make("slack");
  generator->connect({generatorNode});
  line->connect({generatorNode, slackNode});
  slack->connect({slackNode});
  SystemTopology system(frequency, {generatorNode, slackNode},
                        {generator, line, slack});
  system.initWithPowerflow(pfSystem, Domain::EMT);
  auto logger = DataLogger::make(simName);
  logger->logAttribute("generator_voltage", generatorNode->attribute("v"));
  logger->logAttribute("generator_current", generator->attribute("i_intf"));
  logger->logAttribute("generator_emf", generator->attribute("Ep"));
  logger->logAttribute("slack_voltage", slackNode->attribute("v"));
  Simulation simulation(simName, Logger::Level::off);
  simulation.setSystem(system);
  simulation.setDomain(Domain::EMT);
  simulation.setTimeStep(timeStep);
  simulation.setFinalTime(finalTime);
  simulation.addLogger(logger);
  const auto start = std::chrono::steady_clock::now();
  simulation.run();
  return std::chrono::duration<Real>(std::chrono::steady_clock::now() - start)
      .count();
}

int main() {
  const Real legacyTime = runCase("Legacy", true);
  const Real directTime = runCase("Direct", false);
  fmt::print("legacy_wall_s={:.9f}\ndirect_wall_s={:.9f}\n", legacyTime,
             directTime);
  const Real legacySourceTime = runInjection("Legacy", true);
  const Real directSourceTime = runInjection("Direct", false);
  fmt::print("source_legacy_wall_s={:.9f}\nsource_direct_wall_s={:.9f}\n",
             legacySourceTime, directSourceTime);
  const Real legacyShuntTime = runShunt("Legacy", true);
  const Real directShuntTime = runShunt("Direct", false);
  fmt::print("shunt_legacy_wall_s={:.9f}\nshunt_direct_wall_s={:.9f}\n",
             legacyShuntTime, directShuntTime);
  const Real legacyPiLineTime = runPiLine("Legacy", true);
  const Real directPiLineTime = runPiLine("Direct", false);
  fmt::print("pi_line_legacy_wall_s={:.9f}\npi_line_direct_wall_s={:.9f}\n",
             legacyPiLineTime, directPiLineTime);
  const Real legacyRXLoadTime = runRXLoad("Legacy", true);
  const Real directRXLoadTime = runRXLoad("Direct", false);
  fmt::print("rxload_legacy_wall_s={:.9f}\nrxload_direct_wall_s={:.9f}\n",
             legacyRXLoadTime, directRXLoadTime);
  const Real legacyTransformerTime = runTransformer("Legacy", true);
  const Real directTransformerTime = runTransformer("Direct", false);
  fmt::print(
      "transformer_legacy_wall_s={:.9f}\ntransformer_direct_wall_s={:.9f}\n",
      legacyTransformerTime, directTransformerTime);
  const Real legacyLossyTransformerTime =
      runTransformer("LossyLegacy", true, true);
  const Real directLossyTransformerTime =
      runTransformer("LossyDirect", false, true);
  fmt::print("transformer_lossy_legacy_wall_s={:.9f}\n"
             "transformer_lossy_direct_wall_s={:.9f}\n",
             legacyLossyTransformerTime, directLossyTransformerTime);
  const Real legacyHalfLineTime = runHalfDecouplingLine("Legacy", true);
  const Real directHalfLineTime = runHalfDecouplingLine("Direct", false);
  fmt::print("half_line_legacy_wall_s={:.9f}\nhalf_line_direct_wall_s={:.9f}\n",
             legacyHalfLineTime, directHalfLineTime);
  const Real legacyDCPiLineTime = runDCPiLine("Legacy", true);
  const Real directDCPiLineTime = runDCPiLine("Direct", false);
  fmt::print("dc_piline_legacy_wall_s={:.9f}\ndc_piline_direct_wall_s={:.9f}\n",
             legacyDCPiLineTime, directDCPiLineTime);
  const Real legacyPh1PiLineTime = runPh1PiLine("Legacy", true);
  const Real directPh1PiLineTime = runPh1PiLine("Direct", false);
  fmt::print(
      "ph1_piline_legacy_wall_s={:.9f}\nph1_piline_direct_wall_s={:.9f}\n",
      legacyPh1PiLineTime, directPh1PiLineTime);
  const Real legacyRampTime = runVoltageSourceRamp("Legacy", true);
  const Real directRampTime = runVoltageSourceRamp("Direct", false);
  fmt::print("ramp_legacy_wall_s={:.9f}\nramp_direct_wall_s={:.9f}\n",
             legacyRampTime, directRampTime);
  const Real legacyGeneratorTime = runIdealGenerator("Legacy", true);
  const Real directGeneratorTime = runIdealGenerator("Direct", false);
  fmt::print("ideal_generator_legacy_wall_s={:.9f}\nideal_generator_direct_"
             "wall_s={:.9f}\n",
             legacyGeneratorTime, directGeneratorTime);
  const Real legacyTrStabTime = runTransientGenerator("Legacy", true);
  const Real directTrStabTime = runTransientGenerator("Direct", false);
  fmt::print("trstab_legacy_wall_s={:.9f}\ntrstab_direct_wall_s={:.9f}\n",
             legacyTrStabTime, directTrStabTime);
  const Real avLegacy = runAvInverter("Legacy", true);
  const Real avDirect = runAvInverter("Direct", false);
  fmt::print("av_vsi_legacy_wall_s={:.9f}\nav_vsi_direct_wall_s={:.9f}\n",
             avLegacy, avDirect);
  const Real vcoLegacy = runVcoInverter("Legacy", true);
  const Real vcoDirect = runVcoInverter("Direct", false);
  fmt::print("vco_vsi_legacy_wall_s={:.9f}\nvco_vsi_direct_wall_s={:.9f}\n",
             vcoLegacy, vcoDirect);
  return 0;
}
