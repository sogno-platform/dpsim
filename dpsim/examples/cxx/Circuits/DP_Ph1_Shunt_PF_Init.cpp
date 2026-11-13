// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <DPsim.h>
#include <dpsim-models/DP/DP_Ph1_RXLoad.h>
#include <dpsim-models/DP/DP_Ph1_SVC.h>
#include <dpsim-models/DP/DP_Ph1_SynchronGeneratorTrStab.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>

using namespace DPsim;
using namespace CPS;

namespace {
constexpr Real frequency = 50.;
constexpr Real voltage = 400.;
constexpr Real conductance = 0.1;
constexpr Real susceptance = 0.05;
constexpr Real timeStep = 1e-5;
constexpr Real finalTime = 0.04;

struct Csv {
  std::vector<std::vector<Real>> rows;
  std::map<String, size_t> columns;
};

Csv readCsv(const String &name) {
  String path = "logs/" + name + "/" + name + ".csv";
  std::ifstream file(path);
  if (!file) {
    const String flatPath = "logs/" + name + ".csv";
    file.open(flatPath);
    if (file)
      path = flatPath;
  }
  if (!file && std::filesystem::exists("logs")) {
    for (const auto &entry :
         std::filesystem::recursive_directory_iterator("logs")) {
      if (entry.is_regular_file() && entry.path().filename() == name + ".csv") {
        path = entry.path().string();
        file.open(path);
        if (file)
          break;
      }
    }
  }
  if (!file)
    throw std::runtime_error("Cannot open " + path);
  Csv csv;
  String line, cell;
  std::getline(file, line);
  std::stringstream header(line);
  size_t index = 0;
  while (std::getline(header, cell, ',')) {
    while (!cell.empty() && cell.front() == ' ')
      cell.erase(cell.begin());
    while (!cell.empty() && cell.back() == ' ')
      cell.pop_back();
    csv.columns[cell] = index++;
  }
  while (std::getline(file, line)) {
    if (line.empty())
      continue;
    std::stringstream values(line);
    std::vector<Real> row;
    while (std::getline(values, cell, ','))
      row.push_back(std::stod(cell));
    csv.rows.push_back(std::move(row));
  }
  return csv;
}

Real at(const Csv &csv, size_t row, const String &column) {
  return csv.rows.at(row).at(csv.columns.at(column));
}

Complex fundamental(const Csv &csv, const String &column, Real start,
                    Real end) {
  Complex result = 0.;
  size_t samples = 0;
  for (const auto &row : csv.rows) {
    const Real time = row[csv.columns.at("time")];
    if (time < start || time >= end)
      continue;
    const Real angle = 2. * PI * frequency * time;
    result += 2. * row[csv.columns.at(column)] *
              Complex(std::cos(angle), -std::sin(angle));
    ++samples;
  }
  if (!samples)
    throw std::runtime_error("Empty shunt fundamental window");
  return result / static_cast<Real>(samples);
}

Real runShuntCase(Bool useSubcomponents, UInt run) {
  constexpr Real runTimeStep = 1e-5;
  constexpr Real runFinalTime = 0.04;
  const String name = String("DP_Shunt_Benchmark_") +
                      (useSubcomponents ? "Legacy_" : "Direct_") +
                      std::to_string(run);
  auto bus = SimNode<Complex>::make("bus", PhaseType::Single);
  bus->setInitialVoltage(Complex(voltage, 0.));
  auto source = DP::Ph1::VoltageSource::make("source");
  source->setParameters(Complex(voltage, 0.), frequency);
  auto shunt = DP::Ph1::Shunt::make("shunt", useSubcomponents);
  shunt->setParameters(conductance, susceptance);
  source->connect({SimNode<Complex>::GND, bus});
  shunt->connect({bus});
  SystemTopology system(frequency, {bus}, {source, shunt});
  Simulation simulation(name, Logger::Level::off);
  simulation.setSystem(system);
  simulation.setDomain(Domain::DP);
  simulation.setTimeStep(runTimeStep);
  simulation.setFinalTime(runFinalTime);
  const auto start = std::chrono::steady_clock::now();
  simulation.run();
  return std::chrono::duration<Real>(std::chrono::steady_clock::now() - start)
      .count();
}

Csv runRXLoadComparison() {
  constexpr Real activePower = 1000.;
  constexpr Real reactivePower = 500.;
  const String name = "DP_RXLoad_DirectCompare";
  auto bus = SimNode<Complex>::make("bus", PhaseType::Single);
  bus->setInitialVoltage(Complex(voltage, 0.));
  auto source = DP::Ph1::VoltageSource::make("source");
  source->setParameters(Complex(voltage, 0.), frequency);
  auto legacy = DP::Ph1::RXLoad::make("legacy_load", true);
  auto direct = DP::Ph1::RXLoad::make("direct_load", false);
  legacy->setParameters(activePower, reactivePower, voltage);
  direct->setParameters(activePower, reactivePower, voltage);
  source->connect({SimNode<Complex>::GND, bus});
  legacy->connect({bus});
  direct->connect({bus});
  SystemTopology system(frequency, {bus}, {source, legacy, direct});
  auto logger = DataLogger::make(name);
  logger->logAttribute("legacy_current", legacy->attribute("i_intf"));
  logger->logAttribute("direct_current", direct->attribute("i_intf"));
  Simulation simulation(name, Logger::Level::off);
  simulation.setSystem(system);
  simulation.setDomain(Domain::DP);
  simulation.setTimeStep(1e-5);
  simulation.setFinalTime(0.04);
  simulation.addLogger(logger);
  simulation.run();
  return readCsv(name);
}

Csv runNetworkInjectionComparison() {
  const String name = "DP_NetworkInjection_DirectCompare";
  auto legacyBus = SimNode<Complex>::make("legacy_bus", PhaseType::Single);
  auto directBus = SimNode<Complex>::make("direct_bus", PhaseType::Single);
  legacyBus->setInitialVoltage(Complex(voltage, 0.));
  directBus->setInitialVoltage(Complex(voltage, 0.));
  auto legacySource = DP::Ph1::NetworkInjection::make("legacy_source", true);
  auto directSource = DP::Ph1::NetworkInjection::make("direct_source", false);
  legacySource->setParameters(Complex(voltage, 0.), frequency);
  directSource->setParameters(Complex(voltage, 0.), frequency);
  auto legacyLoad = DP::Ph1::Resistor::make("legacy_load");
  auto directLoad = DP::Ph1::Resistor::make("direct_load");
  legacyLoad->setParameters(voltage * voltage / 1000.);
  directLoad->setParameters(voltage * voltage / 1000.);
  legacySource->connect({legacyBus});
  directSource->connect({directBus});
  legacyLoad->connect({legacyBus, SimNode<Complex>::GND});
  directLoad->connect({directBus, SimNode<Complex>::GND});
  SystemTopology system(frequency, {legacyBus, directBus},
                        {legacySource, directSource, legacyLoad, directLoad});
  auto logger = DataLogger::make(name);
  logger->logAttribute("legacy_voltage", legacySource->attribute("v_intf"));
  logger->logAttribute("direct_voltage", directSource->attribute("v_intf"));
  logger->logAttribute("legacy_current", legacySource->attribute("i_intf"));
  logger->logAttribute("direct_current", directSource->attribute("i_intf"));
  Simulation simulation(name, Logger::Level::off);
  simulation.setSystem(system);
  simulation.setDomain(Domain::DP);
  simulation.setTimeStep(1e-5);
  simulation.setFinalTime(0.04);
  simulation.addLogger(logger);
  simulation.run();
  return readCsv(name);
}

Csv runPiLineComparison() {
  const String name = "DP_PiLine_DirectCompare";
  auto legacyBus0 = SimNode<Complex>::make("legacy_bus0", PhaseType::Single);
  auto legacyBus1 = SimNode<Complex>::make("legacy_bus1", PhaseType::Single);
  auto directBus0 = SimNode<Complex>::make("direct_bus0", PhaseType::Single);
  auto directBus1 = SimNode<Complex>::make("direct_bus1", PhaseType::Single);
  for (const auto &bus : {legacyBus0, directBus0})
    bus->setInitialVoltage(Complex(voltage, 0.));
  for (const auto &bus : {legacyBus1, directBus1})
    bus->setInitialVoltage(Complex(voltage * 0.99, -2.));
  auto legacySource = DP::Ph1::VoltageSource::make("legacy_source");
  auto directSource = DP::Ph1::VoltageSource::make("direct_source");
  legacySource->setParameters(Complex(voltage, 0.), frequency);
  directSource->setParameters(Complex(voltage, 0.), frequency);
  auto legacyLine = DP::Ph1::PiLine::make("legacy_line", true);
  auto directLine = DP::Ph1::PiLine::make("direct_line", false);
  for (const auto &line : {legacyLine, directLine})
    line->setParameters(0.1, 1e-3, 1e-6, 1e-6);
  auto legacyLoad = DP::Ph1::Resistor::make("legacy_load");
  auto directLoad = DP::Ph1::Resistor::make("direct_load");
  legacyLoad->setParameters(voltage * voltage / 1000.);
  directLoad->setParameters(voltage * voltage / 1000.);
  legacySource->connect({SimNode<Complex>::GND, legacyBus0});
  directSource->connect({SimNode<Complex>::GND, directBus0});
  legacyLine->connect({legacyBus0, legacyBus1});
  directLine->connect({directBus0, directBus1});
  legacyLoad->connect({legacyBus1, SimNode<Complex>::GND});
  directLoad->connect({directBus1, SimNode<Complex>::GND});
  SystemTopology system(frequency,
                        {legacyBus0, legacyBus1, directBus0, directBus1},
                        {legacySource, directSource, legacyLine, directLine,
                         legacyLoad, directLoad});
  auto logger = DataLogger::make(name);
  logger->logAttribute("legacy_voltage", legacyLine->attribute("v_intf"));
  logger->logAttribute("direct_voltage", directLine->attribute("v_intf"));
  logger->logAttribute("legacy_current", legacyLine->attribute("i_intf"));
  logger->logAttribute("direct_current", directLine->attribute("i_intf"));
  Simulation simulation(name, Logger::Level::off);
  simulation.setSystem(system);
  simulation.setDomain(Domain::DP);
  simulation.setTimeStep(timeStep);
  simulation.setFinalTime(finalTime);
  simulation.addLogger(logger);
  simulation.run();
  return readCsv(name);
}

Csv runRxLineComparison() {
  const String name = "DP_RxLine_DirectCompare";
  auto legacyBus0 = SimNode<Complex>::make("legacy_bus0", PhaseType::Single);
  auto legacyBus1 = SimNode<Complex>::make("legacy_bus1", PhaseType::Single);
  auto directBus0 = SimNode<Complex>::make("direct_bus0", PhaseType::Single);
  auto directBus1 = SimNode<Complex>::make("direct_bus1", PhaseType::Single);
  for (const auto &bus : {legacyBus0, directBus0})
    bus->setInitialVoltage(Complex(voltage, 0.));
  for (const auto &bus : {legacyBus1, directBus1})
    bus->setInitialVoltage(Complex(voltage * 0.99, -2.));
  auto legacySource = DP::Ph1::VoltageSource::make("legacy_source");
  auto directSource = DP::Ph1::VoltageSource::make("direct_source");
  legacySource->setParameters(Complex(voltage, 0.), frequency);
  directSource->setParameters(Complex(voltage, 0.), frequency);
  auto legacyLine = DP::Ph1::RxLine::make("legacy_line", true);
  auto directLine = DP::Ph1::RxLine::make("direct_line", false);
  for (const auto &line : {legacyLine, directLine})
    line->setParameters(0.1, 1e-3);
  auto legacyLoad = DP::Ph1::Resistor::make("legacy_load");
  auto directLoad = DP::Ph1::Resistor::make("direct_load");
  legacyLoad->setParameters(voltage * voltage / 1000.);
  directLoad->setParameters(voltage * voltage / 1000.);
  legacySource->connect({SimNode<Complex>::GND, legacyBus0});
  directSource->connect({SimNode<Complex>::GND, directBus0});
  legacyLine->connect({legacyBus0, legacyBus1});
  directLine->connect({directBus0, directBus1});
  legacyLoad->connect({legacyBus1, SimNode<Complex>::GND});
  directLoad->connect({directBus1, SimNode<Complex>::GND});
  SystemTopology system(frequency,
                        {legacyBus0, legacyBus1, directBus0, directBus1},
                        {legacySource, directSource, legacyLine, directLine,
                         legacyLoad, directLoad});
  auto logger = DataLogger::make(name);
  logger->logAttribute("legacy_voltage", legacyLine->attribute("v_intf"));
  logger->logAttribute("direct_voltage", directLine->attribute("v_intf"));
  logger->logAttribute("legacy_current", legacyLine->attribute("i_intf"));
  logger->logAttribute("direct_current", directLine->attribute("i_intf"));
  Simulation simulation(name, Logger::Level::off);
  simulation.setSystem(system);
  simulation.setDomain(Domain::DP);
  simulation.setTimeStep(timeStep);
  simulation.setFinalTime(finalTime);
  simulation.addLogger(logger);
  simulation.run();
  return readCsv(name);
}

Csv runSourceWrapperComparison(Bool generator) {
  const String name = generator ? "DP_IdealGenerator_DirectCompare"
                                : "DP_VoltageSourceRamp_DirectCompare";
  auto legacyBus = SimNode<Complex>::make("legacy_bus", PhaseType::Single);
  auto directBus = SimNode<Complex>::make("direct_bus", PhaseType::Single);
  legacyBus->setInitialVoltage(Complex(voltage, 0.));
  directBus->setInitialVoltage(Complex(voltage, 0.));
  SimPowerComp<Complex>::Ptr legacySource;
  SimPowerComp<Complex>::Ptr directSource;
  if (generator) {
    legacySource = DP::Ph1::SynchronGeneratorIdeal::make("legacy_source", true);
    directSource =
        DP::Ph1::SynchronGeneratorIdeal::make("direct_source", false);
    legacySource->connect({legacyBus});
    directSource->connect({directBus});
  } else {
    auto legacyRamp = DP::Ph1::VoltageSourceRamp::make("legacy_source", true);
    auto directRamp = DP::Ph1::VoltageSourceRamp::make("direct_source", false);
    legacyRamp->setParameters(Complex(voltage, 0.), Complex(20., 0.), frequency,
                              0.2, 0.01, 0.01);
    directRamp->setParameters(Complex(voltage, 0.), Complex(20., 0.), frequency,
                              0.2, 0.01, 0.01);
    legacySource = legacyRamp;
    directSource = directRamp;
    legacySource->connect({SimNode<Complex>::GND, legacyBus});
    directSource->connect({SimNode<Complex>::GND, directBus});
  }
  auto legacyLoad = DP::Ph1::Resistor::make("legacy_load");
  auto directLoad = DP::Ph1::Resistor::make("direct_load");
  legacyLoad->setParameters(voltage * voltage / 1000.);
  directLoad->setParameters(voltage * voltage / 1000.);
  legacyLoad->connect({legacyBus, SimNode<Complex>::GND});
  directLoad->connect({directBus, SimNode<Complex>::GND});
  SystemTopology system(frequency, {legacyBus, directBus},
                        {legacySource, directSource, legacyLoad, directLoad});
  auto logger = DataLogger::make(name);
  logger->logAttribute("legacy_voltage", legacySource->attribute("v_intf"));
  logger->logAttribute("direct_voltage", directSource->attribute("v_intf"));
  logger->logAttribute("legacy_current", legacySource->attribute("i_intf"));
  logger->logAttribute("direct_current", directSource->attribute("i_intf"));
  Simulation simulation(name, Logger::Level::off);
  simulation.setSystem(system);
  simulation.setDomain(Domain::DP);
  simulation.setTimeStep(timeStep);
  simulation.setFinalTime(finalTime);
  simulation.addLogger(logger);
  simulation.run();
  return readCsv(name);
}

Csv runPQLoadCSComparison() {
  const String name = "DP_PQLoadCS_DirectCompare";
  auto legacyBus = SimNode<Complex>::make("legacy_bus", PhaseType::Single);
  auto directBus = SimNode<Complex>::make("direct_bus", PhaseType::Single);
  legacyBus->setInitialVoltage(Complex(voltage, 0.));
  directBus->setInitialVoltage(Complex(voltage, 0.));
  auto legacySource = DP::Ph1::VoltageSource::make("legacy_source");
  auto directSource = DP::Ph1::VoltageSource::make("direct_source");
  legacySource->setParameters(Complex(voltage, 0.), frequency);
  directSource->setParameters(Complex(voltage, 0.), frequency);
  auto legacyLoad = DP::Ph1::PQLoadCS::make("legacy_load", true);
  auto directLoad = DP::Ph1::PQLoadCS::make("direct_load", false);
  legacyLoad->setParameters(1000., 500., voltage);
  directLoad->setParameters(1000., 500., voltage);
  legacySource->connect({SimNode<Complex>::GND, legacyBus});
  directSource->connect({SimNode<Complex>::GND, directBus});
  legacyLoad->connect({legacyBus});
  directLoad->connect({directBus});
  SystemTopology system(frequency, {legacyBus, directBus},
                        {legacySource, directSource, legacyLoad, directLoad});
  auto logger = DataLogger::make(name);
  logger->logAttribute("legacy_voltage", legacyLoad->attribute("v_intf"));
  logger->logAttribute("direct_voltage", directLoad->attribute("v_intf"));
  logger->logAttribute("legacy_current", legacyLoad->attribute("i_intf"));
  logger->logAttribute("direct_current", directLoad->attribute("i_intf"));
  Simulation simulation(name, Logger::Level::off);
  simulation.setSystem(system);
  simulation.setDomain(Domain::DP);
  simulation.setTimeStep(timeStep);
  simulation.setFinalTime(finalTime);
  simulation.addLogger(logger);
  simulation.run();
  return readCsv(name);
}

Csv runSVCComparison() {
  const String name = "DP_SVC_DirectCompare";
  auto legacyBus = SimNode<Complex>::make("legacy_bus", PhaseType::Single);
  auto directBus = SimNode<Complex>::make("direct_bus", PhaseType::Single);
  legacyBus->setInitialVoltage(Complex(voltage, 0.));
  directBus->setInitialVoltage(Complex(voltage, 0.));
  auto legacySource = DP::Ph1::VoltageSource::make("legacy_source");
  auto directSource = DP::Ph1::VoltageSource::make("direct_source");
  legacySource->setParameters(Complex(voltage, 0.), frequency);
  directSource->setParameters(Complex(voltage, 0.), frequency);
  auto legacySVC = DP::Ph1::SVC::make("legacy_svc", true);
  auto directSVC = DP::Ph1::SVC::make("direct_svc", false);
  for (const auto &svc : {legacySVC, directSVC}) {
    svc->setParameters(1., -1., 1e5, voltage, voltage);
    svc->setControllerParameters(0.01, 1.);
  }
  auto legacyLoad = DP::Ph1::Resistor::make("legacy_load");
  auto directLoad = DP::Ph1::Resistor::make("direct_load");
  legacyLoad->setParameters(voltage * voltage / 1000.);
  directLoad->setParameters(voltage * voltage / 1000.);
  legacySource->connect({SimNode<Complex>::GND, legacyBus});
  directSource->connect({SimNode<Complex>::GND, directBus});
  legacySVC->connect({legacyBus});
  directSVC->connect({directBus});
  legacyLoad->connect({legacyBus, SimNode<Complex>::GND});
  directLoad->connect({directBus, SimNode<Complex>::GND});
  SystemTopology system(frequency, {legacyBus, directBus},
                        {legacySource, directSource, legacySVC, directSVC,
                         legacyLoad, directLoad});
  auto logger = DataLogger::make(name);
  logger->logAttribute("legacy_voltage", legacySVC->attribute("v_intf"));
  logger->logAttribute("direct_voltage", directSVC->attribute("v_intf"));
  logger->logAttribute("legacy_current", legacySVC->attribute("i_intf"));
  logger->logAttribute("direct_current", directSVC->attribute("i_intf"));
  Simulation simulation(name, Logger::Level::off);
  simulation.setSystem(system);
  simulation.setDomain(Domain::DP);
  simulation.setTimeStep(timeStep);
  simulation.setFinalTime(finalTime);
  simulation.addLogger(logger);
  simulation.run();
  return readCsv(name);
}

Csv runDPInverterCase(Bool useSubcomponents, const String &name,
                      Real *wallTime = nullptr, Bool controlEnabled = false,
                      Real duration = 0.04, Bool withTransformer = false) {
  auto bus = SimNode<Complex>::make("bus", PhaseType::Single);
  bus->setInitialVoltage(Complex(voltage, 0.));
  auto source = DP::Ph1::VoltageSource::make("source");
  source->setParameters(Complex(voltage, 0.), frequency);
  auto inverter = DP::Ph1::AvVoltageSourceInverterDQ::make(
      "inverter", useSubcomponents, Logger::Level::off, withTransformer);
  inverter->setParameters(2. * PI * frequency, voltage, 1000., 250.);
  inverter->setControllerParameters(0.25, 2., 0.001, 0.08, 3.77, 1400.,
                                    2. * PI * frequency);
  inverter->setFilterParameters(0.16, 10e-6, 5., 100.);
  if (withTransformer)
    inverter->setTransformerParameters(voltage, voltage, 1e6, 1., 0., 0., 1e-3);
  inverter->withControl(controlEnabled);
  auto load = DP::Ph1::Resistor::make("load");
  load->setParameters(voltage * voltage / 1000.);
  source->connect({SimNode<Complex>::GND, bus});
  inverter->connect({bus});
  load->connect({bus, SimNode<Complex>::GND});
  SystemTopology system(frequency, {bus}, {source, inverter, load});
  auto logger = DataLogger::make(name);
  logger->logAttribute("voltage", inverter->attribute("v_intf"));
  logger->logAttribute("current", inverter->attribute("i_intf"));
  logger->logAttribute("source_voltage", inverter->mVs);
  Simulation simulation(name, Logger::Level::off);
  simulation.setSystem(system);
  simulation.setDomain(Domain::DP);
  simulation.setTimeStep(1e-5);
  simulation.setFinalTime(duration);
  simulation.addLogger(logger);
  const auto start = std::chrono::steady_clock::now();
  simulation.run();
  if (wallTime)
    *wallTime =
        std::chrono::duration<Real>(std::chrono::steady_clock::now() - start)
            .count();
  return readCsv(name);
}

Csv runRXLoadSwitchCase(Bool useSubcomponents, const String &name,
                        Real *wallTime = nullptr) {
  auto bus = SimNode<Complex>::make("bus", PhaseType::Single);
  bus->setInitialVoltage(Complex(voltage, 0.));
  auto source = DP::Ph1::VoltageSource::make("source");
  source->setParameters(Complex(voltage, 0.), frequency);
  auto load = DP::Ph1::RXLoadSwitch::make("load", useSubcomponents);
  load->setParameters(1000., 0., voltage, 1e9, 1e-6, true);
  source->connect({SimNode<Complex>::GND, bus});
  load->connect({bus});
  SystemTopology system(frequency, {bus}, {source, load});
  auto logger = DataLogger::make(name);
  logger->logAttribute("voltage", load->attribute("v_intf"));
  logger->logAttribute("current", load->attribute("i_intf"));
  Simulation simulation(name, Logger::Level::off);
  simulation.setSystem(system);
  simulation.setDomain(Domain::DP);
  simulation.setTimeStep(1e-5);
  simulation.setFinalTime(0.04);
  simulation.addLogger(logger);
  const auto start = std::chrono::steady_clock::now();
  simulation.run();
  if (wallTime)
    *wallTime =
        std::chrono::duration<Real>(std::chrono::steady_clock::now() - start)
            .count();
  return readCsv(name);
}

Real runSVCBenchmarkCase(Bool useSubcomponents, UInt run) {
  const String name = String("DP_SVC_Benchmark_") +
                      (useSubcomponents ? "Legacy_" : "Direct_") +
                      std::to_string(run);
  auto bus = SimNode<Complex>::make("bus", PhaseType::Single);
  bus->setInitialVoltage(Complex(voltage, 0.));
  auto source = DP::Ph1::VoltageSource::make("source");
  source->setParameters(Complex(voltage, 0.), frequency);
  auto svc = DP::Ph1::SVC::make("svc", useSubcomponents);
  svc->setParameters(1., -1., 1e5, voltage, voltage);
  svc->setControllerParameters(0.01, 1.);
  auto load = DP::Ph1::Resistor::make("load");
  load->setParameters(voltage * voltage / 1000.);
  source->connect({SimNode<Complex>::GND, bus});
  svc->connect({bus});
  load->connect({bus, SimNode<Complex>::GND});
  SystemTopology system(frequency, {bus}, {source, svc, load});
  Simulation simulation(name, Logger::Level::off);
  simulation.setSystem(system);
  simulation.setDomain(Domain::DP);
  simulation.setTimeStep(timeStep);
  simulation.setFinalTime(finalTime);
  const auto start = std::chrono::steady_clock::now();
  simulation.run();
  return std::chrono::duration<Real>(std::chrono::steady_clock::now() - start)
      .count();
}

Csv runThreePhaseNetworkInjectionComparison() {
  const String name = "DP_Ph3_NetworkInjection_DirectCompare";
  auto legacyBus = SimNode<Complex>::make("legacy_bus", PhaseType::ABC);
  auto directBus = SimNode<Complex>::make("direct_bus", PhaseType::ABC);
  auto legacySource = DP::Ph3::NetworkInjection::make("legacy_source", true);
  auto directSource = DP::Ph3::NetworkInjection::make("direct_source", false);
  const MatrixComp reference =
      Math::singlePhaseVariableToThreePhase(Complex(230., 0.));
  legacySource->setParameters(reference, frequency);
  directSource->setParameters(reference, frequency);
  legacySource->connect({legacyBus});
  directSource->connect({directBus});
  SystemTopology system(frequency, {legacyBus, directBus},
                        {legacySource, directSource});
  auto logger = DataLogger::make(name);
  logger->logAttribute("legacy_voltage", legacySource->attribute("v_intf"));
  logger->logAttribute("direct_voltage", directSource->attribute("v_intf"));
  logger->logAttribute("legacy_current", legacySource->attribute("i_intf"));
  logger->logAttribute("direct_current", directSource->attribute("i_intf"));
  Simulation simulation(name, Logger::Level::off);
  simulation.setSystem(system);
  simulation.setDomain(Domain::DP);
  simulation.setTimeStep(timeStep);
  simulation.setFinalTime(finalTime);
  simulation.addLogger(logger);
  simulation.run();
  return readCsv(name);
}

Csv runThreePhasePiLineComparison() {
  const String name = "DP_Ph3_PiLine_DirectCompare";
  auto legacyBus0 = SimNode<Complex>::make("legacy_bus0", PhaseType::ABC);
  auto legacyBus1 = SimNode<Complex>::make("legacy_bus1", PhaseType::ABC);
  auto directBus0 = SimNode<Complex>::make("direct_bus0", PhaseType::ABC);
  auto directBus1 = SimNode<Complex>::make("direct_bus1", PhaseType::ABC);
  auto legacySource = DP::Ph3::NetworkInjection::make("legacy_source", true);
  auto directSource = DP::Ph3::NetworkInjection::make("direct_source", true);
  const MatrixComp reference =
      Math::singlePhaseVariableToThreePhase(Complex(230., 0.));
  legacySource->setParameters(reference, frequency);
  directSource->setParameters(reference, frequency);
  auto legacyLine = DP::Ph3::PiLine::make("legacy_line", true);
  auto directLine = DP::Ph3::PiLine::make("direct_line", false);
  for (const auto &line : {legacyLine, directLine})
    line->setParameters(
        0.1 * Matrix::Identity(3, 3), 1e-3 * Matrix::Identity(3, 3),
        1e-6 * Matrix::Identity(3, 3), 1e-6 * Matrix::Identity(3, 3));
  auto legacyLoad = DP::Ph3::Resistor::make("legacy_load");
  auto directLoad = DP::Ph3::Resistor::make("direct_load");
  legacyLoad->setParameters(100. * Matrix::Identity(3, 3));
  directLoad->setParameters(100. * Matrix::Identity(3, 3));
  legacySource->connect({legacyBus0});
  directSource->connect({directBus0});
  legacyLine->connect({legacyBus0, legacyBus1});
  directLine->connect({directBus0, directBus1});
  legacyLoad->connect({legacyBus1, SimNode<Complex>::GND});
  directLoad->connect({directBus1, SimNode<Complex>::GND});
  SystemTopology system(frequency,
                        {legacyBus0, legacyBus1, directBus0, directBus1},
                        {legacySource, directSource, legacyLine, directLine,
                         legacyLoad, directLoad});
  auto logger = DataLogger::make(name);
  logger->logAttribute("legacy_voltage", legacyLine->attribute("v_intf"));
  logger->logAttribute("direct_voltage", directLine->attribute("v_intf"));
  logger->logAttribute("legacy_current", legacyLine->attribute("i_intf"));
  logger->logAttribute("direct_current", directLine->attribute("i_intf"));
  Simulation simulation(name, Logger::Level::off);
  simulation.setSystem(system);
  simulation.setDomain(Domain::DP);
  simulation.setTimeStep(timeStep);
  simulation.setFinalTime(finalTime);
  simulation.addLogger(logger);
  simulation.run();
  return readCsv(name);
}

Csv runTransformerComparison() {
  const String name = "DP_Ph1_Transformer_DirectCompare";
  auto legacyBus0 = SimNode<Complex>::make("legacy_bus0", PhaseType::Single);
  auto legacyBus1 = SimNode<Complex>::make("legacy_bus1", PhaseType::Single);
  auto directBus0 = SimNode<Complex>::make("direct_bus0", PhaseType::Single);
  auto directBus1 = SimNode<Complex>::make("direct_bus1", PhaseType::Single);
  auto legacySource = DP::Ph1::NetworkInjection::make("legacy_source");
  auto directSource = DP::Ph1::NetworkInjection::make("direct_source");
  legacySource->setParameters(voltage, frequency);
  directSource->setParameters(voltage, frequency);
  auto legacyTransformer = DP::Ph1::Transformer::make(
      "legacy_transformer", "legacy_transformer", Logger::Level::off, true);
  auto directTransformer = DP::Ph1::Transformer::make(
      "direct_transformer", false, Logger::Level::off, true);
  for (const auto &transformer : {legacyTransformer, directTransformer}) {
    transformer->setReferenceWinding(WindingReference::Primary);
    transformer->setParameters(230., 230., 10000., 1., 0., 0.1, 1e-3);
  }
  auto legacyLoad = DP::Ph1::Resistor::make("legacy_load");
  auto directLoad = DP::Ph1::Resistor::make("direct_load");
  legacyLoad->setParameters(100.);
  directLoad->setParameters(100.);
  legacySource->connect({legacyBus0});
  directSource->connect({directBus0});
  legacyTransformer->connect({legacyBus0, legacyBus1});
  directTransformer->connect({directBus0, directBus1});
  legacyLoad->connect({legacyBus1, SimNode<Complex>::GND});
  directLoad->connect({directBus1, SimNode<Complex>::GND});
  SystemTopology system(frequency,
                        {legacyBus0, legacyBus1, directBus0, directBus1},
                        {legacySource, directSource, legacyTransformer,
                         directTransformer, legacyLoad, directLoad});
  auto logger = DataLogger::make(name);
  logger->logAttribute("legacy_voltage",
                       legacyTransformer->attribute("v_intf"));
  logger->logAttribute("direct_voltage",
                       directTransformer->attribute("v_intf"));
  logger->logAttribute("legacy_current",
                       legacyTransformer->attribute("i_intf"));
  logger->logAttribute("direct_current",
                       directTransformer->attribute("i_intf"));
  Simulation simulation(name, Logger::Level::off);
  simulation.setSystem(system);
  simulation.setDomain(Domain::DP);
  simulation.setTimeStep(timeStep);
  simulation.setFinalTime(finalTime);
  simulation.addLogger(logger);
  simulation.run();
  return readCsv(name);
}

Csv runThreePhaseTransformerComparison() {
  auto runIsolated = [](Bool useSubcomponents, const String &variant) {
    const String name = "DP_Ph3_Transformer_" + variant + "_Isolated";
    auto bus0 = SimNode<Complex>::make("bus0", PhaseType::ABC);
    auto bus1 = SimNode<Complex>::make("bus1", PhaseType::ABC);
    auto source = DP::Ph3::NetworkInjection::make("source", true);
    const MatrixComp sourceVoltage =
        Math::singlePhaseVariableToThreePhase(Complex(230., 0.));
    bus0->setInitialVoltage(sourceVoltage);
    bus1->setInitialVoltage(0.98 * sourceVoltage);
    source->setParameters(sourceVoltage, frequency);
    auto transformer =
        useSubcomponents
            ? DP::Ph3::Transformer::make("transformer", "transformer",
                                         Logger::Level::off, true)
            : DP::Ph3::Transformer::make("transformer", false,
                                         Logger::Level::off, true);
    transformer->setReferenceWinding(WindingReference::Primary);
    transformer->setParameters(230., 230., 10000., 1., 0.,
                               0.1 * Matrix::Identity(3, 3),
                               1e-3 * Matrix::Identity(3, 3));
    auto load = DP::Ph3::Resistor::make("load");
    load->setParameters(100. * Matrix::Identity(3, 3));
    source->connect({bus0});
    transformer->connect({bus0, bus1});
    load->connect({bus1, SimNode<Complex>::GND});
    SystemTopology system(frequency, {bus0, bus1}, {source, transformer, load});
    auto logger = DataLogger::make(name);
    logger->logAttribute("voltage", transformer->attribute("v_intf"));
    logger->logAttribute("current", transformer->attribute("i_intf"));
    Simulation simulation(name, Logger::Level::off);
    simulation.setSystem(system);
    simulation.setDomain(Domain::DP);
    simulation.setTimeStep(timeStep);
    simulation.setFinalTime(finalTime);
    simulation.addLogger(logger);
    simulation.run();
    return readCsv(name);
  };

  const Csv legacy = runIsolated(true, "Legacy");
  const Csv direct = runIsolated(false, "Direct");
  if (legacy.rows.size() != direct.rows.size())
    throw std::runtime_error("Isolated transformer trace lengths differ");

  Csv combined;
  combined.columns["time"] = 0;
  size_t nextColumn = 1;
  for (const String &variant : {String("legacy"), String("direct")})
    for (const String &quantity : {String("voltage"), String("current")})
      for (UInt phase = 0; phase < 3; ++phase)
        for (const String &part : {String("im"), String("re")})
          combined.columns[variant + "_" + quantity + "_" +
                           std::to_string(phase) + "." + part] = nextColumn++;
  combined.rows.resize(legacy.rows.size(), std::vector<Real>(nextColumn));
  for (size_t row = 0; row < legacy.rows.size(); ++row) {
    combined.rows[row][0] = at(legacy, row, "time");
    for (const auto &[variant, trace] :
         {std::pair<String, const Csv *>("legacy", &legacy),
          std::pair<String, const Csv *>("direct", &direct)}) {
      for (const String &quantity : {String("voltage"), String("current")})
        for (UInt phase = 0; phase < 3; ++phase)
          for (const String &part : {String("im"), String("re")}) {
            const String column =
                quantity + "_" + std::to_string(phase) + "." + part;
            combined.rows[row][combined.columns.at(
                variant + "_" + quantity + "_" + std::to_string(phase) + "." +
                part)] = at(*trace, row, column);
          }
    }
  }
  return combined;
}

Real runNetworkInjectionCase(Bool useSubcomponents, UInt run) {
  const String name = String("DP_NetworkInjection_Benchmark_") +
                      (useSubcomponents ? "Legacy_" : "Direct_") +
                      std::to_string(run);
  auto bus = SimNode<Complex>::make("bus", PhaseType::Single);
  bus->setInitialVoltage(Complex(voltage, 0.));
  auto source = DP::Ph1::NetworkInjection::make("source", useSubcomponents);
  source->setParameters(Complex(voltage, 0.), frequency);
  auto load = DP::Ph1::Resistor::make("load");
  load->setParameters(voltage * voltage / 1000.);
  source->connect({bus});
  load->connect({bus, SimNode<Complex>::GND});
  SystemTopology system(frequency, {bus}, {source, load});
  Simulation simulation(name, Logger::Level::off);
  simulation.setSystem(system);
  simulation.setDomain(Domain::DP);
  simulation.setTimeStep(1e-5);
  simulation.setFinalTime(0.04);
  const auto start = std::chrono::steady_clock::now();
  simulation.run();
  return std::chrono::duration<Real>(std::chrono::steady_clock::now() - start)
      .count();
}

Real runRXLoadCase(Bool useSubcomponents, UInt run) {
  constexpr Real activePower = 1000.;
  constexpr Real reactivePower = 500.;
  constexpr Real runTimeStep = 1e-5;
  constexpr Real runFinalTime = 0.04;
  const String name = String("DP_RXLoad_Benchmark_") +
                      (useSubcomponents ? "Legacy_" : "Direct_") +
                      std::to_string(run);
  auto bus = SimNode<Complex>::make("bus", PhaseType::Single);
  bus->setInitialVoltage(Complex(voltage, 0.));
  auto source = DP::Ph1::VoltageSource::make("source");
  source->setParameters(Complex(voltage, 0.), frequency);
  auto load = DP::Ph1::RXLoad::make("load", useSubcomponents);
  load->setParameters(activePower, reactivePower, voltage);
  source->connect({SimNode<Complex>::GND, bus});
  load->connect({bus});
  SystemTopology system(frequency, {bus}, {source, load});
  Simulation simulation(name, Logger::Level::off);
  simulation.setSystem(system);
  simulation.setDomain(Domain::DP);
  simulation.setTimeStep(runTimeStep);
  simulation.setFinalTime(runFinalTime);
  const auto start = std::chrono::steady_clock::now();
  simulation.run();
  return std::chrono::duration<Real>(std::chrono::steady_clock::now() - start)
      .count();
}

Real runLineCase(Bool piLine, Bool useSubcomponents, UInt run) {
  const String name =
      String(piLine ? "DP_PiLine_Benchmark_" : "DP_RxLine_Benchmark_") +
      (useSubcomponents ? "Legacy_" : "Direct_") + std::to_string(run);
  auto bus0 = SimNode<Complex>::make("bus0", PhaseType::Single);
  auto bus1 = SimNode<Complex>::make("bus1", PhaseType::Single);
  bus0->setInitialVoltage(Complex(voltage, 0.));
  bus1->setInitialVoltage(Complex(voltage * 0.99, -2.));
  auto source = DP::Ph1::VoltageSource::make("source");
  source->setParameters(Complex(voltage, 0.), frequency);
  SimPowerComp<Complex>::Ptr line;
  if (piLine) {
    auto component = DP::Ph1::PiLine::make("line", useSubcomponents);
    component->setParameters(0.1, 1e-3, 1e-6, 1e-6);
    line = component;
  } else {
    auto component = DP::Ph1::RxLine::make("line", useSubcomponents);
    component->setParameters(0.1, 1e-3);
    line = component;
  }
  auto load = DP::Ph1::Resistor::make("load");
  load->setParameters(voltage * voltage / 1000.);
  source->connect({SimNode<Complex>::GND, bus0});
  line->connect({bus0, bus1});
  load->connect({bus1, SimNode<Complex>::GND});
  SystemTopology system(frequency, {bus0, bus1}, {source, line, load});
  Simulation simulation(name, Logger::Level::off);
  simulation.setSystem(system);
  simulation.setDomain(Domain::DP);
  simulation.setTimeStep(timeStep);
  simulation.setFinalTime(finalTime);
  const auto start = std::chrono::steady_clock::now();
  simulation.run();
  return std::chrono::duration<Real>(std::chrono::steady_clock::now() - start)
      .count();
}

Real runSourceWrapperCase(Bool generator, Bool useSubcomponents, UInt run) {
  const String name = String(generator ? "DP_IdealGenerator_Benchmark_"
                                       : "DP_VoltageRamp_Benchmark_") +
                      (useSubcomponents ? "Legacy_" : "Direct_") +
                      std::to_string(run);
  auto bus = SimNode<Complex>::make("bus", PhaseType::Single);
  bus->setInitialVoltage(Complex(voltage, 0.));
  SimPowerComp<Complex>::Ptr source;
  if (generator) {
    source = DP::Ph1::SynchronGeneratorIdeal::make("source", useSubcomponents);
    source->connect({bus});
  } else {
    auto ramp = DP::Ph1::VoltageSourceRamp::make("source", useSubcomponents);
    ramp->setParameters(Complex(voltage, 0.), Complex(20., 0.), frequency, 0.2,
                        0.01, 0.01);
    source = ramp;
    source->connect({SimNode<Complex>::GND, bus});
  }
  auto load = DP::Ph1::Resistor::make("load");
  load->setParameters(voltage * voltage / 1000.);
  load->connect({bus, SimNode<Complex>::GND});
  SystemTopology system(frequency, {bus}, {source, load});
  Simulation simulation(name, Logger::Level::off);
  simulation.setSystem(system);
  simulation.setDomain(Domain::DP);
  simulation.setTimeStep(timeStep);
  simulation.setFinalTime(finalTime);
  const auto start = std::chrono::steady_clock::now();
  simulation.run();
  return std::chrono::duration<Real>(std::chrono::steady_clock::now() - start)
      .count();
}

Csv runTransientGeneratorTrace(Bool useSubcomponents, UInt run) {
  const String name = String("DP_TrStab_Generator_Trace_") +
                      (useSubcomponents ? "Legacy_" : "Direct_") +
                      std::to_string(run);
  auto bus = SimNode<Complex>::make("bus", PhaseType::Single,
                                    std::vector<Complex>{Complex(voltage, 0.)});
  auto generator =
      DP::Ph1::SynchronGeneratorTrStab::make("generator", useSubcomponents);
  generator->setStandardParametersPU(1e6, voltage, frequency, 0.2, 3.0, 0.01,
                                     0.01);
  generator->setInitialValues(Complex(-5e5, 0.), 5e5);
  generator->connect({bus});
  auto load = DP::Ph1::Resistor::make("load");
  load->setParameters(voltage * voltage / 5e5);
  load->connect({SimNode<Complex>::GND, bus});
  SystemTopology system(frequency, {bus}, {generator, load});
  auto logger = DataLogger::make(name);
  logger->logAttribute("voltage", generator->attribute("v_intf"));
  logger->logAttribute("current", generator->attribute("i_intf"));
  logger->logAttribute("speed", generator->attribute("w_r"));
  Simulation simulation(name, Logger::Level::off);
  simulation.setSystem(system);
  simulation.setDomain(Domain::DP);
  simulation.setTimeStep(1e-5);
  simulation.setFinalTime(0.04);
  simulation.addLogger(logger);
  simulation.run();
  return readCsv(name);
}

Real runTransientGeneratorBenchmark(Bool useSubcomponents, UInt run) {
  const String name = String("DP_TrStab_Generator_Benchmark_") +
                      (useSubcomponents ? "Legacy_" : "Direct_") +
                      std::to_string(run);
  auto bus = SimNode<Complex>::make("bus", PhaseType::Single,
                                    std::vector<Complex>{Complex(voltage, 0.)});
  auto generator =
      DP::Ph1::SynchronGeneratorTrStab::make("generator", useSubcomponents);
  generator->setStandardParametersPU(1e6, voltage, frequency, 0.2, 3.0, 0.01,
                                     0.01);
  generator->setInitialValues(Complex(-5e5, 0.), 5e5);
  generator->connect({bus});
  auto load = DP::Ph1::Resistor::make("load");
  load->setParameters(voltage * voltage / 5e5);
  load->connect({SimNode<Complex>::GND, bus});
  SystemTopology system(frequency, {bus}, {generator, load});
  Simulation simulation(name, Logger::Level::off);
  simulation.setSystem(system);
  simulation.setDomain(Domain::DP);
  simulation.setTimeStep(1e-5);
  simulation.setFinalTime(0.04);
  const auto start = std::chrono::steady_clock::now();
  simulation.run();
  return std::chrono::duration<Real>(std::chrono::steady_clock::now() - start)
      .count();
}

Real runPQLoadCSCase(Bool useSubcomponents, UInt run) {
  const String name = String("DP_PQLoadCS_Benchmark_") +
                      (useSubcomponents ? "Legacy_" : "Direct_") +
                      std::to_string(run);
  auto bus = SimNode<Complex>::make("bus", PhaseType::Single);
  bus->setInitialVoltage(Complex(voltage, 0.));
  auto source = DP::Ph1::VoltageSource::make("source");
  source->setParameters(Complex(voltage, 0.), frequency);
  auto load = DP::Ph1::PQLoadCS::make("load", useSubcomponents);
  load->setParameters(1000., 500., voltage);
  source->connect({SimNode<Complex>::GND, bus});
  load->connect({bus});
  SystemTopology system(frequency, {bus}, {source, load});
  Simulation simulation(name, Logger::Level::off);
  simulation.setSystem(system);
  simulation.setDomain(Domain::DP);
  simulation.setTimeStep(timeStep);
  simulation.setFinalTime(finalTime);
  const auto start = std::chrono::steady_clock::now();
  simulation.run();
  return std::chrono::duration<Real>(std::chrono::steady_clock::now() - start)
      .count();
}

Real runThreePhaseInjectionCase(Bool useSubcomponents, UInt run) {
  const String name = String("DP_Ph3_NetworkInjection_Benchmark_") +
                      (useSubcomponents ? "Legacy_" : "Direct_") +
                      std::to_string(run);
  auto bus = SimNode<Complex>::make("bus", PhaseType::ABC);
  auto source = DP::Ph3::NetworkInjection::make("source", useSubcomponents);
  source->setParameters(
      Math::singlePhaseVariableToThreePhase(Complex(230., 0.)), frequency);
  source->connect({bus});
  SystemTopology system(frequency, {bus}, {source});
  Simulation simulation(name, Logger::Level::off);
  simulation.setSystem(system);
  simulation.setDomain(Domain::DP);
  simulation.setTimeStep(timeStep);
  simulation.setFinalTime(finalTime);
  const auto start = std::chrono::steady_clock::now();
  simulation.run();
  return std::chrono::duration<Real>(std::chrono::steady_clock::now() - start)
      .count();
}

Real runThreePhasePiLineCase(Bool useSubcomponents, UInt run) {
  const String name = String("DP_Ph3_PiLine_Benchmark_") +
                      (useSubcomponents ? "Legacy_" : "Direct_") +
                      std::to_string(run);
  auto bus0 = SimNode<Complex>::make("bus0", PhaseType::ABC);
  auto bus1 = SimNode<Complex>::make("bus1", PhaseType::ABC);
  auto source = DP::Ph3::NetworkInjection::make("source", true);
  source->setParameters(
      Math::singlePhaseVariableToThreePhase(Complex(230., 0.)), frequency);
  auto line = DP::Ph3::PiLine::make("line", useSubcomponents);
  line->setParameters(
      0.1 * Matrix::Identity(3, 3), 1e-3 * Matrix::Identity(3, 3),
      1e-6 * Matrix::Identity(3, 3), 1e-6 * Matrix::Identity(3, 3));
  auto load = DP::Ph3::Resistor::make("load");
  load->setParameters(100. * Matrix::Identity(3, 3));
  source->connect({bus0});
  line->connect({bus0, bus1});
  load->connect({bus1, SimNode<Complex>::GND});
  SystemTopology system(frequency, {bus0, bus1}, {source, line, load});
  Simulation simulation(name, Logger::Level::off);
  simulation.setSystem(system);
  simulation.setDomain(Domain::DP);
  simulation.setTimeStep(timeStep);
  simulation.setFinalTime(finalTime);
  const auto start = std::chrono::steady_clock::now();
  simulation.run();
  return std::chrono::duration<Real>(std::chrono::steady_clock::now() - start)
      .count();
}

Real runTransformerCase(Bool useSubcomponents, UInt run) {
  const String name = String("DP_Transformer_Benchmark_") +
                      (useSubcomponents ? "Legacy_" : "Direct_") +
                      std::to_string(run);
  auto bus0 = SimNode<Complex>::make("bus0", PhaseType::Single);
  auto bus1 = SimNode<Complex>::make("bus1", PhaseType::Single);
  auto source = DP::Ph1::NetworkInjection::make("source");
  source->setParameters(voltage, frequency);
  auto transformer =
      useSubcomponents
          ? DP::Ph1::Transformer::make("transformer", "transformer",
                                       Logger::Level::off, true)
          : DP::Ph1::Transformer::make("transformer", false, Logger::Level::off,
                                       true);
  transformer->setReferenceWinding(WindingReference::Primary);
  transformer->setParameters(230., 230., 10000., 1., 0., 0.1, 1e-3);
  auto load = DP::Ph1::Resistor::make("load");
  load->setParameters(100.);
  source->connect({bus0});
  transformer->connect({bus0, bus1});
  load->connect({bus1, SimNode<Complex>::GND});
  SystemTopology system(frequency, {bus0, bus1}, {source, transformer, load});
  Simulation simulation(name, Logger::Level::off);
  simulation.setSystem(system);
  simulation.setDomain(Domain::DP);
  simulation.setTimeStep(timeStep);
  simulation.setFinalTime(finalTime);
  const auto start = std::chrono::steady_clock::now();
  simulation.run();
  return std::chrono::duration<Real>(std::chrono::steady_clock::now() - start)
      .count();
}

Real runThreePhaseTransformerCase(Bool useSubcomponents, UInt run) {
  const String name = String("DP_Ph3_Transformer_Benchmark_") +
                      (useSubcomponents ? "Legacy_" : "Direct_") +
                      std::to_string(run);
  auto bus0 = SimNode<Complex>::make("bus0", PhaseType::ABC);
  auto bus1 = SimNode<Complex>::make("bus1", PhaseType::ABC);
  auto source = DP::Ph3::NetworkInjection::make("source", true);
  source->setParameters(
      Math::singlePhaseVariableToThreePhase(Complex(230., 0.)), frequency);
  auto transformer =
      useSubcomponents
          ? DP::Ph3::Transformer::make("transformer", "transformer",
                                       Logger::Level::off, true)
          : DP::Ph3::Transformer::make("transformer", false, Logger::Level::off,
                                       true);
  transformer->setReferenceWinding(WindingReference::Primary);
  transformer->setParameters(230., 230., 10000., 1., 0.,
                             0.1 * Matrix::Identity(3, 3),
                             1e-3 * Matrix::Identity(3, 3));
  auto load = DP::Ph3::Resistor::make("load");
  load->setParameters(100. * Matrix::Identity(3, 3));
  source->connect({bus0});
  transformer->connect({bus0, bus1});
  load->connect({bus1, SimNode<Complex>::GND});
  SystemTopology system(frequency, {bus0, bus1}, {source, transformer, load});
  Simulation simulation(name, Logger::Level::off);
  simulation.setSystem(system);
  simulation.setDomain(Domain::DP);
  simulation.setTimeStep(timeStep);
  simulation.setFinalTime(finalTime);
  const auto start = std::chrono::steady_clock::now();
  simulation.run();
  return std::chrono::duration<Real>(std::chrono::steady_clock::now() - start)
      .count();
}

} // namespace

int main() {
  auto pfBus = SimNode<Complex>::make("bus", PhaseType::Single);
  auto pfLoadBus = SimNode<Complex>::make("load_bus", PhaseType::Single);
  auto pfSource = SP::Ph1::NetworkInjection::make("Slack");
  pfSource->setParameters(voltage);
  pfSource->setBaseVoltage(voltage);
  pfSource->modifyPowerFlowBusType(PowerflowBusType::VD);
  auto pfShunt = SP::Ph1::Shunt::make("Shunt");
  pfShunt->setParameters(conductance, susceptance);
  pfShunt->setBaseVoltage(voltage);
  auto pfLine = SP::Ph1::PiLine::make("Line");
  pfLine->setParameters(0.01, 0.1e-3, 1e-6, 1e-6);
  pfLine->setBaseVoltage(voltage);
  auto pfAnchor = SP::Ph1::Load::make("PQAnchor");
  pfAnchor->setParameters(1., 0., voltage);
  pfAnchor->modifyPowerFlowBusType(PowerflowBusType::PQ);
  pfSource->connect({pfBus});
  pfShunt->connect({pfBus});
  pfLine->connect({pfBus, pfLoadBus});
  pfAnchor->connect({pfLoadBus});
  SystemTopology systemPF(frequency, {pfBus, pfLoadBus},
                          {pfSource, pfLine, pfShunt, pfAnchor});

  Simulation simPF("Shunt_PF", Logger::Level::off);
  simPF.setSystem(systemPF);
  simPF.setTimeStep(0.01);
  simPF.setFinalTime(0.02);
  simPF.setDomain(Domain::SP);
  simPF.setSolverType(Solver::Type::NRP);
  simPF.setSolverAndComponentBehaviour(Solver::Behaviour::Initialization);
  simPF.doInitFromNodesAndTerminals(true);
  simPF.run();

  auto dpBus = SimNode<Complex>::make("bus", PhaseType::Single);
  auto dpLoadBus = SimNode<Complex>::make("load_bus", PhaseType::Single);
  auto dpSource = DP::Ph1::VoltageSource::make("Slack");
  dpSource->setParameters(Complex(voltage, 0.), 0.);
  auto dpLine = DP::Ph1::PiLine::make("Line");
  dpLine->setParameters(0.01, 0.1e-3, 1e-6, 1e-6);
  auto dpAnchor = DP::Ph1::Resistor::make("PQAnchor");
  dpAnchor->setParameters(voltage * voltage);
  auto dpShunt = DP::Ph1::Shunt::make("Shunt");
  auto dpDirectShunt = DP::Ph1::Shunt::make("ShuntDirect", false);
  dpShunt->setParameters(conductance, susceptance);
  dpDirectShunt->setParameters(conductance, susceptance);
  dpSource->connect({SimNode<Complex>::GND, dpBus});
  dpLine->connect({dpBus, dpLoadBus});
  dpAnchor->connect({dpLoadBus, SimNode<Complex>::GND});
  dpShunt->connect({dpBus});
  dpDirectShunt->connect({dpBus});
  SystemTopology systemDP(frequency, {dpBus, dpLoadBus},
                          {dpSource, dpLine, dpAnchor, dpShunt, dpDirectShunt});
  systemDP.initWithPowerflow(systemPF, Domain::DP);

  std::filesystem::create_directories("logs/DP_Ph1_Shunt_PF_Init");
  Logger::setLogDir("logs/DP_Ph1_Shunt_PF_Init");
  auto logDP = DataLogger::make(String("DP_Ph1_Shunt_PF_Init"));
  logDP->logAttribute("v_bus", dpBus->attribute("v"));
  logDP->logAttribute("i_shunt", dpShunt->attribute("i_intf"));
  logDP->logAttribute("i_shunt_direct", dpDirectShunt->attribute("i_intf"));
  Simulation simDP("DP_Ph1_Shunt_PF_Init", Logger::Level::off);
  simDP.setSystem(systemDP);
  simDP.setDomain(Domain::DP);
  simDP.setTimeStep(timeStep);
  simDP.setFinalTime(finalTime);
  simDP.addLogger(logDP);
  simDP.run();

  auto emtBus = SimNode<Real>::make("bus", PhaseType::ABC);
  auto emtLoadBus = SimNode<Real>::make("load_bus", PhaseType::ABC);
  auto emtSource = EMT::Ph3::VoltageSource::make("Slack");
  emtSource->setParameters(
      Math::singlePhaseVariableToThreePhase(Complex(voltage, 0.)), frequency);
  auto emtShunt = EMT::Ph3::Shunt::make("Shunt");
  auto emtLine = EMT::Ph3::PiLine::make("Line");
  emtLine->setParameters(
      0.01 * Matrix::Identity(3, 3), 0.1e-3 * Matrix::Identity(3, 3),
      1e-6 * Matrix::Identity(3, 3), 1e-6 * Matrix::Identity(3, 3));
  auto emtAnchor = EMT::Ph3::PQLoad::make("PQAnchor");
  emtSource->connect({SimNode<Real>::GND, emtBus});
  emtLine->connect({emtBus, emtLoadBus});
  emtAnchor->connect({emtLoadBus});
  emtShunt->connect({emtBus});
  SystemTopology systemEMT(frequency, {emtBus, emtLoadBus},
                           {emtSource, emtLine, emtAnchor, emtShunt});
  systemEMT.initWithPowerflow(systemPF, Domain::EMT);

  std::filesystem::create_directories("logs/EMT_Ph3_Shunt_PF_Init");
  Logger::setLogDir("logs/EMT_Ph3_Shunt_PF_Init");
  auto logEMT = DataLogger::make(String("EMT_Ph3_Shunt_PF_Init"));
  logEMT->logAttribute("v_bus", emtBus->attribute("v"));
  logEMT->logAttribute("i_shunt", emtShunt->attribute("i_intf"));
  Simulation simEMT("EMT_Ph3_Shunt_PF_Init", Logger::Level::off);
  simEMT.setSystem(systemEMT);
  simEMT.setDomain(Domain::EMT);
  simEMT.setTimeStep(timeStep);
  simEMT.setFinalTime(finalTime);
  simEMT.addLogger(logEMT);
  simEMT.run();

  const Csv dp = readCsv("DP_Ph1_Shunt_PF_Init");
  const Csv emt = readCsv("EMT_Ph3_Shunt_PF_Init");
  Real directCurrentError = 0.;
  for (size_t row = 0; row < dp.rows.size(); ++row) {
    const Complex legacyCurrent(at(dp, row, "i_shunt.re"),
                                at(dp, row, "i_shunt.im"));
    const Complex directCurrent(at(dp, row, "i_shunt_direct.re"),
                                at(dp, row, "i_shunt_direct.im"));
    directCurrentError = std::max(directCurrentError,
                                  std::abs(legacyCurrent - directCurrent) /
                                      std::max(1e-12, std::abs(legacyCurrent)));
  }
  const Complex dpCurrent(at(dp, dp.rows.size() - 1, "i_shunt.re"),
                          at(dp, dp.rows.size() - 1, "i_shunt.im"));
  const Complex emtCurrent = fundamental(emt, "i_shunt_0", 0.02, finalTime);
  const Complex expectedEmtCurrent = -RMS3PH_TO_PEAK1PH * dpCurrent;
  const Real parityError = std::abs(expectedEmtCurrent - emtCurrent) /
                           std::max(1e-12, std::abs(expectedEmtCurrent));
  const Complex pfVoltage = pfBus->singleVoltage();
  const Complex expectedDPVoltage = pfVoltage;
  const Complex measuredDPVoltage(at(dp, 0, "v_bus.re"), at(dp, 0, "v_bus.im"));
  const Real initError = std::abs(measuredDPVoltage - expectedDPVoltage) /
                         std::max(1e-12, std::abs(expectedDPVoltage));
  Real firstCyclePeakError = 0.;
  for (const auto &row : emt.rows) {
    const Real time = row[emt.columns.at("time")];
    if (time >= 0.02)
      break;
    const Real expected = std::real(
        emtCurrent * std::polar<Real>(1., 2. * PI * frequency * time));
    firstCyclePeakError =
        std::max(firstCyclePeakError,
                 std::abs(row[emt.columns.at("i_shunt_0")] - expected));
  }
  std::cout << "DP Ph1 / EMT Ph3 shunt current relative error: " << parityError
            << "\nDP legacy/direct shunt current relative error: "
            << directCurrentError
            << "\nInitial DP/PF voltage relative error: " << initError
            << "\nFirst-cycle EMT shunt current peak deviation: "
            << firstCyclePeakError << " A\n";
  if (parityError > 1e-3 || directCurrentError > 1e-10 || initError > 1e-6 ||
      firstCyclePeakError > 1e-3)
    throw std::runtime_error("SP/DP/EMT shunt PF initialization failed");

  const Csv rxLoad = runRXLoadComparison();
  Real rxLoadCurrentError = 0.;
  for (size_t row = 0; row < rxLoad.rows.size(); ++row) {
    const Complex legacyCurrent(at(rxLoad, row, "legacy_current.re"),
                                at(rxLoad, row, "legacy_current.im"));
    const Complex directCurrent(at(rxLoad, row, "direct_current.re"),
                                at(rxLoad, row, "direct_current.im"));
    rxLoadCurrentError = std::max(rxLoadCurrentError,
                                  std::abs(legacyCurrent - directCurrent) /
                                      std::max(1e-12, std::abs(legacyCurrent)));
  }
  std::cout << "DP RXLoad legacy/direct full-trace relative error: "
            << rxLoadCurrentError << "\n";
  if (rxLoadCurrentError > 1e-10)
    throw std::runtime_error("DP RXLoad direct trace comparison failed");

  const Csv rxLoadSwitchLegacy =
      runRXLoadSwitchCase(true, "DP_RXLoadSwitch_LegacyCompare");
  const Csv rxLoadSwitchDirect =
      runRXLoadSwitchCase(false, "DP_RXLoadSwitch_DirectCompare");
  Real rxLoadSwitchVoltageError = 0.;
  Real rxLoadSwitchCurrentError = 0.;
  for (size_t row = 0; row < rxLoadSwitchLegacy.rows.size(); ++row) {
    for (const String &quantity : {String("voltage"), String("current")}) {
      Real &error = quantity == "voltage" ? rxLoadSwitchVoltageError
                                          : rxLoadSwitchCurrentError;
      const Complex legacyValue(at(rxLoadSwitchLegacy, row, quantity + ".re"),
                                at(rxLoadSwitchLegacy, row, quantity + ".im"));
      const Complex directValue(at(rxLoadSwitchDirect, row, quantity + ".re"),
                                at(rxLoadSwitchDirect, row, quantity + ".im"));
      error = std::max(error, std::abs(legacyValue - directValue) /
                                  std::max(1e-12, std::abs(legacyValue)));
    }
  }
  std::cout << "DP RXLoadSwitch full-trace relative errors: voltage="
            << rxLoadSwitchVoltageError
            << ", current=" << rxLoadSwitchCurrentError << "\n";
  if (rxLoadSwitchVoltageError > 1e-6 || rxLoadSwitchCurrentError > 1e-6)
    throw std::runtime_error("DP RXLoadSwitch direct trace comparison failed");

  const Csv generatorLegacy = runTransientGeneratorTrace(true, 0);
  const Csv generatorDirect = runTransientGeneratorTrace(false, 0);
  Real generatorVoltageError = 0.;
  Real generatorCurrentError = 0.;
  Real generatorSpeedError = 0.;
  for (size_t row = 0; row < generatorLegacy.rows.size(); ++row) {
    for (const String &part : {String("re"), String("im")}) {
      generatorVoltageError =
          std::max(generatorVoltageError,
                   std::abs(at(generatorLegacy, row, "voltage." + part) -
                            at(generatorDirect, row, "voltage." + part)) /
                       voltage);
      generatorCurrentError =
          std::max(generatorCurrentError,
                   std::abs(at(generatorLegacy, row, "current." + part) -
                            at(generatorDirect, row, "current." + part)) /
                       1250.);
    }
    generatorSpeedError = std::max(generatorSpeedError,
                                   std::abs(at(generatorLegacy, row, "speed") -
                                            at(generatorDirect, row, "speed")) /
                                       (2. * PI * frequency));
  }
  std::cout << "DP TrStab generator base-scaled trace errors: voltage="
            << generatorVoltageError << ", current=" << generatorCurrentError
            << ", speed=" << generatorSpeedError << "\n";
  if (generatorVoltageError > 2e-3 || generatorCurrentError > 2e-3 ||
      generatorSpeedError > 2e-3)
    throw std::runtime_error("DP TrStab generator direct trace mismatch");

  const Csv injection = runNetworkInjectionComparison();
  Real injectionVoltageError = 0.;
  Real injectionCurrentError = 0.;
  for (size_t row = 0; row < injection.rows.size(); ++row) {
    for (const String &quantity : {String("voltage"), String("current")}) {
      const Complex legacyValue(
          at(injection, row, "legacy_" + quantity + ".re"),
          at(injection, row, "legacy_" + quantity + ".im"));
      const Complex directValue(
          at(injection, row, "direct_" + quantity + ".re"),
          at(injection, row, "direct_" + quantity + ".im"));
      const Real error = std::abs(legacyValue - directValue) /
                         std::max(1e-12, std::abs(legacyValue));
      if (quantity == "voltage")
        injectionVoltageError = std::max(injectionVoltageError, error);
      else
        injectionCurrentError = std::max(injectionCurrentError, error);
    }
  }
  std::cout << "DP NetworkInjection full-trace relative errors: voltage="
            << injectionVoltageError << ", current=" << injectionCurrentError
            << "\n";
  if (injectionVoltageError > 1e-10 || injectionCurrentError > 1e-10)
    throw std::runtime_error("DP NetworkInjection direct trace failed");

  const Csv piLine = runPiLineComparison();
  Real piLineVoltageError = 0.;
  Real piLineCurrentError = 0.;
  for (size_t row = 0; row < piLine.rows.size(); ++row) {
    const Complex legacyVoltage(at(piLine, row, "legacy_voltage.re"),
                                at(piLine, row, "legacy_voltage.im"));
    const Complex directVoltage(at(piLine, row, "direct_voltage.re"),
                                at(piLine, row, "direct_voltage.im"));
    const Complex legacyCurrent(at(piLine, row, "legacy_current.re"),
                                at(piLine, row, "legacy_current.im"));
    const Complex directCurrent(at(piLine, row, "direct_current.re"),
                                at(piLine, row, "direct_current.im"));
    piLineVoltageError =
        std::max(piLineVoltageError, std::abs(legacyVoltage - directVoltage) /
                                         std::max(1., std::abs(legacyVoltage)));
    piLineCurrentError =
        std::max(piLineCurrentError, std::abs(legacyCurrent - directCurrent) /
                                         std::max(1., std::abs(legacyCurrent)));
  }
  std::cout << "DP PiLine full-trace relative errors: voltage="
            << piLineVoltageError << ", current=" << piLineCurrentError << "\n";
  if (piLineVoltageError > 1e-5 || piLineCurrentError > 1e-5)
    throw std::runtime_error("DP PiLine direct trace comparison failed");

  const Csv rxLine = runRxLineComparison();
  Real rxLineVoltageError = 0.;
  Real rxLineCurrentError = 0.;
  for (size_t row = 0; row < rxLine.rows.size(); ++row) {
    const Complex legacyVoltage(at(rxLine, row, "legacy_voltage.re"),
                                at(rxLine, row, "legacy_voltage.im"));
    const Complex directVoltage(at(rxLine, row, "direct_voltage.re"),
                                at(rxLine, row, "direct_voltage.im"));
    const Complex legacyCurrent(at(rxLine, row, "legacy_current.re"),
                                at(rxLine, row, "legacy_current.im"));
    const Complex directCurrent(at(rxLine, row, "direct_current.re"),
                                at(rxLine, row, "direct_current.im"));
    rxLineVoltageError = std::max(rxLineVoltageError,
                                  std::abs(legacyVoltage - directVoltage) /
                                      std::max(1e-12, std::abs(legacyVoltage)));
    rxLineCurrentError = std::max(rxLineCurrentError,
                                  std::abs(legacyCurrent - directCurrent) /
                                      std::max(1e-12, std::abs(legacyCurrent)));
  }
  std::cout << "DP RxLine full-trace relative errors: voltage="
            << rxLineVoltageError << ", current=" << rxLineCurrentError << "\n";
  if (rxLineVoltageError > 1e-8 || rxLineCurrentError > 1e-8)
    throw std::runtime_error("DP RxLine direct trace comparison failed");

  for (Bool generator : {false, true}) {
    const Csv source = runSourceWrapperComparison(generator);
    Real voltageError = 0.;
    Real currentError = 0.;
    for (size_t row = 0; row < source.rows.size(); ++row) {
      const Complex legacyVoltage(at(source, row, "legacy_voltage.re"),
                                  at(source, row, "legacy_voltage.im"));
      const Complex directVoltage(at(source, row, "direct_voltage.re"),
                                  at(source, row, "direct_voltage.im"));
      const Complex legacyCurrent(at(source, row, "legacy_current.re"),
                                  at(source, row, "legacy_current.im"));
      const Complex directCurrent(at(source, row, "direct_current.re"),
                                  at(source, row, "direct_current.im"));
      voltageError =
          std::max(voltageError, std::abs(legacyVoltage - directVoltage) /
                                     std::max(1e-12, std::abs(legacyVoltage)));
      currentError =
          std::max(currentError, std::abs(legacyCurrent - directCurrent) /
                                     std::max(1e-12, std::abs(legacyCurrent)));
    }
    std::cout << (generator ? "DP ideal generator" : "DP voltage ramp")
              << " full-trace relative errors: voltage=" << voltageError
              << ", current=" << currentError << "\n";
    if (voltageError > 1e-8 || currentError > 1e-8)
      throw std::runtime_error("DP voltage source wrapper trace mismatch");
  }

  const Csv pqLoad = runPQLoadCSComparison();
  Real pqLoadVoltageError = 0.;
  Real pqLoadCurrentError = 0.;
  for (size_t row = 0; row < pqLoad.rows.size(); ++row) {
    const Complex legacyVoltage(at(pqLoad, row, "legacy_voltage.re"),
                                at(pqLoad, row, "legacy_voltage.im"));
    const Complex directVoltage(at(pqLoad, row, "direct_voltage.re"),
                                at(pqLoad, row, "direct_voltage.im"));
    const Complex legacyCurrent(at(pqLoad, row, "legacy_current.re"),
                                at(pqLoad, row, "legacy_current.im"));
    const Complex directCurrent(at(pqLoad, row, "direct_current.re"),
                                at(pqLoad, row, "direct_current.im"));
    pqLoadVoltageError = std::max(pqLoadVoltageError,
                                  std::abs(legacyVoltage - directVoltage) /
                                      std::max(1e-12, std::abs(legacyVoltage)));
    pqLoadCurrentError = std::max(pqLoadCurrentError,
                                  std::abs(legacyCurrent - directCurrent) /
                                      std::max(1e-12, std::abs(legacyCurrent)));
  }
  std::cout << "DP PQLoadCS full-trace relative errors: voltage="
            << pqLoadVoltageError << ", current=" << pqLoadCurrentError << "\n";
  if (pqLoadVoltageError > 1e-8 || pqLoadCurrentError > 1e-8)
    throw std::runtime_error("DP PQLoadCS direct trace comparison failed");

  const Csv svc = runSVCComparison();
  Real svcVoltageError = 0.;
  Real svcCurrentError = 0.;
  for (size_t row = 0; row < svc.rows.size(); ++row) {
    for (const auto &[quantity, error] :
         {std::pair<String, Real *>("voltage", &svcVoltageError),
          std::pair<String, Real *>("current", &svcCurrentError)}) {
      const Complex legacyValue(at(svc, row, "legacy_" + quantity + ".re"),
                                at(svc, row, "legacy_" + quantity + ".im"));
      const Complex directValue(at(svc, row, "direct_" + quantity + ".re"),
                                at(svc, row, "direct_" + quantity + ".im"));
      *error = std::max(*error, std::abs(legacyValue - directValue) /
                                    std::max(1e-12, std::abs(legacyValue)));
    }
  }
  std::cout << "DP SVC initial-trace relative errors: voltage="
            << svcVoltageError << ", current=" << svcCurrentError << "\n";
  if (svcVoltageError > 1e-6 || svcCurrentError > 1e-6)
    throw std::runtime_error("DP SVC direct initial trace comparison failed");

  const Csv inverterLegacy = runDPInverterCase(true, "DP_AvVSI_LegacyCompare");
  const Csv inverterDirect = runDPInverterCase(false, "DP_AvVSI_DirectCompare");
  Real inverterVoltageError = 0.;
  Real inverterCurrentError = 0.;
  for (size_t row = 0; row < inverterLegacy.rows.size(); ++row) {
    for (const String &quantity : {String("voltage"), String("current")}) {
      Real &error =
          quantity == "voltage" ? inverterVoltageError : inverterCurrentError;
      const Complex legacyValue(at(inverterLegacy, row, quantity + ".re"),
                                at(inverterLegacy, row, quantity + ".im"));
      const Complex directValue(at(inverterDirect, row, quantity + ".re"),
                                at(inverterDirect, row, quantity + ".im"));
      const Real base = quantity == "voltage" ? voltage : 10.;
      error = std::max(error, std::abs(legacyValue - directValue) / base);
    }
  }
  std::cout << "DP AvVSI base-scaled trace errors: voltage="
            << inverterVoltageError << ", current=" << inverterCurrentError
            << "\n";
  if (inverterVoltageError > 2e-3 || inverterCurrentError > 2e-3)
    throw std::runtime_error("DP AvVSI direct trace comparison failed");

  const Csv inverterControlledLegacy = runDPInverterCase(
      true, "DP_AvVSI_Controlled_LegacyCompare", nullptr, true, 0.005);
  const Csv inverterControlledDirect = runDPInverterCase(
      false, "DP_AvVSI_Controlled_DirectCompare", nullptr, true, 0.005);
  Real inverterControlledVoltageError = 0.;
  Real inverterControlledCurrentError = 0.;
  for (size_t row = 0; row < inverterControlledLegacy.rows.size(); ++row) {
    for (const String &quantity : {String("voltage"), String("current")}) {
      Real &error = quantity == "voltage" ? inverterControlledVoltageError
                                          : inverterControlledCurrentError;
      const Complex legacyValue(
          at(inverterControlledLegacy, row, quantity + ".re"),
          at(inverterControlledLegacy, row, quantity + ".im"));
      const Complex directValue(
          at(inverterControlledDirect, row, quantity + ".re"),
          at(inverterControlledDirect, row, quantity + ".im"));
      const Real base = quantity == "voltage" ? voltage : 10.;
      error = std::max(error, std::abs(legacyValue - directValue) / base);
    }
  }
  std::cout << "DP AvVSI controlled 5 ms base-scaled errors: voltage="
            << inverterControlledVoltageError
            << ", current=" << inverterControlledCurrentError << "\n";
  if (inverterControlledVoltageError > 2e-3 ||
      inverterControlledCurrentError > 2e-3)
    throw std::runtime_error(
        "DP AvVSI controlled direct trace comparison failed");

  const Csv inverterTransformerLegacy = runDPInverterCase(
      true, "DP_AvVSI_Transformer_LegacyCompare", nullptr, false, 0.005, true);
  const Csv inverterTransformerDirect = runDPInverterCase(
      false, "DP_AvVSI_Transformer_DirectCompare", nullptr, false, 0.005, true);
  Real inverterTransformerVoltageError = 0.;
  Real inverterTransformerCurrentError = 0.;
  for (size_t row = 0; row < inverterTransformerLegacy.rows.size(); ++row) {
    for (const String &quantity : {String("voltage"), String("current")}) {
      Real &error = quantity == "voltage" ? inverterTransformerVoltageError
                                          : inverterTransformerCurrentError;
      const Complex legacyValue(
          at(inverterTransformerLegacy, row, quantity + ".re"),
          at(inverterTransformerLegacy, row, quantity + ".im"));
      const Complex directValue(
          at(inverterTransformerDirect, row, quantity + ".re"),
          at(inverterTransformerDirect, row, quantity + ".im"));
      const Real base = quantity == "voltage" ? voltage : 10.;
      const Real scale =
          quantity == "current" ? std::max(base, std::abs(legacyValue)) : base;
      error = std::max(error, std::abs(legacyValue - directValue) / scale);
    }
  }
  std::cout << "DP AvVSI transformer direct trace errors: voltage="
            << inverterTransformerVoltageError
            << ", current=" << inverterTransformerCurrentError << "\n";
  if (inverterTransformerVoltageError > 2e-3 ||
      inverterTransformerCurrentError > 2e-3)
    throw std::runtime_error(
        "DP AvVSI transformer direct trace comparison failed");

  const Csv networkInjection3ph = runThreePhaseNetworkInjectionComparison();
  Real networkInjection3phError = 0.;
  for (size_t row = 0; row < networkInjection3ph.rows.size(); ++row)
    for (const String &quantity : {String("voltage"), String("current")})
      for (UInt phase = 0; phase < 3; ++phase)
        for (const String &part : {String("re"), String("im")}) {
          const String suffix = "_" + std::to_string(phase) + "." + part;
          const Real legacyValue =
              at(networkInjection3ph, row, "legacy_" + quantity + suffix);
          const Real directValue =
              at(networkInjection3ph, row, "direct_" + quantity + suffix);
          networkInjection3phError =
              std::max(networkInjection3phError,
                       std::abs(legacyValue - directValue) /
                           std::max(1., std::abs(legacyValue)));
        }
  std::cout << "DP Ph3 NetworkInjection full-trace max relative error: "
            << networkInjection3phError << "\n";
  if (networkInjection3phError > 1e-8)
    throw std::runtime_error("DP Ph3 NetworkInjection direct trace mismatch");

  const Csv piLine3ph = runThreePhasePiLineComparison();
  Real piLine3phError = 0.;
  for (size_t row = 0; row < piLine3ph.rows.size(); ++row)
    for (const String &quantity : {String("voltage"), String("current")})
      for (UInt phase = 0; phase < 3; ++phase)
        for (const String &part : {String("re"), String("im")}) {
          const String suffix = "_" + std::to_string(phase) + "." + part;
          const Real legacyValue =
              at(piLine3ph, row, "legacy_" + quantity + suffix);
          const Real directValue =
              at(piLine3ph, row, "direct_" + quantity + suffix);
          piLine3phError =
              std::max(piLine3phError, std::abs(legacyValue - directValue) /
                                           std::max(1., std::abs(legacyValue)));
        }
  std::cout << "DP Ph3 PiLine full-trace max relative error: " << piLine3phError
            << "\n";
  if (piLine3phError > 1e-8)
    throw std::runtime_error("DP Ph3 PiLine direct trace mismatch");

  const Csv transformer = runTransformerComparison();
  Real transformerVoltageError = 0.;
  Real transformerCurrentError = 0.;
  for (size_t row = 0; row < transformer.rows.size(); ++row) {
    for (const auto &quantity : {String("voltage"), String("current")}) {
      const Real legacyValue =
          at(transformer, row, "legacy_" + quantity + ".re");
      const Real directValue =
          at(transformer, row, "direct_" + quantity + ".re");
      const Real base = quantity == "voltage" ? 230. : 6.;
      const Real error = std::abs(legacyValue - directValue) / base;
      (quantity == "voltage" ? transformerVoltageError
                             : transformerCurrentError) =
          std::max(quantity == "voltage" ? transformerVoltageError
                                         : transformerCurrentError,
                   error);
    }
  }
  std::cout
      << "DP Transformer full-trace base-scaled errors (230 V, 6 A): voltage="
      << transformerVoltageError << ", current=" << transformerCurrentError
      << "\n";
  if (transformerVoltageError > 2e-3 || transformerCurrentError > 2e-3)
    throw std::runtime_error("DP Transformer direct trace mismatch");

  const Csv transformer3ph = runThreePhaseTransformerComparison();
  Real transformer3phVoltageError = 0.;
  Real transformer3phCurrentError = 0.;
  for (size_t row = 0; row < transformer3ph.rows.size(); ++row)
    for (const auto &quantity : {String("voltage"), String("current")})
      for (UInt phase = 0; phase < 3; ++phase)
        for (const auto &part : {String("re"), String("im")}) {
          const String suffix = "_" + std::to_string(phase) + "." + part;
          const Real legacyValue =
              at(transformer3ph, row, "legacy_" + quantity + suffix);
          const Real directValue =
              at(transformer3ph, row, "direct_" + quantity + suffix);
          const Real error = std::abs(legacyValue - directValue) /
                             (quantity == "voltage" ? 230. : 6.);
          (quantity == "voltage" ? transformer3phVoltageError
                                 : transformer3phCurrentError) =
              std::max(quantity == "voltage" ? transformer3phVoltageError
                                             : transformer3phCurrentError,
                       error);
        }
  std::cout << "DP Ph3 Transformer base-scaled trace errors: voltage="
            << transformer3phVoltageError
            << ", current=" << transformer3phCurrentError << "\n";
  if (transformer3phVoltageError > 2e-3 || transformer3phCurrentError > 2e-3)
    throw std::runtime_error("DP Ph3 Transformer direct trace mismatch");

  constexpr UInt timingRuns = 5;
  Real injectionLegacyTime = 0.;
  Real injectionDirectTime = 0.;
  for (UInt run = 0; run < timingRuns; ++run) {
    injectionLegacyTime += runNetworkInjectionCase(true, run);
    injectionDirectTime += runNetworkInjectionCase(false, run);
  }
  std::cout << "DP NetworkInjection benchmark (5-run mean, 0.04 s at 10 us): "
            << "legacy=" << injectionLegacyTime / timingRuns
            << " s, direct=" << injectionDirectTime / timingRuns << " s\n";

  Real rxLoadLegacyTime = 0.;
  Real rxLoadDirectTime = 0.;
  for (UInt run = 0; run < timingRuns; ++run) {
    rxLoadLegacyTime += runRXLoadCase(true, run);
    rxLoadDirectTime += runRXLoadCase(false, run);
  }
  std::cout << "DP RXLoad benchmark (5-run mean, 0.04 s at 10 us): legacy="
            << rxLoadLegacyTime / timingRuns
            << " s, direct=" << rxLoadDirectTime / timingRuns << " s\n";

  Real legacyTime = 0.;
  Real directTime = 0.;
  for (UInt run = 0; run < timingRuns; ++run) {
    legacyTime += runShuntCase(true, run);
    directTime += runShuntCase(false, run);
  }
  std::cout << "DP shunt benchmark (5-run mean, 0.04 s at 10 us): legacy="
            << legacyTime / timingRuns
            << " s, direct=" << directTime / timingRuns << " s\n";

  const auto benchmarkDirectPair = [&](const String &label, auto runCase) {
    Real legacy = 0.;
    Real direct = 0.;
    for (UInt run = 0; run < timingRuns; ++run) {
      legacy += runCase(true, run);
      direct += runCase(false, run);
    }
    std::cout << label << " benchmark (5-run mean, 0.04 s at 10 us): legacy="
              << legacy / timingRuns << " s, direct=" << direct / timingRuns
              << " s\n";
  };
  benchmarkDirectPair("DP PiLine", [&](Bool use, UInt run) {
    return runLineCase(true, use, run);
  });
  benchmarkDirectPair("DP RxLine", [&](Bool use, UInt run) {
    return runLineCase(false, use, run);
  });
  benchmarkDirectPair("DP voltage ramp", [&](Bool use, UInt run) {
    return runSourceWrapperCase(false, use, run);
  });
  benchmarkDirectPair("DP ideal generator", [&](Bool use, UInt run) {
    return runSourceWrapperCase(true, use, run);
  });
  benchmarkDirectPair("DP TrStab generator", [&](Bool use, UInt run) {
    return runTransientGeneratorBenchmark(use, run);
  });
  benchmarkDirectPair("DP PQLoadCS", [&](Bool use, UInt run) {
    return runPQLoadCSCase(use, run);
  });
  benchmarkDirectPair("DP Ph3 NetworkInjection", [&](Bool use, UInt run) {
    return runThreePhaseInjectionCase(use, run);
  });
  benchmarkDirectPair("DP Ph3 PiLine", [&](Bool use, UInt run) {
    return runThreePhasePiLineCase(use, run);
  });
  benchmarkDirectPair("DP RXLoadSwitch", [&](Bool use, UInt run) {
    const String name = String("DP_RXLoadSwitch_Benchmark_") +
                        (use ? "Legacy_" : "Direct_") + std::to_string(run);
    Real elapsed = 0.;
    runRXLoadSwitchCase(use, name, &elapsed);
    return elapsed;
  });
  benchmarkDirectPair("DP Transformer", [&](Bool use, UInt run) {
    return runTransformerCase(use, run);
  });
  benchmarkDirectPair("DP Ph3 Transformer", [&](Bool use, UInt run) {
    return runThreePhaseTransformerCase(use, run);
  });
  Real svcLegacyTime = 0.;
  Real svcDirectTime = 0.;
  for (UInt run = 0; run < timingRuns; ++run) {
    svcLegacyTime += runSVCBenchmarkCase(true, run);
    svcDirectTime += runSVCBenchmarkCase(false, run);
  }
  std::cout << "DP SVC benchmark (5-run mean, 0.04 s at 10 us): legacy="
            << svcLegacyTime / timingRuns
            << " s, direct=" << svcDirectTime / timingRuns << " s\n";

  Real inverterLegacyTime = 0.;
  Real inverterDirectTime = 0.;
  for (UInt run = 0; run < timingRuns; ++run) {
    Real elapsed = 0.;
    runDPInverterCase(true, "DP_AvVSI_LegacyBenchmark_" + std::to_string(run),
                      &elapsed);
    inverterLegacyTime += elapsed;
    runDPInverterCase(false, "DP_AvVSI_DirectBenchmark_" + std::to_string(run),
                      &elapsed);
    inverterDirectTime += elapsed;
  }
  std::cout << "DP AvVSI benchmark (5-run mean, 0.04 s at 10 us): legacy="
            << inverterLegacyTime / timingRuns
            << " s, direct=" << inverterDirectTime / timingRuns << " s\n";

  Real inverterTransformerLegacyTime = 0.;
  Real inverterTransformerDirectTime = 0.;
  for (UInt run = 0; run < timingRuns; ++run) {
    Real elapsed = 0.;
    runDPInverterCase(
        true, "DP_AvVSI_Transformer_LegacyBenchmark_" + std::to_string(run),
        &elapsed, false, 0.04, true);
    inverterTransformerLegacyTime += elapsed;
    runDPInverterCase(
        false, "DP_AvVSI_Transformer_DirectBenchmark_" + std::to_string(run),
        &elapsed, false, 0.04, true);
    inverterTransformerDirectTime += elapsed;
  }
  std::cout << "DP AvVSI with transformer benchmark (5-run mean, 0.04 s at "
               "10 us): legacy="
            << inverterTransformerLegacyTime / timingRuns
            << " s, direct=" << inverterTransformerDirectTime / timingRuns
            << " s\n";
}
