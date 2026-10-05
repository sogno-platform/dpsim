// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <DPsim.h>

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
constexpr Real susceptance = 0.0;
constexpr Real timeStep = 1e-5;
constexpr Real finalTime = 0.04;

struct Csv {
  std::vector<std::vector<Real>> rows;
  std::map<String, size_t> columns;
};

Csv readCsv(const String &name) {
  const String path = "logs/" + name + "/" + name + ".csv";
  std::ifstream file(path);
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
  dpSource->connect({SimNode<Complex>::GND, dpBus});
  dpLine->connect({dpBus, dpLoadBus});
  dpAnchor->connect({dpLoadBus, SimNode<Complex>::GND});
  dpShunt->connect({dpBus});
  SystemTopology systemDP(frequency, {dpBus, dpLoadBus},
                          {dpSource, dpLine, dpAnchor, dpShunt});
  systemDP.initWithPowerflow(systemPF, Domain::DP);

  std::filesystem::create_directories("logs/DP_Ph1_Shunt_PF_Init");
  Logger::setLogDir("logs/DP_Ph1_Shunt_PF_Init");
  auto logDP = DataLogger::make(String("DP_Ph1_Shunt_PF_Init"));
  logDP->logAttribute("v_bus", dpBus->attribute("v"));
  logDP->logAttribute("i_shunt", dpShunt->attribute("i_intf"));
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
            << "\nInitial DP/PF voltage relative error: " << initError
            << "\nFirst-cycle EMT shunt current peak deviation: "
            << firstCyclePeakError << " A\n";
  if (parityError > 1e-3 || initError > 1e-6 || firstCyclePeakError > 1e-3)
    throw std::runtime_error("SP/DP/EMT shunt PF initialization failed");
}
