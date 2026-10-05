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
constexpr Real voltageLL = 400.;
constexpr Real pLoad = 10e3;
constexpr Real qLoad = 5e3;
constexpr Real linkResistance = 0.12;
constexpr Real timeStep = 1e-5;
constexpr Real finalTime = 0.04;

struct Csv {
  std::vector<String> header;
  std::vector<std::vector<Real>> rows;
  std::map<String, size_t> column;

  Real at(size_t row, const String &name) const {
    return rows.at(row).at(column.at(name));
  }
};

Csv readCsv(const String &name) {
  const String path = "logs/" + name + "/" + name + ".csv";
  std::ifstream file(path);
  if (!file)
    throw std::runtime_error("Could not open validation output " + path);

  Csv data;
  String line;
  std::getline(file, line);
  std::stringstream header(line);
  String cell;
  while (std::getline(header, cell, ',')) {
    while (!cell.empty() && cell.front() == ' ')
      cell.erase(cell.begin());
    while (!cell.empty() && cell.back() == ' ')
      cell.pop_back();
    data.column[cell] = data.header.size();
    data.header.push_back(cell);
  }

  while (std::getline(file, line)) {
    if (line.empty())
      continue;
    std::stringstream rowStream(line);
    std::vector<Real> row;
    while (std::getline(rowStream, cell, ','))
      row.push_back(std::stod(cell));
    data.rows.push_back(std::move(row));
  }
  return data;
}

Complex fundamental(const Csv &data, const String &columnName, Real t0,
                    Real t1) {
  Complex phasor = 0.;
  size_t count = 0;
  for (const auto &row : data.rows) {
    const Real time = row[data.column.at("time")];
    if (time < t0 || time >= t1)
      continue;
    const Real sample = row[data.column.at(columnName)];
    const Real angle = 2. * PI * frequency * time;
    phasor +=
        Complex(2. * sample * std::cos(angle), -2. * sample * std::sin(angle));
    ++count;
  }
  if (!count)
    throw std::runtime_error("No samples in Fourier window for " + columnName);
  return phasor / static_cast<Real>(count);
}

} // namespace

int main() {
  // The real PF topology is held here so both dynamic domains start from the
  // exact same voltage and terminal power.
  auto sourceNodePF = SimNode<Complex>::make("source_bus", PhaseType::Single);
  auto loadNodePF = SimNode<Complex>::make("load_bus", PhaseType::Single);
  auto slackPF = SP::Ph1::NetworkInjection::make("Slack");
  slackPF->setParameters(voltageLL);
  slackPF->setBaseVoltage(voltageLL);
  slackPF->modifyPowerFlowBusType(PowerflowBusType::VD);
  auto loadPF = SP::Ph1::Load::make("Load");
  loadPF->setParameters(pLoad, qLoad, voltageLL);
  loadPF->modifyPowerFlowBusType(PowerflowBusType::PQ);
  auto linePF = SP::Ph1::PiLine::make("Line");
  linePF->setParameters(linkResistance, 0., 0.);
  linePF->setBaseVoltage(voltageLL);
  slackPF->connect({sourceNodePF});
  linePF->connect({sourceNodePF, loadNodePF});
  loadPF->connect({loadNodePF});
  SystemTopology systemPF(frequency, {sourceNodePF, loadNodePF},
                          {slackPF, linePF, loadPF});

  Simulation simPF("CurrentSource_PF", Logger::Level::off);
  simPF.setSystem(systemPF);
  simPF.setTimeStep(0.01);
  simPF.setFinalTime(0.02);
  simPF.setDomain(Domain::SP);
  simPF.setSolverType(Solver::Type::NRP);
  simPF.setSolverAndComponentBehaviour(Solver::Behaviour::Initialization);
  simPF.doInitFromNodesAndTerminals(true);
  simPF.run();

  // These EMT-only controlled-source classes must also derive their initial
  // references from the matching PF devices without caller-side assignments.
  auto controlVoltageSource = EMT::Ph3::ControlledVoltageSource::make("Line");
  auto controlVoltageBus0 = SimNode<Real>::make("source_bus", PhaseType::ABC);
  auto controlVoltageBus1 = SimNode<Real>::make("load_bus", PhaseType::ABC);
  controlVoltageSource->connect({controlVoltageBus0, controlVoltageBus1});
  SystemTopology controlVoltageSystem(frequency,
                                      {controlVoltageBus0, controlVoltageBus1},
                                      {controlVoltageSource});
  controlVoltageSystem.initWithPowerflow(systemPF, Domain::EMT);
  controlVoltageSource->initializeFromNodesAndTerminals(frequency);
  const Matrix expectedControlledVoltage =
      (RMS3PH_TO_PEAK1PH *
       Math::singlePhaseVariableToThreePhase(loadNodePF->singleVoltage() -
                                             sourceNodePF->singleVoltage()))
          .real();
  if (!(controlVoltageSource->mVoltageRef->get().isApprox(
          expectedControlledVoltage, 1e-10)))
    throw std::runtime_error(
        "ControlledVoltageSource did not initialize from power-flow nodes.");

  auto controlCurrentSource = EMT::Ph3::ControlledCurrentSource::make("Load");
  auto controlCurrentBus = SimNode<Real>::make("load_bus", PhaseType::ABC);
  controlCurrentSource->connect({SimNode<Real>::GND, controlCurrentBus});
  SystemTopology controlCurrentSystem(frequency, {controlCurrentBus},
                                      {controlCurrentSource});
  controlCurrentSystem.initWithPowerflow(systemPF, Domain::EMT);
  controlCurrentSource->initializeFromNodesAndTerminals(frequency);
  const Complex expectedControlledCurrentRms = std::conj(
      Complex(pLoad, qLoad) / (std::sqrt(3.) * loadNodePF->singleVoltage()));
  const Matrix expectedControlledCurrent =
      RMS_TO_PEAK *
      Math::singlePhaseVariableToThreePhase(expectedControlledCurrentRms)
          .real();
  if (!(controlCurrentSource->mCurrentRef->get().isApprox(
          expectedControlledCurrent, 1e-10)))
    throw std::runtime_error(
        "ControlledCurrentSource did not initialize from power-flow data.");

  auto idealGenerator = EMT::Ph3::SynchronGeneratorIdeal::make("Slack");
  auto idealGeneratorBus = SimNode<Real>::make("source_bus", PhaseType::ABC);
  idealGenerator->connect({idealGeneratorBus});
  SystemTopology idealGeneratorSystem(frequency, {idealGeneratorBus},
                                      {idealGenerator});
  idealGeneratorSystem.initWithPowerflow(systemPF, Domain::EMT);
  idealGenerator->initializeFromNodesAndTerminals(frequency);
  auto idealVoltageSource = std::dynamic_pointer_cast<EMT::Ph3::VoltageSource>(
      idealGenerator->subComponents().at(0));
  const MatrixComp expectedIdealVoltage =
      Math::singlePhaseVariableToThreePhase(sourceNodePF->singleVoltage());
  if (!idealVoltageSource || !(idealVoltageSource->mVoltageRef->get().isApprox(
                                 expectedIdealVoltage, 1e-10)))
    throw std::runtime_error(
        "Ideal synchronous generator did not initialize from power flow.");

  auto sourceNodeDP = SimNode<Complex>::make("source_bus", PhaseType::ABC);
  auto loadNodeDP = SimNode<Complex>::make("load_bus", PhaseType::ABC);
  auto sourceDP = DP::Ph3::NetworkInjection::make("Slack");
  sourceDP->setParameters(Math::singlePhaseVariableToThreePhase(
                              RMS3PH_TO_PEAK1PH * Complex(voltageLL, 0.)),
                          0.);
  auto lineDP = DP::Ph3::Resistor::make("Line");
  lineDP->setParameters(linkResistance * Matrix::Identity(3, 3));
  auto loadDP = DP::Ph3::CurrentSource::make("Load");
  sourceDP->connect({sourceNodeDP});
  lineDP->connect({sourceNodeDP, loadNodeDP});
  loadDP->connect({SimNode<Complex>::GND, loadNodeDP});
  SystemTopology systemDP(frequency, {sourceNodeDP, loadNodeDP},
                          {sourceDP, lineDP, loadDP});
  systemDP.initWithPowerflow(systemPF, Domain::DP);

  std::filesystem::create_directories("logs/DP_Ph3_CurrentSource_PF_Init");
  Logger::setLogDir("logs/DP_Ph3_CurrentSource_PF_Init");
  auto loggerDP = DataLogger::make(String("DP_Ph3_CurrentSource_PF_Init"));
  loggerDP->logAttribute("v_bus", loadNodeDP->attribute("v"));
  loggerDP->logAttribute("i_load", loadDP->attribute("i_intf"));
  Simulation simDP("DP_Ph3_CurrentSource_PF_Init", Logger::Level::off);
  simDP.setSystem(systemDP);
  simDP.setDomain(Domain::DP);
  simDP.setTimeStep(timeStep);
  simDP.setFinalTime(finalTime);
  simDP.addLogger(loggerDP);
  simDP.run();

  auto sourceNodeEMT = SimNode<Real>::make("source_bus", PhaseType::ABC);
  auto loadNodeEMT = SimNode<Real>::make("load_bus", PhaseType::ABC);
  auto sourceEMT = EMT::Ph3::NetworkInjection::make("Slack");
  sourceEMT->setParameters(
      Math::singlePhaseVariableToThreePhase(Complex(voltageLL, 0.)), frequency);
  auto lineEMT = EMT::Ph3::Resistor::make("Line");
  lineEMT->setParameters(linkResistance * Matrix::Identity(3, 3));
  auto loadEMT = EMT::Ph3::CurrentSource::make("Load");
  sourceEMT->connect({sourceNodeEMT});
  lineEMT->connect({sourceNodeEMT, loadNodeEMT});
  loadEMT->connect({SimNode<Real>::GND, loadNodeEMT});
  SystemTopology systemEMT(frequency, {sourceNodeEMT, loadNodeEMT},
                           {sourceEMT, lineEMT, loadEMT});
  systemEMT.initWithPowerflow(systemPF, Domain::EMT);

  std::filesystem::create_directories("logs/EMT_Ph3_CurrentSource_PF_Init");
  Logger::setLogDir("logs/EMT_Ph3_CurrentSource_PF_Init");
  auto loggerEMT = DataLogger::make(String("EMT_Ph3_CurrentSource_PF_Init"));
  loggerEMT->logAttribute("v_bus", loadNodeEMT->attribute("v"));
  loggerEMT->logAttribute("i_load", loadEMT->attribute("i_intf"));
  Simulation simEMT("EMT_Ph3_CurrentSource_PF_Init", Logger::Level::off);
  simEMT.setSystem(systemEMT);
  simEMT.setDomain(Domain::EMT);
  simEMT.setTimeStep(timeStep);
  simEMT.setFinalTime(finalTime);
  simEMT.addLogger(loggerEMT);
  simEMT.run();

  const Csv dp = readCsv("DP_Ph3_CurrentSource_PF_Init");
  const Csv emt = readCsv("EMT_Ph3_CurrentSource_PF_Init");
  const Complex dpCurrent(dp.at(dp.rows.size() - 1, "i_load_0.re"),
                          dp.at(dp.rows.size() - 1, "i_load_0.im"));
  const Complex emtCurrent = fundamental(emt, "i_load_0", 0.02, finalTime);
  const Complex dpVoltage(dp.at(dp.rows.size() - 1, "v_bus_0.re"),
                          dp.at(dp.rows.size() - 1, "v_bus_0.im"));
  const Complex emtVoltage = fundamental(emt, "v_bus_0", 0.02, finalTime);
  const Real currentRelativeError =
      std::abs(dpCurrent - emtCurrent) / std::abs(dpCurrent);
  const Real voltageRelativeError =
      std::abs(dpVoltage - emtVoltage) / std::abs(dpVoltage);
  const Complex expectedInitialVoltage =
      RMS3PH_TO_PEAK1PH * loadNodePF->singleVoltage();
  const Complex measuredInitialVoltage(dp.at(0, "v_bus_0.re"),
                                       dp.at(0, "v_bus_0.im"));
  const Real pfVoltageRelativeError =
      std::abs(measuredInitialVoltage - expectedInitialVoltage) /
      std::abs(expectedInitialVoltage);
  const Complex expectedRmsCurrent = std::conj(
      Complex(pLoad, qLoad) / (std::sqrt(3.) * loadNodePF->singleVoltage()));
  const Real referenceRelativeError =
      std::abs(dpCurrent - RMS_TO_PEAK * expectedRmsCurrent) /
      std::abs(dpCurrent);

  Real firstCyclePeakError = 0.;
  Real firstCycleVoltagePeakError = 0.;
  for (size_t i = 0; i < emt.rows.size(); ++i) {
    const Real time = emt.at(i, "time");
    if (time < 0.02)
      firstCyclePeakError =
          std::max(firstCyclePeakError,
                   std::abs(emt.at(i, "i_load_0") -
                            std::real(emtCurrent *
                                      std::polar<Real>(1., 2. * PI * frequency *
                                                               time))));
    if (time < 0.02)
      firstCycleVoltagePeakError =
          std::max(firstCycleVoltagePeakError,
                   std::abs(emt.at(i, "v_bus_0") -
                            std::real(emtVoltage *
                                      std::polar<Real>(1., 2. * PI * frequency *
                                                               time))));
  }

  std::cout << "DP/EMT current relative error: " << currentRelativeError
            << "\nDP/EMT voltage relative error: " << voltageRelativeError
            << "\nInitial DP/PF voltage relative error: "
            << pfVoltageRelativeError
            << "\nPF-reference relative error: " << referenceRelativeError
            << "\nFirst-cycle current peak deviation: " << firstCyclePeakError
            << " A\nFirst-cycle voltage peak deviation: "
            << firstCycleVoltagePeakError << " V\n";
  if (currentRelativeError > 0.01 || voltageRelativeError > 0.01 ||
      pfVoltageRelativeError > 1e-6 || referenceRelativeError > 0.01 ||
      firstCyclePeakError > 0.02 * std::abs(emtCurrent) ||
      firstCycleVoltagePeakError > 0.02 * std::abs(emtVoltage))
    throw std::runtime_error("DP/EMT Ph3 current-source PF validation failed");
}
