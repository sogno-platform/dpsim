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
constexpr Real timeStep = 1e-5;
constexpr Real finalTime = 0.04;
constexpr Real pqAnchorPower = 1.;
const Real pqAnchorResistance = voltageLL * voltageLL / pqAnchorPower;

enum class Kind {
  Resistor,
  SeriesResistor,
  Capacitor,
  Inductor,
  SeriesSwitch,
  Switch,
  GenericVType,
  GenericIType,
  FullSerialRLC,
  MutualInductor,
  MutualResistor,
  Shunt,
  ShuntInductor
};
Matrix eye3();
Matrix inductanceMatrix(Kind kind);
Matrix resistanceMatrix(Kind kind);

struct Case {
  Kind kind;
  String label;
  Real conductance;
  Real susceptance;
};

struct Csv {
  std::vector<std::vector<Real>> rows;
  std::map<String, size_t> columns;
  Real at(size_t row, const String &name) const {
    return rows.at(row).at(columns.at(name));
  }
};

Real componentResistance(Kind kind) {
  return (kind == Kind::Switch || kind == Kind::SeriesSwitch) ? 0.5 : 10.;
}

SimPowerComp<Complex>::Ptr makeDPBranch(Kind kind) {
  const Real r = componentResistance(kind);
  if (kind == Kind::Shunt) {
    auto c = DP::Ph3::Resistor::make("Branch");
    c->setParameters(resistanceMatrix(kind));
    return c;
  }
  if (kind == Kind::ShuntInductor) {
    auto c = DP::Ph3::Inductor::make("Branch");
    c->setParameters(0.05 * eye3());
    return c;
  }
  if (kind == Kind::Resistor) {
    auto c = DP::Ph3::Resistor::make("Branch");
    c->setParameters(resistanceMatrix(kind));
    return c;
  }
  if (kind == Kind::MutualResistor) {
    auto c = DP::Ph3::Resistor::make("Branch");
    c->setParameters(resistanceMatrix(kind));
    return c;
  }
  if (kind == Kind::SeriesResistor) {
    auto c = DP::Ph3::SeriesResistor::make("Branch");
    c->setParameters(r);
    return c;
  }
  if (kind == Kind::Capacitor) {
    auto c = DP::Ph3::Capacitor::make("Branch");
    c->setParameters(15e-6 * eye3());
    return c;
  }
  if (kind == Kind::Inductor) {
    auto c = DP::Ph3::Inductor::make("Branch");
    c->setParameters(inductanceMatrix(kind));
    return c;
  }
  if (kind == Kind::MutualInductor) {
    auto c = DP::Ph3::Inductor::make("Branch");
    c->setParameters(inductanceMatrix(kind));
    return c;
  }
  if (kind == Kind::SeriesSwitch) {
    auto c = DP::Ph3::SeriesSwitch::make("Branch");
    c->setParameters(1e9, r);
    c->close();
    return c;
  }
  if (kind == Kind::GenericVType) {
    auto c = DP::Ph3::GenericTwoTerminalVTypeSSN::make("Branch");
    const Matrix zero = Matrix::Zero(3, 3);
    c->setParameters(-eye3(), zero, zero, eye3() / r);
    return c;
  }
  if (kind == Kind::GenericIType) {
    auto c = DP::Ph3::GenericTwoTerminalITypeSSN::make("Branch");
    const Matrix zero = Matrix::Zero(3, 3);
    c->setParameters(-eye3(), zero, zero, r * eye3());
    return c;
  }
  if (kind == Kind::FullSerialRLC) {
    auto c = DP::Ph3::SSN::Full_Serial_RLC::make("Branch");
    c->setParameters(10. * eye3(), 0.05 * eye3(), 15e-6 * eye3());
    return c;
  }
  auto c = DP::Ph3::Switch::make("Branch");
  c->setParameters(1e9 * eye3(), r * eye3(), true);
  return c;
}

SimPowerComp<Real>::Ptr makeEMTBranch(Kind kind) {
  const Real r = componentResistance(kind);
  if (kind == Kind::Shunt || kind == Kind::ShuntInductor)
    return EMT::Ph3::Shunt::make("Branch");
  if (kind == Kind::Resistor) {
    auto c = EMT::Ph3::Resistor::make("Branch");
    c->setParameters(resistanceMatrix(kind));
    return c;
  }
  if (kind == Kind::MutualResistor) {
    auto c = EMT::Ph3::Resistor::make("Branch");
    c->setParameters(resistanceMatrix(kind));
    return c;
  }
  if (kind == Kind::SeriesResistor) {
    auto c = EMT::Ph3::SeriesResistor::make("Branch");
    c->setParameters(r);
    return c;
  }
  if (kind == Kind::Capacitor) {
    auto c = EMT::Ph3::Capacitor::make("Branch");
    c->setParameters(15e-6 * eye3());
    return c;
  }
  if (kind == Kind::Inductor) {
    auto c = EMT::Ph3::Inductor::make("Branch");
    c->setParameters(inductanceMatrix(kind));
    return c;
  }
  if (kind == Kind::MutualInductor) {
    auto c = EMT::Ph3::Inductor::make("Branch");
    c->setParameters(inductanceMatrix(kind));
    return c;
  }
  if (kind == Kind::SeriesSwitch) {
    auto c = EMT::Ph3::SeriesSwitch::make("Branch");
    c->setParameters(1e9, r);
    c->close();
    return c;
  }
  if (kind == Kind::GenericVType) {
    auto c = EMT::Ph3::GenericTwoTerminalVTypeSSN::make("Branch");
    const Matrix zero = Matrix::Zero(3, 3);
    c->setParameters(-eye3(), zero, zero, eye3() / r);
    return c;
  }
  if (kind == Kind::GenericIType) {
    auto c = EMT::Ph3::GenericTwoTerminalITypeSSN::make("Branch");
    const Matrix zero = Matrix::Zero(3, 3);
    c->setParameters(-eye3(), zero, zero, r * eye3());
    return c;
  }
  if (kind == Kind::FullSerialRLC) {
    auto c = EMT::Ph3::SSN::Full_Serial_RLC::make("Branch");
    c->setParameters(10. * eye3(), 0.05 * eye3(), 15e-6 * eye3());
    return c;
  }
  auto c = EMT::Ph3::Switch::make("Branch");
  c->setParameters(1e9 * eye3(), r * eye3(), true);
  return c;
}

Csv readCsv(const String &name) {
  const String path = "logs/" + name + "/" + name + ".csv";
  std::ifstream file(path);
  if (!file)
    throw std::runtime_error("Could not open " + path);
  Csv data;
  String line;
  std::getline(file, line);
  std::stringstream hs(line);
  String cell;
  size_t col = 0;
  while (std::getline(hs, cell, ',')) {
    while (!cell.empty() && cell.front() == ' ')
      cell.erase(cell.begin());
    while (!cell.empty() && cell.back() == ' ')
      cell.pop_back();
    data.columns[cell] = col++;
  }
  while (std::getline(file, line)) {
    if (line.empty())
      continue;
    std::stringstream rs(line);
    std::vector<Real> row;
    while (std::getline(rs, cell, ','))
      row.push_back(std::stod(cell));
    data.rows.push_back(std::move(row));
  }
  return data;
}

Complex fundamental(const Csv &data, const String &column, Real t0, Real t1) {
  Complex value = 0.;
  size_t count = 0;
  for (const auto &row : data.rows) {
    const Real time = row[data.columns.at("time")];
    if (time < t0 || time >= t1)
      continue;
    const Real x = row[data.columns.at(column)];
    const Real phase = 2. * PI * frequency * time;
    value += Complex(2. * x * std::cos(phase), -2. * x * std::sin(phase));
    ++count;
  }
  if (!count)
    throw std::runtime_error("Empty Fourier window for " + column);
  return value / static_cast<Real>(count);
}

Matrix eye3() { return Matrix::Identity(3, 3); }

Matrix inductanceMatrix(Kind kind) {
  Matrix inductance = 0.05 * eye3();
  if (kind == Kind::MutualInductor) {
    inductance(0, 1) = inductance(1, 0) = 0.01;
    inductance(0, 2) = inductance(2, 0) = 0.01;
    inductance(1, 2) = inductance(2, 1) = 0.01;
  }
  return inductance;
}

Matrix resistanceMatrix(Kind kind) {
  Matrix resistance = 10. * eye3();
  if (kind == Kind::MutualResistor) {
    resistance(0, 1) = resistance(1, 0) = 2.;
    resistance(0, 2) = resistance(2, 0) = 2.;
    resistance(1, 2) = resistance(2, 1) = 2.;
  }
  return resistance;
}

void runCase(const Case &test) {
  const String suffix = test.label;
  const String pfName = "Ph3_Passive_PF_" + suffix;
  auto pfSourceNode = SimNode<Complex>::make("source_bus", PhaseType::Single);
  auto pfLoadNode = SimNode<Complex>::make("load_bus", PhaseType::Single);
  auto pfSlack = SP::Ph1::NetworkInjection::make("Slack");
  pfSlack->setParameters(voltageLL);
  pfSlack->setBaseVoltage(voltageLL);
  pfSlack->modifyPowerFlowBusType(PowerflowBusType::VD);
  auto pfLink = SP::Ph1::PiLine::make("Link");
  pfLink->setParameters(0.01, 0.1e-3, 1e-6, 1e-6);
  pfLink->setBaseVoltage(voltageLL);
  auto pfBranch = SP::Ph1::Shunt::make("Branch");
  pfBranch->setParameters(test.conductance, test.susceptance);
  pfBranch->setBaseVoltage(voltageLL);
  auto pfAnchor = SP::Ph1::Load::make("PQAnchor");
  pfAnchor->setParameters(pqAnchorPower, 0., voltageLL);
  pfAnchor->modifyPowerFlowBusType(PowerflowBusType::PQ);
  pfSlack->connect({pfSourceNode});
  pfLink->connect({pfSourceNode, pfLoadNode});
  pfBranch->connect({pfSourceNode});
  pfAnchor->connect({pfLoadNode});
  SystemTopology systemPF(frequency, {pfSourceNode, pfLoadNode},
                          {pfSlack, pfLink, pfBranch, pfAnchor});
  Simulation pf(pfName, Logger::Level::off);
  pf.setSystem(systemPF);
  pf.setTimeStep(0.01);
  pf.setFinalTime(0.02);
  pf.setDomain(Domain::SP);
  pf.setSolverType(Solver::Type::NRP);
  pf.setSolverAndComponentBehaviour(Solver::Behaviour::Initialization);
  pf.doInitFromNodesAndTerminals(true);
  pf.run();

  auto dpSourceNode = SimNode<Complex>::make("source_bus", PhaseType::ABC);
  auto dpLoadNode = SimNode<Complex>::make("load_bus", PhaseType::ABC);
  auto dpSource = DP::Ph3::VoltageSource::make("Slack");
  dpSource->setParameters(Math::singlePhaseVariableToThreePhase(
                              RMS3PH_TO_PEAK1PH * Complex(voltageLL, 0.)),
                          0.);
  auto dpLink = DP::Ph3::PiLine::make("Link");
  dpLink->setParameters(0.01 * eye3(), 0.1e-3 * eye3(), 1e-6 * eye3(),
                        1e-6 * eye3());
  auto dpAnchor = DP::Ph3::Resistor::make("PQAnchor");
  dpAnchor->setParameters(pqAnchorResistance * eye3());
  auto dpBranch = makeDPBranch(test.kind);
  dpSource->connect({SimNode<Complex>::GND, dpSourceNode});
  dpLink->connect({dpSourceNode, dpLoadNode});
  dpAnchor->connect({dpLoadNode, SimNode<Complex>::GND});
  dpBranch->connect({dpSourceNode, SimNode<Complex>::GND});
  SystemTopology systemDP(frequency, {dpSourceNode, dpLoadNode},
                          {dpSource, dpLink, dpAnchor, dpBranch});
  systemDP.initWithPowerflow(systemPF, Domain::DP);

  const String dpName = "DP_Ph3_Passive_PF_" + suffix;
  std::filesystem::create_directories("logs/" + dpName);
  Logger::setLogDir("logs/" + dpName);
  auto logDP = DataLogger::make(dpName);
  logDP->logAttribute("v_bus", dpSourceNode->attribute("v"));
  logDP->logAttribute("v_load", dpLoadNode->attribute("v"));
  logDP->logAttribute("i_link", dpLink->attribute("i_intf"));
  logDP->logAttribute("i_branch", dpBranch->attribute("i_intf"));
  Simulation simDP(dpName, Logger::Level::off);
  simDP.setSystem(systemDP);
  simDP.setDomain(Domain::DP);
  simDP.setTimeStep(timeStep);
  simDP.setFinalTime(finalTime);
  simDP.addLogger(logDP);
  simDP.run();

  auto emtSourceNode = SimNode<Real>::make("source_bus", PhaseType::ABC);
  auto emtLoadNode = SimNode<Real>::make("load_bus", PhaseType::ABC);
  auto emtSource = EMT::Ph3::VoltageSource::make("Slack");
  emtSource->setParameters(
      Math::singlePhaseVariableToThreePhase(Complex(voltageLL, 0.)), frequency);
  auto emtLink = EMT::Ph3::PiLine::make("Link");
  emtLink->setParameters(0.01 * eye3(), 0.1e-3 * eye3(), 1e-6 * eye3(),
                         1e-6 * eye3());
  // Intentionally leave the PQLoad unconfigured: initWithPowerflow() must
  // provide P/Q and terminal voltage from the matched SP power-flow load.
  auto emtAnchor = EMT::Ph3::PQLoad::make("PQAnchor");
  auto emtBranch = makeEMTBranch(test.kind);
  emtSource->connect({SimNode<Real>::GND, emtSourceNode});
  emtLink->connect({emtSourceNode, emtLoadNode});
  emtAnchor->connect({emtLoadNode});
  if (test.kind == Kind::Shunt || test.kind == Kind::ShuntInductor)
    emtBranch->connect({emtSourceNode});
  else
    emtBranch->connect({emtSourceNode, SimNode<Real>::GND});
  SystemTopology systemEMT(frequency, {emtSourceNode, emtLoadNode},
                           {emtSource, emtLink, emtAnchor, emtBranch});
  systemEMT.initWithPowerflow(systemPF, Domain::EMT);

  const String emtName = "EMT_Ph3_Passive_PF_" + suffix;
  std::filesystem::create_directories("logs/" + emtName);
  Logger::setLogDir("logs/" + emtName);
  auto logEMT = DataLogger::make(emtName);
  logEMT->logAttribute("v_bus", emtSourceNode->attribute("v"));
  logEMT->logAttribute("v_load", emtLoadNode->attribute("v"));
  logEMT->logAttribute("i_link", emtLink->attribute("i_intf"));
  logEMT->logAttribute("i_branch", emtBranch->attribute("i_intf"));
  Simulation simEMT(emtName, Logger::Level::off);
  simEMT.setSystem(systemEMT);
  simEMT.setDomain(Domain::EMT);
  simEMT.setTimeStep(timeStep);
  simEMT.setFinalTime(finalTime);
  simEMT.addLogger(logEMT);
  simEMT.run();

  const Csv dp = readCsv(dpName);
  const Csv emt = readCsv(emtName);
  const String dpCurrentRealColumn =
      dp.columns.count("i_branch_0.re") ? "i_branch_0.re" : "i_branch.re";
  const String dpCurrentImagColumn =
      dp.columns.count("i_branch_0.im") ? "i_branch_0.im" : "i_branch.im";
  const String emtCurrentColumn =
      emt.columns.count("i_branch_0") ? "i_branch_0" : "i_branch";
  const Complex dpCurrent(dp.at(dp.rows.size() - 1, dpCurrentRealColumn),
                          dp.at(dp.rows.size() - 1, dpCurrentImagColumn));
  const Complex emtCurrent =
      fundamental(emt, emtCurrentColumn, 0.02, finalTime);
  const Real currentError =
      std::abs(dpCurrent - emtCurrent) / std::max(1e-12, std::abs(dpCurrent));
  const Complex dpLinkCurrent(dp.at(dp.rows.size() - 1, "i_link_0.re"),
                              dp.at(dp.rows.size() - 1, "i_link_0.im"));
  const Complex emtLinkCurrent = fundamental(emt, "i_link_0", 0.02, finalTime);
  const Real linkCurrentError = std::abs(dpLinkCurrent - emtLinkCurrent) /
                                std::max(1e-12, std::abs(dpLinkCurrent));
  const Complex dpVoltage(dp.at(0, "v_bus_0.re"), dp.at(0, "v_bus_0.im"));
  const Complex expectedVoltage =
      RMS3PH_TO_PEAK1PH * pfSourceNode->singleVoltage();
  const Real initVoltageError =
      std::abs(dpVoltage - expectedVoltage) / std::abs(expectedVoltage);
  const Complex dpLoadVoltage(dp.at(dp.rows.size() - 1, "v_load_0.re"),
                              dp.at(dp.rows.size() - 1, "v_load_0.im"));
  const Complex emtLoadVoltage = fundamental(emt, "v_load_0", 0.02, finalTime);
  const Real loadVoltageError = std::abs(dpLoadVoltage - emtLoadVoltage) /
                                std::max(1e-12, std::abs(dpLoadVoltage));
  Real firstCyclePeakError = 0.;
  for (size_t i = 0; i < emt.rows.size(); ++i) {
    const Real time = emt.at(i, "time");
    if (time >= 0.02)
      continue;
    const Real expected = std::real(
        emtCurrent * std::polar<Real>(1., 2. * PI * frequency * time));
    firstCyclePeakError = std::max(
        firstCyclePeakError, std::abs(emt.at(i, emtCurrentColumn) - expected));
  }
  Real firstCycleLinkPeakError = 0.;
  for (size_t i = 0; i < emt.rows.size(); ++i) {
    const Real time = emt.at(i, "time");
    if (time >= 0.02)
      continue;
    const Real expected = std::real(
        emtLinkCurrent * std::polar<Real>(1., 2. * PI * frequency * time));
    firstCycleLinkPeakError = std::max(
        firstCycleLinkPeakError, std::abs(emt.at(i, "i_link_0") - expected));
  }
  const Real errorLimit = std::max(2e-5, 1e-3 * std::abs(dpCurrent));
  std::cout << suffix << ": DP/EMT current error=" << currentError
            << ", DP/EMT PiLine current error=" << linkCurrentError
            << ", DP/EMT load voltage error=" << loadVoltageError
            << ", PF voltage error=" << initVoltageError
            << ", first-cycle peak deviation=" << firstCyclePeakError << " A"
            << ", PiLine first-cycle peak deviation=" << firstCycleLinkPeakError
            << " A" << std::endl;
  if (currentError > 1e-3 || linkCurrentError > 1e-2 ||
      loadVoltageError > 1e-2 || initVoltageError > 1e-6 ||
      firstCyclePeakError > errorLimit ||
      firstCycleLinkPeakError > std::max(2e-5, 0.02 * std::abs(emtLinkCurrent)))
    throw std::runtime_error("Ph3 passive PF parity failed for " + suffix);
}
} // namespace

int main() {
  const Real omega = 2. * PI * frequency;
  const Real capacitor = 15e-6;
  const Real inductor = 0.05;
  runCase({Kind::Resistor, "Resistor", 1. / 10., 0.});
  runCase({Kind::MutualResistor, "MutualResistor", 1. / 8., 0.});
  runCase({Kind::SeriesResistor, "SeriesResistor", 1. / 10., 0.});
  runCase({Kind::Capacitor, "Capacitor", 0., omega * capacitor});
  runCase({Kind::Inductor, "Inductor", 0., -1. / (omega * inductor)});
  runCase({Kind::MutualInductor, "MutualInductor", 0.,
           -1. / (omega * (inductor - 0.01))});
  runCase({Kind::SeriesSwitch, "SeriesSwitch", 1. / 0.5, 0.});
  runCase({Kind::Shunt, "Shunt", 1. / 10., 0.});
  runCase({Kind::ShuntInductor, "ShuntInductor", 0., -1. / (omega * inductor)});
  runCase({Kind::Switch, "Switch", 1. / 0.5, 0.});
  runCase({Kind::GenericVType, "GenericVTypeSSN", 1. / 10., 0.});
  runCase({Kind::GenericIType, "GenericITypeSSN", 1. / 10., 0.});
  const Complex impedance(10., 2. * PI * frequency * 0.05 -
                                   1. / (2. * PI * frequency * 15e-6));
  const Complex admittance = 1. / impedance;
  runCase({Kind::FullSerialRLC, "FullSerialRLCSSN", admittance.real(),
           -admittance.imag()});
}
