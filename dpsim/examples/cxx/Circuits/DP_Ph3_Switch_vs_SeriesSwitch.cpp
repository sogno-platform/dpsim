// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <DPsim.h>
#include <fstream>
#include <sstream>
#include <stdexcept>

using namespace DPsim;
using namespace CPS::DP;

// Parity check: a balanced diagonal R must reproduce DP::Ph3::SeriesSwitch, and
// an asymmetric closed matrix (phase A only) must give per-phase-diverging
// current, the capability SeriesSwitch lacks.

static const Real switchOpenR = 1e6;
static const Real switchClosedR = 0.001;
static const Real startTimeFault = 0.03;
static const Real endTimeFault = 0.06;
static const Real timeStep = 0.0001;
static const Real finalTime = 0.1;

static Matrix identity3() {
  Matrix m = Matrix::Zero(3, 3);
  m << 1., 0, 0, 0, 1., 0, 0, 0, 1.;
  return m;
}

// Builds the shared R-network and returns the two nodes.
static void buildNetwork(SimNode::Ptr &n1, SimNode::Ptr &n2,
                         SystemComponentList &comps) {
  const Matrix param = identity3();

  auto cs0 = Ph3::CurrentSource::make("CS0");
  cs0->setParameters(
      CPS::Math::singlePhaseVariableToThreePhase(CPS::Math::polar(1.0, 0.0)));
  auto r1 = Ph3::Resistor::make("R1");
  r1->setParameters(10 * param);
  auto r2 = Ph3::Resistor::make("R2");
  r2->setParameters(param);

  cs0->connect(SimNode::List{n1, SimNode::GND});
  r1->connect(SimNode::List{n2, n1});
  r2->connect(SimNode::List{n2, SimNode::GND});

  comps = SystemComponentList{cs0, r1, r2};
}

static void runSeriesSwitch(const String &simName) {
  auto n1 = SimNode::make("n1", PhaseType::ABC);
  auto n2 = SimNode::make("n2", PhaseType::ABC);
  SystemComponentList comps;
  buildNetwork(n1, n2, comps);

  auto fault = Ph3::SeriesSwitch::make("fault");
  fault->setParameters(switchOpenR, switchClosedR);
  fault->open();
  fault->connect(SimNode::List{n2, SimNode::GND});
  comps.push_back(fault);

  auto sys = SystemTopology(50, SystemNodeList{n1, n2}, comps);

  Logger::setLogDir("logs/" + simName);
  auto logger = DataLogger::make(simName);
  logger->logAttribute("v_n2", n2->attribute("v"));
  logger->logAttribute("i_fault", fault->attribute("i_intf"));

  Simulation sim(simName, Logger::Level::info);
  sim.setSystem(sys);
  sim.addLogger(logger);
  sim.setDomain(Domain::DP);
  sim.setTimeStep(timeStep);
  sim.setFinalTime(finalTime);
  sim.doSystemMatrixRecomputation(true);
  sim.addEvent(SwitchEvent::make(startTimeFault, fault, true));
  sim.addEvent(SwitchEvent::make(endTimeFault, fault, false));
  sim.run();
}

// closedR is the 3x3 closed-state resistance matrix (diagonal per phase).
static void runNewSwitch(const String &simName, const Matrix &closedR) {
  auto n1 = SimNode::make("n1", PhaseType::ABC);
  auto n2 = SimNode::make("n2", PhaseType::ABC);
  SystemComponentList comps;
  buildNetwork(n1, n2, comps);

  auto fault = Ph3::Switch::make("fault");
  fault->setParameters(switchOpenR * identity3(), closedR);
  fault->openSwitch();
  fault->connect(SimNode::List{n2, SimNode::GND});
  comps.push_back(fault);

  auto sys = SystemTopology(50, SystemNodeList{n1, n2}, comps);

  Logger::setLogDir("logs/" + simName);
  auto logger = DataLogger::make(simName);
  logger->logAttribute("v_n2", n2->attribute("v"));
  logger->logAttribute("i_fault", fault->attribute("i_intf"));

  Simulation sim(simName, Logger::Level::info);
  sim.setSystem(sys);
  sim.addLogger(logger);
  sim.setDomain(Domain::DP);
  sim.setTimeStep(timeStep);
  sim.setFinalTime(finalTime);
  sim.doSystemMatrixRecomputation(true);
  sim.addEvent(SwitchEvent3Ph::make(startTimeFault, fault, true));
  sim.addEvent(SwitchEvent3Ph::make(endTimeFault, fault, false));
  sim.run();
}

static String trim(const String &s) {
  const size_t a = s.find_first_not_of(" \t\r\n");
  if (a == String::npos)
    return "";
  const size_t b = s.find_last_not_of(" \t\r\n");
  return s.substr(a, b - a + 1);
}

struct CsvData {
  std::vector<String> names;           // column headers, trimmed
  std::vector<std::vector<Real>> rows; // numeric rows
  std::map<String, size_t> colByName;  // header -> column index

  Real at(size_t row, const String &name) const {
    return rows.at(row).at(colByName.at(name));
  }
  bool has(const String &name) const { return colByName.count(name) > 0; }
};

static CsvData readCsv(const String &simName) {
  const String path = "logs/" + simName + "/" + simName + ".csv";
  std::ifstream file(path);
  if (!file.is_open())
    throw std::runtime_error("Could not open CSV: " + path);

  CsvData data;
  String line;
  std::getline(file, line); // header
  {
    std::stringstream ss(line);
    String cell;
    while (std::getline(ss, cell, ','))
      data.names.push_back(trim(cell));
  }
  for (size_t i = 0; i < data.names.size(); ++i)
    data.colByName[data.names[i]] = i;

  while (std::getline(file, line)) {
    if (trim(line).empty())
      continue;
    std::vector<Real> row;
    std::stringstream ss(line);
    String cell;
    while (std::getline(ss, cell, ','))
      row.push_back(std::stod(cell));
    data.rows.push_back(row);
  }
  return data;
}

// Magnitude of a complex column pair name.re / name.im at a given row.
static Real magAt(const CsvData &d, size_t row, const String &base) {
  const Real re = d.at(row, base + ".re");
  const Real im = d.at(row, base + ".im");
  return std::sqrt(re * re + im * im);
}

int main(int argc, char *argv[]) {
  const String nameSeries = "DP_Ph3_Switch_vs_SeriesSwitch_Series";
  const String nameBalanced = "DP_Ph3_Switch_vs_SeriesSwitch_Balanced";
  const String nameAsym = "DP_Ph3_Switch_vs_SeriesSwitch_Asymmetric";

  runSeriesSwitch(nameSeries);
  runNewSwitch(nameBalanced, switchClosedR * identity3());

  // Asymmetric: only phase A shorts; phases B and C stay effectively open.
  Matrix closedAsym = Matrix::Zero(3, 3);
  closedAsym(0, 0) = switchClosedR;
  closedAsym(1, 1) = switchOpenR;
  closedAsym(2, 2) = switchOpenR;
  runNewSwitch(nameAsym, closedAsym);

  // Balanced parity: a balanced diagonal R must reproduce SeriesSwitch's v_n2.
  const auto series = readCsv(nameSeries);
  const auto balanced = readCsv(nameBalanced);
  if (series.rows.size() != balanced.rows.size())
    throw std::runtime_error("Row count mismatch between the two runs.");

  const std::vector<String> voltageCols = {"v_n2_0.re", "v_n2_0.im",
                                           "v_n2_1.re", "v_n2_1.im",
                                           "v_n2_2.re", "v_n2_2.im"};
  Real maxDiff = 0.0;
  for (size_t i = 0; i < series.rows.size(); ++i)
    for (const auto &c : voltageCols)
      maxDiff =
          std::max(maxDiff, std::abs(series.at(i, c) - balanced.at(i, c)));

  std::cout << "Balanced parity max abs v_n2 diff: " << maxDiff << std::endl;
  if (maxDiff > 1e-9)
    throw std::runtime_error(
        "DP::Ph3::Switch does not match DP::Ph3::SeriesSwitch in the balanced "
        "case.");

  // Asymmetric fault: phase-A current must dominate B and C.
  const auto asym = readCsv(nameAsym);
  Real faultPhaseA = 0.0, faultPhaseB = 0.0;
  for (size_t i = 0; i < asym.rows.size(); ++i) {
    const Real t = asym.at(i, "time");
    if (t <= startTimeFault || t >= endTimeFault)
      continue;
    faultPhaseA = std::max(faultPhaseA, magAt(asym, i, "i_fault_0"));
    faultPhaseB = std::max(faultPhaseB, magAt(asym, i, "i_fault_1"));
  }
  std::cout << "Asymmetric fault |I_A|=" << faultPhaseA
            << "  |I_B|=" << faultPhaseB << std::endl;
  if (!(faultPhaseA > 100.0 * faultPhaseB))
    throw std::runtime_error(
        "Asymmetric switch did not produce a single-phase-dominant current.");

  std::cout << "DP_Ph3_Switch parity checks PASSED." << std::endl;
  return 0;
}
