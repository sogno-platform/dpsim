// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <DPsim.h>

using namespace DPsim;
using namespace CPS;

namespace {

const Real frequency = 50;
const Real nomVoltagePrimary = 110e3;
const Real nomVoltageSecondary = 10e3;
const Real ratedPower = 40e6;
const Real shortCircuitVoltage = 0.12;
const Real shortCircuitResistance = 0.005;
const Real noLoadCurrent = 0.010;
const Real noLoadLoss = 1e-3;
const Real openResistance = 1e9;
const Real settlingSteps = 2000;
const Real snubberActivePowerFraction = 1e-3;
const Real snubberReactivePowerFraction = 5e-4;

enum class Termination { Resistive, Floating };

struct Result {
  Real noLoadCurrentPercent;
  Real secondaryVoltageErrorPercent;
  Real reactiveSign;
};

Real leakageResistance() {
  return shortCircuitResistance * std::pow(nomVoltagePrimary, 2) / ratedPower;
}

Real leakageInductance() {
  return std::sqrt(std::pow(shortCircuitVoltage, 2) -
                   std::pow(shortCircuitResistance, 2)) *
         std::pow(nomVoltagePrimary, 2) / ratedPower / (2. * PI * frequency);
}

SystemTopology powerflow(const String &name, Bool magnetizing) {
  Logger::setLogDir("logs/" + name);

  auto n1 = SimNode<Complex>::make("n1", PhaseType::Single);
  auto n2 = SimNode<Complex>::make("n2", PhaseType::Single);

  auto extnet = SP::Ph1::NetworkInjection::make("Slack", Logger::Level::off);
  extnet->setParameters(nomVoltagePrimary);
  extnet->setBaseVoltage(nomVoltagePrimary);
  extnet->modifyPowerFlowBusType(PowerflowBusType::VD);

  auto trafo = SP::Ph1::Transformer::make(name, Logger::Level::off);
  trafo->setParameters(nomVoltagePrimary, nomVoltageSecondary,
                       magnetizing ? ratedPower : 0.,
                       nomVoltagePrimary / nomVoltageSecondary, 0.,
                       leakageResistance(), leakageInductance());
  if (magnetizing)
    trafo->setMagnetizingBranch(noLoadCurrent, noLoadLoss);
  trafo->setBaseVoltage(nomVoltagePrimary);

  auto load = SP::Ph1::Shunt::make("Termination", Logger::Level::off);
  load->setParameters(1. / openResistance, 0.);
  load->setBaseVoltage(nomVoltageSecondary);

  extnet->connect({n1});
  trafo->connect({n1, n2});
  load->connect({n2});

  auto system = SystemTopology(frequency, SystemNodeList{n1, n2},
                               SystemComponentList{extnet, trafo, load});

  Simulation sim(name, Logger::Level::off);
  sim.setSystem(system);
  sim.setTimeStep(1.0);
  sim.setFinalTime(2.0);
  sim.setDomain(Domain::SP);
  sim.setSolverType(Solver::Type::NRP);
  sim.setSolverAndComponentBehaviour(Solver::Behaviour::Initialization);
  sim.doInitFromNodesAndTerminals(true);
  sim.run();

  Logger::setLogDir("logs");
  return system;
}

Result energize(const String &name, Real timeStep, Bool magnetizing,
                Termination termination) {
  auto systemPF = powerflow(name + "_PF", magnetizing);

  auto n1 = SimNode<Complex>::make("n1", PhaseType::Single);
  auto n2 = SimNode<Complex>::make("n2", PhaseType::Single);

  auto vs = DP::Ph1::VoltageSource::make("vs", Logger::Level::off);
  vs->setParameters(Complex(nomVoltagePrimary, 0), 0.);
  vs->connect({SimNode<Complex>::GND, n1});

  auto trafo = DP::Ph1::Transformer::make(name, name, Logger::Level::off, true);
  trafo->setParameters(nomVoltagePrimary, nomVoltageSecondary,
                       magnetizing ? ratedPower : 0.,
                       nomVoltagePrimary / nomVoltageSecondary, 0.,
                       leakageResistance(), leakageInductance());
  if (magnetizing)
    trafo->setMagnetizingBranch(noLoadCurrent, noLoadLoss);
  trafo->connect({n1, n2});

  SystemComponentList components{vs, trafo};

  if (!magnetizing) {
    Real snubberActivePower = snubberActivePowerFraction * ratedPower;
    Real snubberReactivePower = snubberReactivePowerFraction * ratedPower;

    auto snubResistor1 =
        DP::Ph1::Resistor::make("snub_res1", Logger::Level::off);
    snubResistor1->setParameters(std::pow(nomVoltagePrimary, 2) /
                                 snubberActivePower);
    snubResistor1->connect({n1, SimNode<Complex>::GND});
    components.push_back(snubResistor1);

    auto snubResistor2 =
        DP::Ph1::Resistor::make("snub_res2", Logger::Level::off);
    snubResistor2->setParameters(std::pow(nomVoltageSecondary, 2) /
                                 snubberActivePower);
    snubResistor2->connect({n2, SimNode<Complex>::GND});
    components.push_back(snubResistor2);

    auto snubCapacitor2 =
        DP::Ph1::Capacitor::make("snub_cap2", Logger::Level::off);
    snubCapacitor2->setParameters(snubberReactivePower /
                                  std::pow(nomVoltageSecondary, 2) /
                                  (2. * PI * frequency));
    snubCapacitor2->connect({n2, SimNode<Complex>::GND});
    components.push_back(snubCapacitor2);
  }

  auto sw = DP::Ph1::Switch::make("Termination", Logger::Level::off);
  if (termination == Termination::Resistive) {
    sw->setParameters(openResistance, 1e-6, false);
    sw->connect({n2, SimNode<Complex>::GND});
    components.push_back(sw);
  }

  auto sys = SystemTopology(frequency, SystemNodeList{n1, n2}, components);
  sys.initWithPowerflow(systemPF, Domain::DP);

  auto logger = DataLogger::make(name, Logger::Level::off);
  logger->logAttribute("i_primary", vs->attribute("i_intf"));
  logger->logAttribute("v_secondary", n2->attribute("v"));

  Simulation sim(name, Logger::Level::off);
  sim.setSystem(sys);
  sim.setDomain(Domain::DP);
  sim.setTimeStep(timeStep);
  sim.setFinalTime(settlingSteps * timeStep);
  sim.addLogger(logger);
  sim.run();

  Complex current = (**vs->mIntfCurrent)(0, 0);
  Complex secondaryVoltage = (**n2->mVoltage)(0, 0);
  Complex power = nomVoltagePrimary * std::conj(current);

  Result result;
  result.noLoadCurrentPercent = 100. * std::abs(power) / ratedPower;
  result.secondaryVoltageErrorPercent =
      100. * (std::abs(secondaryVoltage) - nomVoltageSecondary) /
      nomVoltageSecondary;
  result.reactiveSign = power.imag() >= 0 ? 1. : -1.;
  return result;
}

} // namespace

int main(int argc, char *argv[]) {
  std::error_code ec;
  std::filesystem::create_directories("./logs", ec);
  Logger::setLogDir("logs");

  auto log = Logger::get("transformer_step_sweep", Logger::Level::info,
                         Logger::Level::info);

  std::vector<Real> timeSteps = {1e-7, 1e-6, 1e-5, 5e-5, 1e-4,
                                 5e-4, 1e-3, 5e-3, 1e-2};

  SPDLOG_LOGGER_INFO(log,
                     "open-secondary sweep, {} MVA {} / {} kV, nameplate i0 {} "
                     "lagging, P0 {}",
                     ratedPower / 1e6, nomVoltagePrimary / 1e3,
                     nomVoltageSecondary / 1e3, noLoadCurrent, noLoadLoss);

  for (Real timeStep : timeSteps) {
    String tag = "dt_" + std::to_string(timeStep);

    Result magnetizing =
        energize(tag + "_mag", timeStep, true, Termination::Resistive);
    Result floating =
        energize(tag + "_float", timeStep, true, Termination::Floating);
    Result snubbers =
        energize(tag + "_snub", timeStep, false, Termination::Resistive);

    SPDLOG_LOGGER_INFO(
        log,
        "dt {:.1e}: magnetizing i0 {:.5f} % {}, v2 error {:.6f} %; floating "
        "i0 {:.5f} % {}, v2 error {:.6f} %; snubbers i0 {:.5f} % {}, v2 error "
        "{:.6f} %",
        timeStep, magnetizing.noLoadCurrentPercent,
        magnetizing.reactiveSign > 0 ? "leading" : "lagging",
        magnetizing.secondaryVoltageErrorPercent, floating.noLoadCurrentPercent,
        floating.reactiveSign > 0 ? "leading" : "lagging",
        floating.secondaryVoltageErrorPercent, snubbers.noLoadCurrentPercent,
        snubbers.reactiveSign > 0 ? "leading" : "lagging",
        snubbers.secondaryVoltageErrorPercent);
  }

  return 0;
}
