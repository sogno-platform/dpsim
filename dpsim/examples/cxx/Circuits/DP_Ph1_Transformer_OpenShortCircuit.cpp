// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <DPsim.h>

using namespace DPsim;
using namespace CPS;

namespace {

const Real frequency = 50;
const Real timeStep = 1e-4;
const Real finalTime = 1.0;
const Real openResistance = 1e9;
const Real closedResistance = 1e-6;

struct TransformerSpec {
  String name;
  Real nomVoltagePrimary;
  Real nomVoltageSecondary;
  Real ratedPower;
  Real shortCircuitVoltage;
  Real shortCircuitResistance;
  Real noLoadCurrent;
  Real noLoadLoss;
};

Real leakageResistanceOf(const TransformerSpec &spec) {
  return spec.shortCircuitResistance * std::pow(spec.nomVoltagePrimary, 2) /
         spec.ratedPower;
}

Real leakageInductanceOf(const TransformerSpec &spec) {
  return std::sqrt(std::pow(spec.shortCircuitVoltage, 2) -
                   std::pow(spec.shortCircuitResistance, 2)) *
         std::pow(spec.nomVoltagePrimary, 2) / spec.ratedPower /
         (2. * PI * frequency);
}

void energizeDP(const TransformerSpec &spec, const String &simName,
                Bool secondaryClosed) {
  Real ratio = spec.nomVoltagePrimary / spec.nomVoltageSecondary;

  String simNamePF = simName + "_PF";
  Logger::setLogDir("logs/" + simNamePF);

  auto n1PF = SimNode<Complex>::make("n1", PhaseType::Single);
  auto n2PF = SimNode<Complex>::make("n2", PhaseType::Single);

  auto extnetPF = SP::Ph1::NetworkInjection::make("Slack", Logger::Level::off);
  extnetPF->setParameters(spec.nomVoltagePrimary);
  extnetPF->setBaseVoltage(spec.nomVoltagePrimary);
  extnetPF->modifyPowerFlowBusType(PowerflowBusType::VD);

  auto trafoPF = SP::Ph1::Transformer::make(simName, Logger::Level::off);
  trafoPF->setParameters(spec.nomVoltagePrimary, spec.nomVoltageSecondary,
                         spec.ratedPower, ratio, 0., leakageResistanceOf(spec),
                         leakageInductanceOf(spec));
  trafoPF->setMagnetizingBranch(spec.noLoadCurrent, spec.noLoadLoss);
  trafoPF->setBaseVoltage(spec.nomVoltagePrimary);

  auto loadPF = SP::Ph1::Shunt::make("Termination", Logger::Level::off);
  loadPF->setParameters(
      secondaryClosed ? 1. / closedResistance : 1. / openResistance, 0.);
  loadPF->setBaseVoltage(spec.nomVoltageSecondary);

  extnetPF->connect({n1PF});
  trafoPF->connect({n1PF, n2PF});
  loadPF->connect({n2PF});
  auto systemPF =
      SystemTopology(frequency, SystemNodeList{n1PF, n2PF},
                     SystemComponentList{extnetPF, trafoPF, loadPF});

  Simulation simPF(simNamePF, Logger::Level::off);
  simPF.setSystem(systemPF);
  simPF.setTimeStep(finalTime);
  simPF.setFinalTime(2. * finalTime);
  simPF.setDomain(Domain::SP);
  simPF.setSolverType(Solver::Type::NRP);
  simPF.setSolverAndComponentBehaviour(Solver::Behaviour::Initialization);
  simPF.doInitFromNodesAndTerminals(true);
  simPF.run();

  Logger::setLogDir("logs");

  auto n1 = SimNode<Complex>::make("n1", PhaseType::Single);
  auto n2 = SimNode<Complex>::make("n2", PhaseType::Single);

  auto vs = DP::Ph1::VoltageSource::make("vs", Logger::Level::off);
  vs->setParameters(Complex(spec.nomVoltagePrimary, 0), 0.);
  vs->connect({SimNode<Complex>::GND, n1});

  auto trafo =
      DP::Ph1::Transformer::make(simName, simName, Logger::Level::debug, true);
  trafo->setParameters(spec.nomVoltagePrimary, spec.nomVoltageSecondary,
                       spec.ratedPower, ratio, 0., leakageResistanceOf(spec),
                       leakageInductanceOf(spec));
  trafo->setMagnetizingBranch(spec.noLoadCurrent, spec.noLoadLoss);
  trafo->connect({n1, n2});

  auto sw = DP::Ph1::Switch::make("Termination", Logger::Level::off);
  sw->setParameters(openResistance, closedResistance, secondaryClosed);
  sw->connect({n2, SimNode<Complex>::GND});

  auto sys = SystemTopology(frequency, SystemNodeList{n1, n2},
                            SystemComponentList{vs, trafo, sw});
  sys.initWithPowerflow(systemPF, Domain::DP);

  auto logger = DataLogger::make(simName, Logger::Level::off);
  logger->logAttribute("i_primary", vs->attribute("i_intf"));
  logger->logAttribute("v_primary", n1->attribute("v"));
  logger->logAttribute("v_secondary", n2->attribute("v"));

  Simulation sim(simName, Logger::Level::off);
  sim.setSystem(sys);
  sim.setDomain(Domain::DP);
  sim.setTimeStep(timeStep);
  sim.setFinalTime(finalTime);
  sim.addLogger(logger);
  sim.run();
}

void energizeSP(const TransformerSpec &spec, const String &simName,
                Bool secondaryClosed) {
  Real ratio = spec.nomVoltagePrimary / spec.nomVoltageSecondary;

  auto n1 = SimNode<Complex>::make("n1", PhaseType::Single);
  auto n2 = SimNode<Complex>::make("n2", PhaseType::Single);

  auto vs = SP::Ph1::VoltageSource::make("vs", Logger::Level::off);
  vs->setParameters(Complex(spec.nomVoltagePrimary, 0), 0.);
  vs->connect({SimNode<Complex>::GND, n1});

  auto trafo =
      SP::Ph1::Transformer::make(simName, simName, Logger::Level::off, true);
  trafo->setParameters(spec.nomVoltagePrimary, spec.nomVoltageSecondary,
                       spec.ratedPower, ratio, 0., leakageResistanceOf(spec),
                       leakageInductanceOf(spec));
  trafo->setMagnetizingBranch(spec.noLoadCurrent, spec.noLoadLoss);
  trafo->connect({n1, n2});

  auto sw = SP::Ph1::Switch::make("Termination", Logger::Level::off);
  sw->setParameters(openResistance, closedResistance, secondaryClosed);
  sw->connect({n2, SimNode<Complex>::GND});

  auto sys = SystemTopology(frequency, SystemNodeList{n1, n2},
                            SystemComponentList{vs, trafo, sw});

  auto logger = DataLogger::make(simName, Logger::Level::off);
  logger->logAttribute("i_primary", vs->attribute("i_intf"));
  logger->logAttribute("v_primary", n1->attribute("v"));
  logger->logAttribute("v_secondary", n2->attribute("v"));

  Simulation sim(simName, Logger::Level::off);
  sim.setSystem(sys);
  sim.setDomain(Domain::SP);
  sim.setTimeStep(timeStep);
  sim.setFinalTime(finalTime);
  sim.addLogger(logger);
  sim.run();
}

void energizeDPPh3(const TransformerSpec &spec, const String &simName,
                   Bool secondaryClosed) {
  Real ratio = spec.nomVoltagePrimary / spec.nomVoltageSecondary;

  String simNamePF = simName + "_PF";
  Logger::setLogDir("logs/" + simNamePF);

  auto n1PF = SimNode<Complex>::make("n1", PhaseType::Single);
  auto n2PF = SimNode<Complex>::make("n2", PhaseType::Single);

  auto extnetPF = SP::Ph1::NetworkInjection::make("Slack", Logger::Level::off);
  extnetPF->setParameters(spec.nomVoltagePrimary);
  extnetPF->setBaseVoltage(spec.nomVoltagePrimary);
  extnetPF->modifyPowerFlowBusType(PowerflowBusType::VD);

  auto trafoPF = SP::Ph1::Transformer::make(simName, Logger::Level::off);
  trafoPF->setParameters(spec.nomVoltagePrimary, spec.nomVoltageSecondary,
                         spec.ratedPower, ratio, 0., leakageResistanceOf(spec),
                         leakageInductanceOf(spec));
  trafoPF->setMagnetizingBranch(spec.noLoadCurrent, spec.noLoadLoss);
  trafoPF->setBaseVoltage(spec.nomVoltagePrimary);

  auto loadPF = SP::Ph1::Shunt::make("Termination", Logger::Level::off);
  loadPF->setParameters(
      secondaryClosed ? 1. / closedResistance : 1. / openResistance, 0.);
  loadPF->setBaseVoltage(spec.nomVoltageSecondary);

  extnetPF->connect({n1PF});
  trafoPF->connect({n1PF, n2PF});
  loadPF->connect({n2PF});
  auto systemPF =
      SystemTopology(frequency, SystemNodeList{n1PF, n2PF},
                     SystemComponentList{extnetPF, trafoPF, loadPF});

  Simulation simPF(simNamePF, Logger::Level::off);
  simPF.setSystem(systemPF);
  simPF.setTimeStep(finalTime);
  simPF.setFinalTime(2. * finalTime);
  simPF.setDomain(Domain::SP);
  simPF.setSolverType(Solver::Type::NRP);
  simPF.setSolverAndComponentBehaviour(Solver::Behaviour::Initialization);
  simPF.doInitFromNodesAndTerminals(true);
  simPF.run();

  Logger::setLogDir("logs");

  auto n1 = SimNode<Complex>::make("n1", PhaseType::ABC);
  auto n2 = SimNode<Complex>::make("n2", PhaseType::ABC);

  auto vs = DP::Ph3::VoltageSource::make("vs", Logger::Level::off);
  vs->setParameters(Math::singlePhaseVariableToThreePhase(
                        RMS3PH_TO_PEAK1PH * Complex(spec.nomVoltagePrimary, 0)),
                    0.);
  vs->connect({SimNode<Complex>::GND, n1});

  auto trafo =
      DP::Ph3::Transformer::make(simName, simName, Logger::Level::debug, true);
  trafo->setParameters(
      spec.nomVoltagePrimary, spec.nomVoltageSecondary, spec.ratedPower, ratio,
      0., Math::singlePhaseParameterToThreePhase(leakageResistanceOf(spec)),
      Math::singlePhaseParameterToThreePhase(leakageInductanceOf(spec)));
  trafo->setMagnetizingBranch(spec.noLoadCurrent, spec.noLoadLoss);
  trafo->connect({n1, n2});

  auto term = DP::Ph3::Resistor::make("Termination", Logger::Level::off);
  term->setParameters(Math::singlePhaseParameterToThreePhase(
      secondaryClosed ? closedResistance : openResistance));
  term->connect({n2, SimNode<Complex>::GND});

  auto sys = SystemTopology(frequency, SystemNodeList{n1, n2},
                            SystemComponentList{vs, trafo, term});
  sys.initWithPowerflow(systemPF, Domain::DP);

  auto logger = DataLogger::make(simName, Logger::Level::off);
  logger->logAttribute("i_primary", vs->attribute("i_intf"));
  logger->logAttribute("v_primary", n1->attribute("v"));
  logger->logAttribute("v_secondary", n2->attribute("v"));

  Simulation sim(simName, Logger::Level::off);
  sim.setSystem(sys);
  sim.setDomain(Domain::DP);
  sim.setTimeStep(timeStep);
  sim.setFinalTime(finalTime);
  sim.addLogger(logger);
  sim.run();
}

void energizeSPPh3(const TransformerSpec &spec, const String &simName,
                   Bool secondaryClosed) {
  Real ratio = spec.nomVoltagePrimary / spec.nomVoltageSecondary;

  auto n1 = SimNode<Complex>::make("n1", PhaseType::ABC);
  auto n2 = SimNode<Complex>::make("n2", PhaseType::ABC);

  auto vs = SP::Ph3::VoltageSource::make("vs", Logger::Level::off);
  vs->setParameters(Complex(spec.nomVoltagePrimary, 0));
  vs->connect({SimNode<Complex>::GND, n1});

  auto trafo =
      SP::Ph3::Transformer::make(simName, simName, Logger::Level::off, true);
  trafo->setParameters(
      spec.nomVoltagePrimary, spec.nomVoltageSecondary, spec.ratedPower, ratio,
      0., Math::singlePhaseParameterToThreePhase(leakageResistanceOf(spec)),
      Math::singlePhaseParameterToThreePhase(leakageInductanceOf(spec)));
  trafo->setMagnetizingBranch(spec.noLoadCurrent, spec.noLoadLoss);
  trafo->connect({n1, n2});

  auto term = SP::Ph3::Resistor::make("Termination", Logger::Level::off);
  term->setParameters(Math::singlePhaseParameterToThreePhase(
      secondaryClosed ? closedResistance : openResistance));
  term->connect({n2, SimNode<Complex>::GND});

  auto sys = SystemTopology(frequency, SystemNodeList{n1, n2},
                            SystemComponentList{vs, trafo, term});

  auto logger = DataLogger::make(simName, Logger::Level::off);
  logger->logAttribute("i_primary", vs->attribute("i_intf"));
  logger->logAttribute("v_primary", n1->attribute("v"));
  logger->logAttribute("v_secondary", n2->attribute("v"));

  Simulation sim(simName, Logger::Level::off);
  sim.setSystem(sys);
  sim.setDomain(Domain::SP);
  sim.setTimeStep(timeStep);
  sim.setFinalTime(finalTime);
  sim.addLogger(logger);
  sim.run();
}

void energizeEMT(const TransformerSpec &spec, const String &simName,
                 Bool secondaryClosed) {
  Real ratio = spec.nomVoltagePrimary / spec.nomVoltageSecondary;

  String simNamePF = simName + "_PF";
  Logger::setLogDir("logs/" + simNamePF);

  auto n1PF = SimNode<Complex>::make("n1", PhaseType::Single);
  auto n2PF = SimNode<Complex>::make("n2", PhaseType::Single);

  auto extnetPF = SP::Ph1::NetworkInjection::make("Slack", Logger::Level::off);
  extnetPF->setParameters(spec.nomVoltagePrimary);
  extnetPF->setBaseVoltage(spec.nomVoltagePrimary);
  extnetPF->modifyPowerFlowBusType(PowerflowBusType::VD);

  auto trafoPF = SP::Ph1::Transformer::make(simName, Logger::Level::off);
  trafoPF->setParameters(spec.nomVoltagePrimary, spec.nomVoltageSecondary,
                         spec.ratedPower, ratio, 0., leakageResistanceOf(spec),
                         leakageInductanceOf(spec));
  trafoPF->setMagnetizingBranch(spec.noLoadCurrent, spec.noLoadLoss);
  trafoPF->setBaseVoltage(spec.nomVoltagePrimary);

  auto loadPF = SP::Ph1::Shunt::make("Termination", Logger::Level::off);
  loadPF->setParameters(
      secondaryClosed ? 1. / closedResistance : 1. / openResistance, 0.);
  loadPF->setBaseVoltage(spec.nomVoltageSecondary);

  extnetPF->connect({n1PF});
  trafoPF->connect({n1PF, n2PF});
  loadPF->connect({n2PF});
  auto systemPF =
      SystemTopology(frequency, SystemNodeList{n1PF, n2PF},
                     SystemComponentList{extnetPF, trafoPF, loadPF});

  Simulation simPF(simNamePF, Logger::Level::off);
  simPF.setSystem(systemPF);
  simPF.setTimeStep(finalTime);
  simPF.setFinalTime(2. * finalTime);
  simPF.setDomain(Domain::SP);
  simPF.setSolverType(Solver::Type::NRP);
  simPF.setSolverAndComponentBehaviour(Solver::Behaviour::Initialization);
  simPF.doInitFromNodesAndTerminals(true);
  simPF.run();

  Logger::setLogDir("logs");

  auto n1 = SimNode<Real>::make("n1", PhaseType::ABC);
  auto n2 = SimNode<Real>::make("n2", PhaseType::ABC);

  auto vs = EMT::Ph3::VoltageSource::make("vs", Logger::Level::off);
  vs->setParameters(
      Math::singlePhaseVariableToThreePhase(Complex(spec.nomVoltagePrimary, 0)),
      frequency);
  vs->connect({SimNode<Real>::GND, n1});

  auto trafo =
      EMT::Ph3::Transformer::make(simName, simName, Logger::Level::off, true);
  trafo->setParameters(
      spec.nomVoltagePrimary, spec.nomVoltageSecondary, spec.ratedPower, ratio,
      0., Math::singlePhaseParameterToThreePhase(leakageResistanceOf(spec)),
      Math::singlePhaseParameterToThreePhase(leakageInductanceOf(spec)));
  trafo->setMagnetizingBranch(spec.noLoadCurrent, spec.noLoadLoss);
  trafo->connect({n1, n2});

  auto sw = EMT::Ph3::Switch::make("Termination", Logger::Level::off);
  sw->setParameters(Math::singlePhaseParameterToThreePhase(openResistance),
                    Math::singlePhaseParameterToThreePhase(closedResistance),
                    secondaryClosed);
  sw->connect({n2, SimNode<Real>::GND});

  auto sys = SystemTopology(frequency, SystemNodeList{n1, n2},
                            SystemComponentList{vs, trafo, sw});
  sys.initWithPowerflow(systemPF, Domain::EMT);

  auto logger = DataLogger::make(simName, Logger::Level::off);
  logger->logAttribute("i_primary", vs->attribute("i_intf"));
  logger->logAttribute("v_primary", n1->attribute("v"));
  logger->logAttribute("v_secondary", n2->attribute("v"));

  Simulation sim(simName, Logger::Level::off);
  sim.setSystem(sys);
  sim.setDomain(Domain::EMT);
  sim.setTimeStep(timeStep);
  sim.setFinalTime(finalTime);
  sim.addLogger(logger);
  sim.run();
}

} // namespace

int main(int argc, char *argv[]) {
  std::error_code ec;
  std::filesystem::create_directories("./logs", ec);
  auto log = Logger::get("transformer_tests", Logger::Level::info,
                         Logger::Level::info);

  std::vector<TransformerSpec> specs = {
      {"unit_110_10", 110e3, 10e3, 40e6, 0.12, 0.005, 0.010, 1e-3},
      {"unit_132_33", 132e3, 33e3, 25e6, 0.10, 0.004, 0.005, 8e-4},
      {"unit_230_16", 230e3, 16.5e3, 100e6, 0.0576, 0.002, 0.020, 2e-3},
  };

  for (const auto &spec : specs) {
    SPDLOG_LOGGER_INFO(
        log, "{}: {} MVA, {} / {} kV, uk {}, uRr {}, i0 {}, P0 {}", spec.name,
        spec.ratedPower / 1e6, spec.nomVoltagePrimary / 1e3,
        spec.nomVoltageSecondary / 1e3, spec.shortCircuitVoltage,
        spec.shortCircuitResistance, spec.noLoadCurrent, spec.noLoadLoss);
    Logger::setLogDir("logs");
    energizeDP(spec, spec.name + "_DP_open", false);
    energizeDP(spec, spec.name + "_DP_short", true);
    energizeSP(spec, spec.name + "_SP_open", false);
    energizeSP(spec, spec.name + "_SP_short", true);
    energizeEMT(spec, spec.name + "_EMT_open", false);
    energizeEMT(spec, spec.name + "_EMT_short", true);
    energizeDPPh3(spec, spec.name + "_DPPh3_open", false);
    energizeDPPh3(spec, spec.name + "_DPPh3_short", true);
    energizeSPPh3(spec, spec.name + "_SPPh3_open", false);
    energizeSPPh3(spec, spec.name + "_SPPh3_short", true);
  }

  return 0;
}
