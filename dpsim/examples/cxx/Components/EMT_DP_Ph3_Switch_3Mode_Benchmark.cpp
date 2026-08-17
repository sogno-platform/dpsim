// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <vector>

#include <DPsim.h>
#include <dpsim-models/DP/DP_Ph3_Switch.h>
#include <dpsim-models/EMT/EMT_Ph3_Switch.h>
#include <dpsim-models/Filesystem.h>

using namespace DPsim;
using namespace CPS;

enum class BenchmarkMode {
  Ideal,
  CurrentZero,
  ExponentialZCSEmulation,
};

enum class SwitchingDirection {
  CloseToOpen,
  OpenToClose,
};

struct Parameters {
  Real frequency = 50.0;
  Real nominalVoltage = 20e3;

  Real loadP = 1.0e6;
  Real loadQ = 0.3e6;

  Real pathAResistance = 0.20;
  Real pathAInductance = 1.0e-3;

  Real pathBResistance = 0.40;
  Real pathBInductance = 2.0e-3;

  Real switchOpenResistance = 1e9;
  Real switchClosedResistance = 1e-3;

  Real zeroCrossingTolerance = 1e-6;

  Real exponentialSwitchingTime = 0.010;

  Real timeStep = 20e-6;

  Real openCommandTime = 0.0437;

  Real closeCommandTime = 0.0437;

  Real finalTime = 0.070;
};

struct RunResult {
  String domain;
  String mode;
  String direction;

  Bool initialClosed = false;
  Bool targetClosed = false;

  Real commandTime = 0.0;
  Real runtimeSeconds = 0.0;

  Real zeroA = -1.0;
  Real zeroB = -1.0;
  Real zeroC = -1.0;

  Real exponentialStart = -1.0;
  Real exponentialEnd = -1.0;
  Real exponentialProgress = 0.0;
  Bool exponentialActiveFinal = false;

  Real finalResistanceA = -1.0;
  Real finalResistanceB = -1.0;
  Real finalResistanceC = -1.0;

  Bool finalCommandedClosed = false;
  Bool finalPoleA = false;
  Bool finalPoleB = false;
  Bool finalPoleC = false;

  Int expectedMatrixRecomputations = 0;

  Bool passCommandState = false;
  Bool passPoleState = false;
  Bool passResistanceState = false;
  Bool passTransitionState = false;
  Bool passOverall = false;
};

String modeName(BenchmarkMode mode) {
  switch (mode) {
  case BenchmarkMode::Ideal:
    return "Ideal";
  case BenchmarkMode::CurrentZero:
    return "CurrentZero";
  case BenchmarkMode::ExponentialZCSEmulation:
    return "ExponentialZCSEmulation";
  }

  return "Unknown";
}

String directionName(SwitchingDirection direction) {
  switch (direction) {
  case SwitchingDirection::CloseToOpen:
    return "CloseToOpen";
  case SwitchingDirection::OpenToClose:
    return "OpenToClose";
  }

  return "Unknown";
}

Bool initialClosed(SwitchingDirection direction) {
  return direction == SwitchingDirection::CloseToOpen;
}

Bool targetClosed(SwitchingDirection direction) {
  return direction == SwitchingDirection::OpenToClose;
}

Real commandTime(const Parameters &p, SwitchingDirection direction) {
  return direction == SwitchingDirection::CloseToOpen ? p.openCommandTime
                                                      : p.closeCommandTime;
}

Int expectedMatrixRecomputations(const Parameters &p, BenchmarkMode mode,
                                 SwitchingDirection direction) {
  if (mode == BenchmarkMode::Ideal) {
    return 0;
  }

  if (direction == SwitchingDirection::OpenToClose) {
    return 1;
  }

  if (mode == BenchmarkMode::CurrentZero) {
    return 3;
  }

  return static_cast<Int>(std::ceil(p.exponentialSwitchingTime / p.timeStep)) +
         1;
}

Bool nearlyEqual(Real a, Real b, Real relTol = 1e-9, Real absTol = 1e-12) {
  const Real scale = std::max(std::abs(a), std::abs(b));

  return std::abs(a - b) <= std::max(absTol, relTol * scale);
}

void evaluateRunResult(const Parameters &p, RunResult &result) {
  const Bool expectedClosed = result.targetClosed;

  result.passCommandState = result.finalCommandedClosed == expectedClosed;

  result.passPoleState = result.finalPoleA == expectedClosed &&
                         result.finalPoleB == expectedClosed &&
                         result.finalPoleC == expectedClosed;

  const Real expectedResistance =
      expectedClosed ? p.switchClosedResistance : p.switchOpenResistance;

  result.passResistanceState =
      nearlyEqual(result.finalResistanceA, expectedResistance) &&
      nearlyEqual(result.finalResistanceB, expectedResistance) &&
      nearlyEqual(result.finalResistanceC, expectedResistance);

  if (result.mode == "ExponentialZCSEmulation") {
    if (expectedClosed) {
      result.passTransitionState =
          !result.exponentialActiveFinal &&
          nearlyEqual(result.exponentialProgress, 0.0, 0.0, 1e-12);
    } else {
      result.passTransitionState =
          !result.exponentialActiveFinal &&
          nearlyEqual(result.exponentialProgress, 1.0, 0.0, 1e-12) &&
          result.exponentialStart >= 0.0 &&
          result.exponentialEnd >= result.exponentialStart;
    }
  } else {
    result.passTransitionState = !result.exponentialActiveFinal;
  }

  result.passOverall = result.passCommandState && result.passPoleState &&
                       result.passResistanceState && result.passTransitionState;
}

String makeRunTimestamp() {
  const auto now = std::chrono::system_clock::now();

  const std::time_t nowTime = std::chrono::system_clock::to_time_t(now);

  std::tm localTime{};

#ifdef _WIN32
  localtime_s(&localTime, &nowTime);
#else
  localtime_r(&nowTime, &localTime);
#endif

  std::ostringstream stream;
  stream << std::put_time(&localTime, "%Y%m%d_%H%M%S");

  return stream.str();
}

String makeResultsDirectory() {
  return "logs/"
         "EMT_DP_Ph3_Switch_3Mode_Bidirectional_Benchmark_results_" +
         makeRunTimestamp();
}

void writeSummary(const String &resultsDir,
                  const std::vector<RunResult> &results) {
  const String path = resultsDir + "/benchmark_summary.csv";

  std::ofstream csv(path, std::ios::out | std::ios::trunc);

  if (!csv.is_open()) {
    throw std::runtime_error("Could not open benchmark summary: " + path);
  }

  csv << std::scientific << std::setprecision(12);

  csv << "domain,"
      << "mode,"
      << "direction,"
      << "initial_closed,"
      << "target_closed,"
      << "command_time_s,"
      << "runtime_s,"
      << "zero_a_s,"
      << "zero_b_s,"
      << "zero_c_s,"
      << "exponential_start_s,"
      << "exponential_end_s,"
      << "exponential_progress_final,"
      << "exponential_active_final,"
      << "final_R_a_ohm,"
      << "final_R_b_ohm,"
      << "final_R_c_ohm,"
      << "final_commanded_closed,"
      << "final_pole_a_closed,"
      << "final_pole_b_closed,"
      << "final_pole_c_closed,"
      << "expected_matrix_recomputations,"
      << "pass_command_state,"
      << "pass_pole_state,"
      << "pass_resistance_state,"
      << "pass_transition_state,"
      << "pass_overall\n";

  for (const auto &r : results) {
    csv << r.domain << "," << r.mode << "," << r.direction << ","
        << static_cast<Int>(r.initialClosed) << ","
        << static_cast<Int>(r.targetClosed) << "," << r.commandTime << ","
        << r.runtimeSeconds << "," << r.zeroA << "," << r.zeroB << ","
        << r.zeroC << "," << r.exponentialStart << "," << r.exponentialEnd
        << "," << r.exponentialProgress << ","
        << static_cast<Int>(r.exponentialActiveFinal) << ","
        << r.finalResistanceA << "," << r.finalResistanceB << ","
        << r.finalResistanceC << "," << static_cast<Int>(r.finalCommandedClosed)
        << "," << static_cast<Int>(r.finalPoleA) << ","
        << static_cast<Int>(r.finalPoleB) << ","
        << static_cast<Int>(r.finalPoleC) << ","
        << r.expectedMatrixRecomputations << ","
        << static_cast<Int>(r.passCommandState) << ","
        << static_cast<Int>(r.passPoleState) << ","
        << static_cast<Int>(r.passResistanceState) << ","
        << static_cast<Int>(r.passTransitionState) << ","
        << static_cast<Int>(r.passOverall) << "\n";
  }
}

SystemTopology runPowerFlow(const Parameters &p, Bool breakerClosed) {
  const String stateName = breakerClosed ? "Closed" : "Open";

  const String simName =
      "EMT_DP_Ph3_Switch_3Mode_Bidirectional_Benchmark_PF_" + stateName;

  auto n1 = SimNode<Complex>::make("n1", PhaseType::Single);

  auto n2 = SimNode<Complex>::make("n2", PhaseType::Single);

  auto n3 = SimNode<Complex>::make("n3", PhaseType::Single);

  auto slack = SP::Ph1::NetworkInjection::make("Slack", Logger::Level::info);

  slack->setParameters(p.nominalVoltage);

  slack->setBaseVoltage(p.nominalVoltage);

  slack->modifyPowerFlowBusType(PowerflowBusType::VD);

  auto pathA = SP::Ph1::PiLine::make("PathA", Logger::Level::info);

  pathA->setParameters(p.pathAResistance, p.pathAInductance, 0.0);

  pathA->setBaseVoltage(p.nominalVoltage);

  auto pathB = SP::Ph1::PiLine::make("PathB", Logger::Level::info);

  pathB->setParameters(p.pathBResistance, p.pathBInductance, 0.0);

  pathB->setBaseVoltage(p.nominalVoltage);

  auto load = SP::Ph1::Shunt::make("LoadPF", Logger::Level::info);

  load->setParameters(p.loadP / std::pow(p.nominalVoltage, 2),
                      -p.loadQ / std::pow(p.nominalVoltage, 2));

  load->setBaseVoltage(p.nominalVoltage);

  slack->connect({n1});
  pathA->connect({n1, n2});
  pathB->connect({n1, n3});
  load->connect({n3});

  SystemComponentList components{
      slack,
      pathA,
      pathB,
      load,
  };

  if (breakerClosed) {
    auto breakerPF = SP::Ph1::PiLine::make("BreakerPF", Logger::Level::info);

    breakerPF->setParameters(p.switchClosedResistance, 0.0, 0.0);

    breakerPF->setBaseVoltage(p.nominalVoltage);

    breakerPF->connect({n2, n3});

    components.push_back(breakerPF);
  }

  auto systemPF = SystemTopology(p.frequency,
                                 SystemNodeList{
                                     n1,
                                     n2,
                                     n3,
                                 },
                                 components);

  auto logger = DataLogger::make(simName);

  logger->logAttribute("v_n1", n1->attribute("v"));

  logger->logAttribute("v_n2", n2->attribute("v"));

  logger->logAttribute("v_n3", n3->attribute("v"));

  Simulation sim(simName, Logger::Level::info);

  sim.setSystem(systemPF);

  sim.setTimeStep(1.0);

  sim.setFinalTime(2.0);

  sim.setDomain(Domain::SP);

  sim.setSolverType(Solver::Type::NRP);

  sim.setSolverAndComponentBehaviour(Solver::Behaviour::Initialization);

  sim.doInitFromNodesAndTerminals(false);

  sim.addLogger(logger);

  sim.run();

  return systemPF;
}

EMT::Ph3::Switch::SwitchingMode toEMTMode(BenchmarkMode mode) {
  switch (mode) {
  case BenchmarkMode::Ideal:
    return EMT::Ph3::Switch::SwitchingMode::Ideal;

  case BenchmarkMode::CurrentZero:
    return EMT::Ph3::Switch::SwitchingMode::CurrentZero;

  case BenchmarkMode::ExponentialZCSEmulation:
    return EMT::Ph3::Switch::SwitchingMode::ExponentialZCSEmulation;
  }

  return EMT::Ph3::Switch::SwitchingMode::Ideal;
}

RunResult runEMT(const Parameters &p, const SystemTopology &systemPF,
                 BenchmarkMode mode, SwitchingDirection direction) {
  const Bool startsClosed = initialClosed(direction);

  const Bool endsClosed = targetClosed(direction);

  const Real eventTime = commandTime(p, direction);

  const String simName =
      "EMT_DP_Ph3_Switch_3Mode_Bidirectional_Benchmark_EMT_" + modeName(mode) +
      "_" + directionName(direction);

  auto n1 = SimNode<Real>::make("n1", PhaseType::ABC);

  auto n2 = SimNode<Real>::make("n2", PhaseType::ABC);

  auto n3 = SimNode<Real>::make("n3", PhaseType::ABC);

  auto slack = EMT::Ph3::NetworkInjection::make("Slack", Logger::Level::info);

  slack->setParameters(Math::singlePhaseVariableToThreePhase(p.nominalVoltage),
                       p.frequency);

  auto pathA = EMT::Ph3::PiLine::make("PathA", Logger::Level::info);

  pathA->setParameters(
      Math::singlePhaseParameterToThreePhase(p.pathAResistance),
      Math::singlePhaseParameterToThreePhase(p.pathAInductance),
      Math::singlePhaseParameterToThreePhase(0.0));

  auto pathB = EMT::Ph3::PiLine::make("PathB", Logger::Level::info);

  pathB->setParameters(
      Math::singlePhaseParameterToThreePhase(p.pathBResistance),
      Math::singlePhaseParameterToThreePhase(p.pathBInductance),
      Math::singlePhaseParameterToThreePhase(0.0));

  auto breaker = EMT::Ph3::Switch::make("Breaker", Logger::Level::info);

  breaker->setParameters(
      Math::singlePhaseParameterToThreePhase(p.switchOpenResistance),
      Math::singlePhaseParameterToThreePhase(p.switchClosedResistance),
      startsClosed);

  breaker->setSwitchingMode(toEMTMode(mode));

  breaker->setZeroCrossingTolerance(p.zeroCrossingTolerance);

  breaker->setExponentialSwitchingTime(p.exponentialSwitchingTime);

  auto load = EMT::Ph3::RXLoad::make("Load", Logger::Level::info);

  load->setParameters(Math::singlePhasePowerToThreePhase(p.loadP),
                      Math::singlePhasePowerToThreePhase(p.loadQ),
                      p.nominalVoltage);

  slack->connect({n1});
  pathA->connect({n1, n2});
  breaker->connect({n2, n3});
  pathB->connect({n1, n3});
  load->connect({n3});

  auto system = SystemTopology(p.frequency,
                               SystemNodeList{
                                   n1,
                                   n2,
                                   n3,
                               },
                               SystemComponentList{
                                   slack,
                                   pathA,
                                   breaker,
                                   pathB,
                                   load,
                               });

  system.initWithPowerflow(systemPF, Domain::EMT);

  auto logger = DataLogger::make(simName);

  const auto breakerCurrent = breaker->attributeTyped<Matrix>("i_intf");

  logger->logAttribute("i_breaker_a", breakerCurrent->deriveCoeff<Real>(0, 0));

  logger->logAttribute("i_breaker_b", breakerCurrent->deriveCoeff<Real>(1, 0));

  logger->logAttribute("i_breaker_c", breakerCurrent->deriveCoeff<Real>(2, 0));

  const auto breakerVoltage = breaker->attributeTyped<Matrix>("v_intf");

  logger->logAttribute("v_breaker_a", breakerVoltage->deriveCoeff<Real>(0, 0));

  logger->logAttribute("v_breaker_b", breakerVoltage->deriveCoeff<Real>(1, 0));

  logger->logAttribute("v_breaker_c", breakerVoltage->deriveCoeff<Real>(2, 0));

  const auto loadBusVoltage = n3->attributeTyped<Matrix>("v");

  logger->logAttribute("v_n3_a", loadBusVoltage->deriveCoeff<Real>(0, 0));

  logger->logAttribute("v_n3_b", loadBusVoltage->deriveCoeff<Real>(1, 0));

  logger->logAttribute("v_n3_c", loadBusVoltage->deriveCoeff<Real>(2, 0));

  logger->logAttribute("commanded_closed", breaker->attribute("is_closed"));

  logger->logAttribute("opening_requested",
                       breaker->attribute("opening_requested"));

  logger->logAttribute("pole_closed_a", breaker->attribute("pole_closed_a"));

  logger->logAttribute("pole_closed_b", breaker->attribute("pole_closed_b"));

  logger->logAttribute("pole_closed_c", breaker->attribute("pole_closed_c"));

  logger->logAttribute("zero_crossing_time_a",
                       breaker->attribute("zero_crossing_time_a"));

  logger->logAttribute("zero_crossing_time_b",
                       breaker->attribute("zero_crossing_time_b"));

  logger->logAttribute("zero_crossing_time_c",
                       breaker->attribute("zero_crossing_time_c"));

  logger->logAttribute("exponential_transition_active",
                       breaker->attribute("exponential_transition_active"));

  logger->logAttribute("exponential_progress",
                       breaker->attribute("exponential_progress"));

  logger->logAttribute("exponential_transition_start_time",
                       breaker->attribute("exponential_transition_start_time"));

  logger->logAttribute("exponential_transition_end_time",
                       breaker->attribute("exponential_transition_end_time"));

  logger->logAttribute("effective_resistance_a",
                       breaker->attribute("effective_resistance_a"));

  logger->logAttribute("effective_resistance_b",
                       breaker->attribute("effective_resistance_b"));

  logger->logAttribute("effective_resistance_c",
                       breaker->attribute("effective_resistance_c"));

  Simulation sim(simName, Logger::Level::info);

  sim.setSystem(system);

  sim.setTimeStep(p.timeStep);

  sim.setFinalTime(p.finalTime);

  sim.setDomain(Domain::EMT);

  sim.setSolverType(Solver::Type::MNA);

  sim.addLogger(logger);

  auto event = SwitchEvent3Ph::make(eventTime, breaker, endsClosed);

  sim.addEvent(event);

  const auto wallStart = std::chrono::steady_clock::now();

  sim.run();

  const auto wallEnd = std::chrono::steady_clock::now();

  RunResult result;

  result.domain = "EMT";

  result.mode = modeName(mode);

  result.direction = directionName(direction);

  result.initialClosed = startsClosed;

  result.targetClosed = endsClosed;

  result.commandTime = eventTime;

  result.runtimeSeconds =
      std::chrono::duration<Real>(wallEnd - wallStart).count();

  result.zeroA = **breaker->attributeTyped<Real>("zero_crossing_time_a");

  result.zeroB = **breaker->attributeTyped<Real>("zero_crossing_time_b");

  result.zeroC = **breaker->attributeTyped<Real>("zero_crossing_time_c");

  result.exponentialStart =
      **breaker->attributeTyped<Real>("exponential_transition_start_time");

  result.exponentialEnd =
      **breaker->attributeTyped<Real>("exponential_transition_end_time");

  result.exponentialProgress =
      **breaker->attributeTyped<Real>("exponential_progress");

  result.exponentialActiveFinal =
      **breaker->attributeTyped<Bool>("exponential_transition_active");

  result.finalResistanceA =
      **breaker->attributeTyped<Real>("effective_resistance_a");

  result.finalResistanceB =
      **breaker->attributeTyped<Real>("effective_resistance_b");

  result.finalResistanceC =
      **breaker->attributeTyped<Real>("effective_resistance_c");

  result.finalCommandedClosed = **breaker->attributeTyped<Bool>("is_closed");

  result.finalPoleA = **breaker->attributeTyped<Bool>("pole_closed_a");

  result.finalPoleB = **breaker->attributeTyped<Bool>("pole_closed_b");

  result.finalPoleC = **breaker->attributeTyped<Bool>("pole_closed_c");

  result.expectedMatrixRecomputations =
      expectedMatrixRecomputations(p, mode, direction);

  evaluateRunResult(p, result);

  return result;
}

DP::Ph3::Switch::SwitchingMode toDPMode(BenchmarkMode mode) {
  switch (mode) {
  case BenchmarkMode::Ideal:
    return DP::Ph3::Switch::SwitchingMode::Ideal;

  case BenchmarkMode::CurrentZero:
    return DP::Ph3::Switch::SwitchingMode::CurrentZero;

  case BenchmarkMode::ExponentialZCSEmulation:
    return DP::Ph3::Switch::SwitchingMode::ExponentialZCSEmulation;
  }

  return DP::Ph3::Switch::SwitchingMode::Ideal;
}

RunResult runDP(const Parameters &p, const SystemTopology &systemPF,
                BenchmarkMode mode, SwitchingDirection direction) {
  const Bool startsClosed = initialClosed(direction);

  const Bool endsClosed = targetClosed(direction);

  const Real eventTime = commandTime(p, direction);

  const String simName = "EMT_DP_Ph3_Switch_3Mode_Bidirectional_Benchmark_DP_" +
                         modeName(mode) + "_" + directionName(direction);

  auto n1 = DP::SimNode::make("n1", PhaseType::ABC);

  auto n2 = DP::SimNode::make("n2", PhaseType::ABC);

  auto n3 = DP::SimNode::make("n3", PhaseType::ABC);

  auto slack = DP::Ph3::NetworkInjection::make("Slack", Logger::Level::info);

  auto pathA = DP::Ph3::PiLine::make("PathA", Logger::Level::info);

  pathA->setParameters(
      Math::singlePhaseParameterToThreePhase(p.pathAResistance),
      Math::singlePhaseParameterToThreePhase(p.pathAInductance),
      Matrix::Zero(3, 3), Matrix::Zero(3, 3));

  auto pathB = DP::Ph3::PiLine::make("PathB", Logger::Level::info);

  pathB->setParameters(
      Math::singlePhaseParameterToThreePhase(p.pathBResistance),
      Math::singlePhaseParameterToThreePhase(p.pathBInductance),
      Matrix::Zero(3, 3), Matrix::Zero(3, 3));

  auto breaker = DP::Ph3::Switch::make("Breaker", Logger::Level::info);

  breaker->setParameters(
      Math::singlePhaseParameterToThreePhase(p.switchOpenResistance),
      Math::singlePhaseParameterToThreePhase(p.switchClosedResistance),
      startsClosed);

  breaker->setSwitchingMode(toDPMode(mode));

  breaker->setZeroCrossingTolerance(p.zeroCrossingTolerance);

  breaker->setExponentialSwitchingTime(p.exponentialSwitchingTime);

  const Real denominator = p.loadP * p.loadP + p.loadQ * p.loadQ;

  const Real loadResistance =
      p.nominalVoltage * p.nominalVoltage * p.loadP / denominator;

  const Real loadReactance =
      p.nominalVoltage * p.nominalVoltage * p.loadQ / denominator;

  const Real loadInductance = loadReactance / (2.0 * PI * p.frequency);

  auto load = DP::Ph3::PiLine::make("Load", Logger::Level::info);

  load->setParameters(Math::singlePhaseParameterToThreePhase(loadResistance),
                      Math::singlePhaseParameterToThreePhase(loadInductance),
                      Matrix::Zero(3, 3), Matrix::Zero(3, 3));

  slack->connect({n1});
  pathA->connect({n1, n2});
  breaker->connect({n2, n3});
  pathB->connect({n1, n3});

  load->connect({
      DP::SimNode::GND,
      n3,
  });

  auto system = SystemTopology(p.frequency,
                               SystemNodeList{
                                   n1,
                                   n2,
                                   n3,
                               },
                               SystemComponentList{
                                   slack,
                                   pathA,
                                   breaker,
                                   pathB,
                                   load,
                               });

  system.initWithPowerflow(systemPF, Domain::DP);

  const Complex slackPhasePeak = RMS3PH_TO_PEAK1PH * n1->initialSingleVoltage();

  slack->setParameters(Math::singlePhaseVariableToThreePhase(slackPhasePeak),
                       0.0);

  auto logger = DataLogger::make(simName);

  const auto breakerCurrent = breaker->attributeTyped<MatrixComp>("i_intf");

  logger->logAttribute("i_breaker_a_env",
                       breakerCurrent->deriveCoeff<Complex>(0, 0));

  logger->logAttribute("i_breaker_b_env",
                       breakerCurrent->deriveCoeff<Complex>(1, 0));

  logger->logAttribute("i_breaker_c_env",
                       breakerCurrent->deriveCoeff<Complex>(2, 0));

  logger->logAttribute("i_instantaneous_a",
                       breaker->attribute("i_instantaneous_a"));

  logger->logAttribute("i_instantaneous_b",
                       breaker->attribute("i_instantaneous_b"));

  logger->logAttribute("i_instantaneous_c",
                       breaker->attribute("i_instantaneous_c"));

  const auto breakerVoltage = breaker->attributeTyped<MatrixComp>("v_intf");

  logger->logAttribute("v_breaker_a_env",
                       breakerVoltage->deriveCoeff<Complex>(0, 0));

  logger->logAttribute("v_breaker_b_env",
                       breakerVoltage->deriveCoeff<Complex>(1, 0));

  logger->logAttribute("v_breaker_c_env",
                       breakerVoltage->deriveCoeff<Complex>(2, 0));

  const auto loadBusVoltage = n3->attributeTyped<MatrixComp>("v");

  logger->logAttribute("v_n3_a_env",
                       loadBusVoltage->deriveCoeff<Complex>(0, 0));

  logger->logAttribute("v_n3_b_env",
                       loadBusVoltage->deriveCoeff<Complex>(1, 0));

  logger->logAttribute("v_n3_c_env",
                       loadBusVoltage->deriveCoeff<Complex>(2, 0));

  logger->logAttribute("commanded_closed", breaker->attribute("is_closed"));

  logger->logAttribute("opening_requested",
                       breaker->attribute("opening_requested"));

  logger->logAttribute("pole_closed_a", breaker->attribute("pole_closed_a"));

  logger->logAttribute("pole_closed_b", breaker->attribute("pole_closed_b"));

  logger->logAttribute("pole_closed_c", breaker->attribute("pole_closed_c"));

  logger->logAttribute("zero_crossing_time_a",
                       breaker->attribute("zero_crossing_time_a"));

  logger->logAttribute("zero_crossing_time_b",
                       breaker->attribute("zero_crossing_time_b"));

  logger->logAttribute("zero_crossing_time_c",
                       breaker->attribute("zero_crossing_time_c"));

  logger->logAttribute("exponential_transition_active",
                       breaker->attribute("exponential_transition_active"));

  logger->logAttribute("exponential_progress",
                       breaker->attribute("exponential_progress"));

  logger->logAttribute("exponential_transition_start_time",
                       breaker->attribute("exponential_transition_start_time"));

  logger->logAttribute("exponential_transition_end_time",
                       breaker->attribute("exponential_transition_end_time"));

  logger->logAttribute("effective_resistance_a",
                       breaker->attribute("effective_resistance_a"));

  logger->logAttribute("effective_resistance_b",
                       breaker->attribute("effective_resistance_b"));

  logger->logAttribute("effective_resistance_c",
                       breaker->attribute("effective_resistance_c"));

  Simulation sim(simName, Logger::Level::info);

  sim.setSystem(system);

  sim.setTimeStep(p.timeStep);

  sim.setFinalTime(p.finalTime);

  sim.setDomain(Domain::DP);

  sim.setSolverType(Solver::Type::MNA);

  sim.addLogger(logger);

  auto event = SwitchEvent3Ph::make(eventTime, breaker, endsClosed);

  sim.addEvent(event);

  const auto wallStart = std::chrono::steady_clock::now();

  sim.run();

  const auto wallEnd = std::chrono::steady_clock::now();

  RunResult result;

  result.domain = "DP";

  result.mode = modeName(mode);

  result.direction = directionName(direction);

  result.initialClosed = startsClosed;

  result.targetClosed = endsClosed;

  result.commandTime = eventTime;

  result.runtimeSeconds =
      std::chrono::duration<Real>(wallEnd - wallStart).count();

  result.zeroA = **breaker->attributeTyped<Real>("zero_crossing_time_a");

  result.zeroB = **breaker->attributeTyped<Real>("zero_crossing_time_b");

  result.zeroC = **breaker->attributeTyped<Real>("zero_crossing_time_c");

  result.exponentialStart =
      **breaker->attributeTyped<Real>("exponential_transition_start_time");

  result.exponentialEnd =
      **breaker->attributeTyped<Real>("exponential_transition_end_time");

  result.exponentialProgress =
      **breaker->attributeTyped<Real>("exponential_progress");

  result.exponentialActiveFinal =
      **breaker->attributeTyped<Bool>("exponential_transition_active");

  result.finalResistanceA =
      **breaker->attributeTyped<Real>("effective_resistance_a");

  result.finalResistanceB =
      **breaker->attributeTyped<Real>("effective_resistance_b");

  result.finalResistanceC =
      **breaker->attributeTyped<Real>("effective_resistance_c");

  result.finalCommandedClosed = **breaker->attributeTyped<Bool>("is_closed");

  result.finalPoleA = **breaker->attributeTyped<Bool>("pole_closed_a");

  result.finalPoleB = **breaker->attributeTyped<Bool>("pole_closed_b");

  result.finalPoleC = **breaker->attributeTyped<Bool>("pole_closed_c");

  result.expectedMatrixRecomputations =
      expectedMatrixRecomputations(p, mode, direction);

  evaluateRunResult(p, result);

  return result;
}

int main() {
  const Parameters p;

  const String resultsDir = makeResultsDirectory();

  fs::create_directories(resultsDir);

  Logger::setLogDir(resultsDir);

  const auto systemPFClosed = runPowerFlow(p, true);

  const auto systemPFOpen = runPowerFlow(p, false);

  const std::vector<BenchmarkMode> modes{
      BenchmarkMode::Ideal,
      BenchmarkMode::CurrentZero,
      BenchmarkMode::ExponentialZCSEmulation,
  };

  const std::vector<SwitchingDirection> directions{
      SwitchingDirection::CloseToOpen,
      SwitchingDirection::OpenToClose,
  };

  std::vector<RunResult> results;
  results.reserve(12);

  for (const auto mode : modes) {
    for (const auto direction : directions) {

      const auto &systemPF =
          initialClosed(direction) ? systemPFClosed : systemPFOpen;

      results.push_back(runEMT(p, systemPF, mode, direction));
    }
  }

  for (const auto mode : modes) {
    for (const auto direction : directions) {

      const auto &systemPF =
          initialClosed(direction) ? systemPFClosed : systemPFOpen;

      results.push_back(runDP(p, systemPF, mode, direction));
    }
  }

  writeSummary(resultsDir, results);

  UInt passed = 0;

  for (const auto &r : results) {
    if (r.passOverall)
      ++passed;

    std::cout << r.domain << " " << r.mode << " " << r.direction << ": "
              << (r.passOverall ? "PASS" : "FAIL") << "\n";
  }

  std::cout << "Regression: " << passed << "/" << results.size()
            << " passed. Results: " << resultsDir << std::endl;

  return passed == results.size() ? 0 : 1;
}
