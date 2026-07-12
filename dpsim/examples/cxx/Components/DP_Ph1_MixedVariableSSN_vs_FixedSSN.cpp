// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

// Plumbing sanity check for DP::Ph1::MixedVTypeVariableSSNComp: a trivial
// variable one-port (Variable_Serial_RLC, held at constant R/L/C) run
// side by side with the fixed-parameter reference of the same circuit
// (Full_Serial_RLC, built on DP::SSNComp) and compared directly.

#include <DPsim.h>
#include <dpsim-models/DP/DP_Ph1_SSN_Variable_Serial_RLC.h>

using namespace DPsim;
using namespace CPS::DP;

namespace {

Real timeStep = 0.0001;
Real finalTime = 0.1;
Real resistance = 10.0;
Real inductance = 0.01;
Real capacitance = 0.002;
Real systemFreq = 50.0;

SimNode::Ptr buildAndRun(const String &simName,
                         CPS::SimPowerComp<Complex>::Ptr rlc,
                         Bool systemMatrixRecomputation) {
  const Complex sourceVoltage =
      CPS::Math::polar(1.0, CPS::Math::degToRad(-90.0));

  auto n1 = SimNode::make("n1");
  n1->setInitialVoltage(sourceVoltage);

  auto vs = Ph1::VoltageSource::make("vs_" + simName);
  vs->setParameters(sourceVoltage);

  vs->connect(SimNode::List{SimNode::GND, n1});
  rlc->connect(SimNode::List{n1, SimNode::GND});

  auto sys = SystemTopology(systemFreq, SystemNodeList{n1},
                            SystemComponentList{vs, rlc});

  Logger::setLogDir("logs/" + simName);
  auto logger = DataLogger::make(simName);
  logger->logAttribute("v_rlc", rlc->attribute("v_intf"));
  logger->logAttribute("i_rlc", rlc->attribute("i_intf"));

  Simulation sim(simName, Logger::Level::info);
  sim.setSystem(sys);
  sim.addLogger(logger);
  sim.setDomain(Domain::DP);
  sim.setTimeStep(timeStep);
  sim.setFinalTime(finalTime);
  sim.doSystemMatrixRecomputation(systemMatrixRecomputation);
  sim.run();

  return n1;
}

} // namespace

int main(int argc, char *argv[]) {
  auto rlcFixed = Ph1::SSN::Full_Serial_RLC::make("rlc_fixed");
  rlcFixed->setParameters(resistance, inductance, capacitance);
  buildAndRun("DP_Ph1_MixedVariableSSN_vs_FixedSSN_Fixed", rlcFixed, false);

  auto rlcVariable = Ph1::SSN::Variable_Serial_RLC::make("rlc_variable");
  rlcVariable->setParameters(resistance, inductance, capacitance,
                             2.0 * PI * systemFreq);
  buildAndRun("DP_Ph1_MixedVariableSSN_vs_FixedSSN_Variable", rlcVariable,
              true);

  const Complex iFixed =
      rlcFixed->attributeTyped<MatrixComp>("i_intf")->get()(0, 0);
  const Complex iVariable =
      rlcVariable->attributeTyped<MatrixComp>("i_intf")->get()(0, 0);
  const Complex vFixed =
      rlcFixed->attributeTyped<MatrixComp>("v_intf")->get()(0, 0);
  const Complex vVariable =
      rlcVariable->attributeTyped<MatrixComp>("v_intf")->get()(0, 0);

  const Real iErr = std::abs(iFixed - iVariable);
  const Real vErr = std::abs(vFixed - vVariable);
  const Real tol = 1e-6;

  auto log = DPsim::Logger::get("DP_Ph1_MixedVariableSSN_vs_FixedSSN");
  SPDLOG_LOGGER_INFO(log, "Final i_intf: fixed={} variable={} |diff|={}",
                     iFixed, iVariable, iErr);
  SPDLOG_LOGGER_INFO(log, "Final v_intf: fixed={} variable={} |diff|={}",
                     vFixed, vVariable, vErr);

  if (iErr > tol || vErr > tol) {
    SPDLOG_LOGGER_ERROR(log,
                        "FAIL: Variable-SSN one-port does not match the "
                        "fixed-SSN reference within tolerance {}",
                        tol);
    return 1;
  }

  SPDLOG_LOGGER_INFO(
      log,
      "PASS: Variable-SSN one-port matches the fixed-SSN reference within "
      "tolerance {}",
      tol);
  return 0;
}
