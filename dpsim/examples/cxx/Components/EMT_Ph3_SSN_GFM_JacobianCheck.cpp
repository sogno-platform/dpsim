// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include <DPsim.h>
#include <dpsim-models/EMT/EMT_Ph3_SSN_GFM.h>

using namespace DPsim;
using namespace CPS;

namespace {

const String simName = "EMT_Ph3_SSN_GFM_JacobianCheck";

struct Deviation {
  Real a = 0.0;
  Real b = 0.0;
  Real c = 0.0;
  Real d = 0.0;

  Real worst() const { return std::max(std::max(a, b), std::max(c, d)); }
};

Real columnwiseDeviation(const Matrix &analytical, const Matrix &numerical) {
  Real worst = 0.0;

  for (Int column = 0; column < analytical.cols(); ++column) {
    const Real reference =
        std::max(analytical.col(column).cwiseAbs().maxCoeff(),
                 numerical.col(column).cwiseAbs().maxCoeff());

    if (reference < 1e-12)
      continue;

    worst = std::max(
        worst,
        (analytical.col(column) - numerical.col(column)).cwiseAbs().maxCoeff() /
            reference);
  }

  return worst;
}

Matrix balancedAbc(Real peak, Real angle) {
  Matrix abc(3, 1);
  abc << peak * std::cos(angle), peak * std::cos(angle - 2.0 * PI / 3.0),
      peak * std::cos(angle + 2.0 * PI / 3.0);
  return abc;
}

UInt stateIndex(const std::vector<String> &names, const String &name) {
  const auto position = std::find(names.begin(), names.end(), name);

  if (position == names.end())
    throw std::invalid_argument("Unknown SSN_GFM state name " + name);

  return static_cast<UInt>(std::distance(names.begin(), position));
}

std::shared_ptr<EMT::Ph3::SSN_GFM> makeInverter(const String &name) {
  const Real phaseVoltageRms = 220.0;
  const Real phaseVoltagePeak = std::sqrt(2.0) * phaseVoltageRms;

  const Real nominalOmega = 2.0 * PI * 50.0;

  const Real filterInductance = 3e-3;
  const Real filterCapacitance = 20e-6;
  const Real filterResistance = 0.05;
  const Real couplingResistance = 0.05;

  const Real voltageLoopBandwidth = 200.0;
  const Real currentLoopBandwidth = 1683.0;
  const Real voltageLoopDamping = 1.0 / std::sqrt(2.0);

  auto inverter = EMT::Ph3::SSN_GFM::make(name, Logger::Level::off);

  inverter->setParameters(
      filterInductance, filterCapacitance, filterResistance, couplingResistance,
      phaseVoltagePeak, nominalOmega, 5e3, 1e3, 0.2, 25.0, 1.0 / 15.0,
      300.0 / 1000.0,
      2.0 * voltageLoopDamping * voltageLoopBandwidth * filterCapacitance,
      voltageLoopBandwidth * voltageLoopBandwidth * filterCapacitance,
      currentLoopBandwidth * filterInductance,
      currentLoopBandwidth * filterResistance, 15.0, 100.0,
      2.0 * PI * 20e3 / 1.5);

  return inverter;
}

Matrix operatingPointState(const std::vector<String> &names, Real theta) {
  const Real phaseVoltagePeak = std::sqrt(2.0) * 220.0;

  Matrix x = Matrix::Zero(static_cast<Int>(names.size()), 1);

  x(stateIndex(names, "p_filtered"), 0) = 4.7e3;
  x(stateIndex(names, "q_filtered"), 0) = 0.9e3;
  x(stateIndex(names, "omega"), 0) = 2.0 * PI * 50.0 * 1.0007;
  x(stateIndex(names, "theta"), 0) = theta;
  x(stateIndex(names, "voltage_magnitude"), 0) = 1.01 * phaseVoltagePeak;

  x(stateIndex(names, "voltage_integrator_d"), 0) = 2.3;
  x(stateIndex(names, "voltage_integrator_q"), 0) = -1.1;

  x(stateIndex(names, "current_integrator_d"), 0) = 0.7;
  x(stateIndex(names, "current_integrator_q"), 0) = -0.4;

  x(stateIndex(names, "delay_voltage_d"), 0) = 1.02 * phaseVoltagePeak;
  x(stateIndex(names, "delay_voltage_q"), 0) = -6.5;

  x.block(stateIndex(names, "vc_a"), 0, 3, 1) =
      balancedAbc(phaseVoltagePeak, theta);
  x.block(stateIndex(names, "if_a"), 0, 3, 1) = balancedAbc(14.3, theta - 0.21);

  return x;
}

Matrix operatingPointInput(Real theta) {
  return balancedAbc(0.985 * std::sqrt(2.0) * 220.0, theta - 0.06);
}

Deviation
checkOperatingPoint(const std::shared_ptr<EMT::Ph3::SSN_GFM> &inverter,
                    Real theta) {
  const Matrix x = operatingPointState(inverter->getLocalStateNames(), theta);
  const Matrix u = operatingPointInput(theta);

  Matrix analyticalA;
  Matrix analyticalB;
  Matrix analyticalC;
  Matrix analyticalD;

  Matrix numericalA;
  Matrix numericalB;
  Matrix numericalC;
  Matrix numericalD;

  inverter->calculateAnalyticalJacobians(x, u, analyticalA, analyticalB,
                                         analyticalC, analyticalD);
  inverter->calculateNumericalJacobians(x, u, numericalA, numericalB,
                                        numericalC, numericalD);

  return {columnwiseDeviation(analyticalA, numericalA),
          columnwiseDeviation(analyticalB, numericalB),
          columnwiseDeviation(analyticalC, numericalC),
          columnwiseDeviation(analyticalD, numericalD)};
}

} // namespace

int main() {
  Logger::setLogDir("logs/" + simName);
  auto log = Logger::get(simName, Logger::Level::info, Logger::Level::info);

  const Real tolerance = 1e-5;

  auto islanded = makeInverter("islanded");

  auto gridConnected = makeInverter("grid_connected");
  gridConnected->setGridCurrentFeedforward(0.0);
  gridConnected->setVirtualImpedance(0.8, 0.3);
  gridConnected->setReactivePowerDroop(1e-4, 30.0);

  const std::vector<std::pair<String, std::shared_ptr<EMT::Ph3::SSN_GFM>>>
      configurations = {{"integral excitation", islanded},
                        {"Q-V droop, virtual impedance", gridConnected}};

  Bool passed = true;

  for (const auto &configuration : configurations) {
    for (const Real theta : {0.37, 1.94, 4.71}) {
      const Deviation deviation =
          checkOperatingPoint(configuration.second, theta);

      log->info("{}, theta = {:.2f} rad: A {:.3e}, B {:.3e}, C {:.3e}, "
                "D {:.3e}",
                configuration.first, theta, deviation.a, deviation.b,
                deviation.c, deviation.d);

      if (!(deviation.worst() < tolerance)) {
        log->error("Analytical and numerical Jacobians disagree by {:.3e}, "
                   "tolerance is {:.3e}",
                   deviation.worst(), tolerance);
        passed = false;
      }
    }
  }

  for (const Real elapsedTime : {0.0, 2.0, 8.0}) {
    const Real theta = 2.0 * PI * 60.0 * elapsedTime + 0.37;
    const Deviation deviation = checkOperatingPoint(islanded, theta);

    log->info("finite-difference error after {:.1f} s of angle growth "
              "(theta = {:.1f} rad): A {:.3e}",
              elapsedTime, theta, deviation.a);
  }

  if (!passed) {
    log->error("Jacobian check failed.");
    return 1;
  }

  log->info("Jacobian check passed.");
  return 0;
}
