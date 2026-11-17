// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <dpsim-models/SP/SP_Ph1_SVC.h>

using namespace CPS;

SP::Ph1::SVC::SVC(String uid, String name, Logger::Level logLevel)
    : SimPowerComp<Complex>(uid, name, logLevel),
      mSetPointVoltage(mAttributes->create<Real>("V_set")),
      mSetPointVoltagePerUnit(mAttributes->create<Real>("V_set_pu")),
      mSetPointReactivePower(mAttributes->create<Real>("Q_set")),
      mSetPointReactivePowerPerUnit(mAttributes->create<Real>("Q_set_pu")),
      mReactivePowerMax(mAttributes->create<Real>(
          "Q_max", std::numeric_limits<Real>::infinity())),
      mReactivePowerMin(mAttributes->create<Real>(
          "Q_min", -std::numeric_limits<Real>::infinity())),
      mReactivePowerMaxPerUnit(mAttributes->create<Real>(
          "Q_max_pu", std::numeric_limits<Real>::infinity())),
      mReactivePowerMinPerUnit(mAttributes->create<Real>(
          "Q_min_pu", -std::numeric_limits<Real>::infinity())) {

  SPDLOG_LOGGER_INFO(mSLog, "Create {} of type {}", name, this->type());
  mSLog->flush();

  setTerminalNumber(1);
};

void SP::Ph1::SVC::setParameters(Real ratedApparentPower, Real ratedVoltage,
                                 Real setPointVoltage, Real qLimMax,
                                 Real qLimMin) {
  **mSetPointVoltage = setPointVoltage;
  **mReactivePowerMax = qLimMax;
  **mReactivePowerMin = qLimMin;
  // Always a PQ bus; the outer loop moves Q_set within the band to hold V_set
  mPowerflowBusType = PowerflowBusType::PQ;

  SPDLOG_LOGGER_INFO(mSLog, "Rated Apparent Power={} [VA] Rated Voltage={} [V]",
                     ratedApparentPower, ratedVoltage);
  SPDLOG_LOGGER_INFO(mSLog, "Voltage Set Point={} [V]", **mSetPointVoltage);
  SPDLOG_LOGGER_INFO(mSLog, "Reactive Power Limits Qmin={} Qmax={} [VAr]",
                     **mReactivePowerMin, **mReactivePowerMax);
  mSLog->flush();
}

// #### Powerflow section ####

Real SP::Ph1::SVC::getBaseVoltage() const { return mBaseVoltage; }

void SP::Ph1::SVC::setBaseVoltage(Real baseVoltage) {
  mBaseVoltage = baseVoltage;
}

void SP::Ph1::SVC::calculatePerUnitParameters(Real baseApparentPower,
                                              Real baseOmega) {
  SPDLOG_LOGGER_INFO(mSLog, "#### Calculate Per Unit Parameters for {}",
                     **mName);
  mBaseApparentPower = baseApparentPower;
  SPDLOG_LOGGER_INFO(mSLog, "Base Power={} [VA]  Base Omega={} [1/s]",
                     mBaseApparentPower, baseOmega);

  // Reject unset/zero divisors before the per-unit conversion below.
  if (!Math::isFinite(mBaseVoltage) || mBaseVoltage <= 0)
    throw std::invalid_argument(
        "SVC: base voltage not set (call setBaseVoltage() before power-flow "
        "initialization).");
  if (!Math::isFinite(mBaseApparentPower) || mBaseApparentPower <= 0)
    throw std::invalid_argument(
        "SVC: base apparent power must be finite and positive.");

  **mSetPointVoltagePerUnit = **mSetPointVoltage / mBaseVoltage;
  **mSetPointReactivePowerPerUnit =
      **mSetPointReactivePower / mBaseApparentPower;
  // +/-inf limits stay +/-inf in pu; finite limits scale
  **mReactivePowerMaxPerUnit = **mReactivePowerMax / mBaseApparentPower;
  **mReactivePowerMinPerUnit = **mReactivePowerMin / mBaseApparentPower;
  SPDLOG_LOGGER_INFO(mSLog, "Voltage Set Point={} [pu]",
                     **mSetPointVoltagePerUnit);
  mSLog->flush();
}

void SP::Ph1::SVC::modifyPowerFlowBusType(PowerflowBusType powerflowBusType) {
  switch (powerflowBusType) {
  case CPS::PowerflowBusType::PQ:
    mPowerflowBusType = powerflowBusType;
    break;
  case CPS::PowerflowBusType::None:
    break;
  case CPS::PowerflowBusType::PV:
  case CPS::PowerflowBusType::VD:
    throw std::invalid_argument("SVC: only PQ is a valid power flow bus type.");
  default:
    throw std::invalid_argument("Invalid power flow bus type");
    break;
  }
}

// Called by the SVC outer control loop
void SP::Ph1::SVC::updateReactivePowerInjection(Complex powerInj) {
  if (!Math::isFinite(mBaseApparentPower) || mBaseApparentPower <= 0)
    throw std::invalid_argument(
        "SVC: base apparent power must be finite and positive (call "
        "calculatePerUnitParameters() before updating the injection).");
  if (!Math::isFinite(powerInj.imag()))
    throw std::invalid_argument(
        "SVC: reactive power injection must be finite.");

  **mSetPointReactivePower = powerInj.imag();
  **mSetPointReactivePowerPerUnit =
      **mSetPointReactivePower / mBaseApparentPower;
}

Complex SP::Ph1::SVC::getApparentPower() const {
  return Complex(0., **mSetPointReactivePower);
}
