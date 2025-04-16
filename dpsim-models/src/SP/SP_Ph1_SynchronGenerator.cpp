/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/SP/SP_Ph1_SynchronGenerator.h>

using namespace CPS;

SP::Ph1::SynchronGenerator::SynchronGenerator(String uid, String name,
                                              Logger::Level logLevel)
    : SimPowerComp<Complex>(uid, name, logLevel),
      mSetPointActivePower(mAttributes->create<Real>("P_set")),
      mSetPointReactivePower(mAttributes->create<Real>("Q_set")),
      mSetPointVoltage(mAttributes->create<Real>("V_set")),
      mSetPointActivePowerPerUnit(mAttributes->create<Real>("P_set_pu")),
      mSetPointReactivePowerPerUnit(mAttributes->create<Real>("Q_set_pu")),
      mSetPointVoltagePerUnit(mAttributes->create<Real>("V_set_pu")) {

  SPDLOG_LOGGER_INFO(mSLog, "Create {} of type {}", name, this->type());
  mSLog->flush();

  setTerminalNumber(1);
};

void SP::Ph1::SynchronGenerator::setParameters(
    Real ratedApparentPower, Real ratedVoltage, Real setPointActivePower,
    Real setPointVoltage, PowerflowBusType powerflowBusType,
    Real setPointReactivePower) {
  **mSetPointActivePower = setPointActivePower;
  **mSetPointReactivePower = setPointReactivePower;
  **mSetPointVoltage = setPointVoltage;
  mPowerflowBusType = powerflowBusType;

  SPDLOG_LOGGER_INFO(mSLog, "Rated Apparent Power={} [VA] Rated Voltage={} [V]",
                     ratedApparentPower, ratedVoltage);
  SPDLOG_LOGGER_INFO(mSLog,
                     "Active Power Set Point={} [W] Voltage Set Point={} [V]",
                     **mSetPointActivePower, **mSetPointVoltage);
  mSLog->flush();
}

// #### Powerflow section ####

Real SP::Ph1::SynchronGenerator::getBaseVoltage() const { return mBaseVoltage; }

void SP::Ph1::SynchronGenerator::setBaseVoltage(Real baseVoltage) {
  mBaseVoltage = baseVoltage;
}

void SP::Ph1::SynchronGenerator::calculatePerUnitParameters(
    Real baseApparentPower, Real baseOmega) {
  SPDLOG_LOGGER_INFO(mSLog, "#### Calculate Per Unit Parameters for {}",
                     **mName);
  mBaseApparentPower = baseApparentPower;
  SPDLOG_LOGGER_INFO(mSLog, "Base Power={} [VA]  Base Omega={} [1/s]",
                     mBaseApparentPower, baseOmega);

  **mSetPointActivePowerPerUnit = **mSetPointActivePower / mBaseApparentPower;
  **mSetPointReactivePowerPerUnit =
      **mSetPointReactivePower / mBaseApparentPower;
  **mSetPointVoltagePerUnit = **mSetPointVoltage / mBaseVoltage;
  SPDLOG_LOGGER_INFO(mSLog,
                     "Active Power Set Point={} [pu] Voltage Set Point={} [pu]",
                     **mSetPointActivePowerPerUnit, **mSetPointVoltagePerUnit);
  mSLog->flush();
}

void SP::Ph1::SynchronGenerator::modifyPowerFlowBusType(
    PowerflowBusType powerflowBusType) {
  switch (powerflowBusType) {
  case CPS::PowerflowBusType::PV:
    mPowerflowBusType = powerflowBusType;
    break;
  case CPS::PowerflowBusType::PQ:
    throw std::invalid_argument(
        "Setting Synchronous Generator as PQNode is currently not supported.");
    break;
  case CPS::PowerflowBusType::VD:
    mPowerflowBusType = powerflowBusType;
    break;
  case CPS::PowerflowBusType::None:
    break;
  default:
    throw std::invalid_argument(" Invalid power flow bus type ");
    break;
  }
}

//Method used in PF to update reactive power for PV-Bus generator
void SP::Ph1::SynchronGenerator::updateReactivePowerInjection(
    Complex powerInj) {
  **mSetPointReactivePower = powerInj.imag();
  **mSetPointReactivePowerPerUnit =
      **mSetPointReactivePower / mBaseApparentPower;
}

//Method used in PF to update active & reactive power for VD-Bus generator
void SP::Ph1::SynchronGenerator::updatePowerInjection(Complex powerInj) {
  **mSetPointActivePower = powerInj.real();
  **mSetPointActivePowerPerUnit = **mSetPointActivePower / mBaseApparentPower;
  **mSetPointReactivePower = powerInj.imag();
  **mSetPointReactivePowerPerUnit =
      **mSetPointReactivePower / mBaseApparentPower;
}

Complex SP::Ph1::SynchronGenerator::getApparentPower() const {
  return Complex(**mSetPointActivePower, **mSetPointReactivePower);
}
