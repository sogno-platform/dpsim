/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/
#include <dpsim-models/SP/SP_Ph1_VoltageSourceInverter.h>

using namespace CPS;

SP::Ph1::VoltageSourceInverter::VoltageSourceInverter(
    String uid, String name, PowerflowBusType powerflowBusType,
    Logger::Level logLevel)
    : SimPowerComp<Complex>(uid, name, logLevel) {
  mNumTerminals = 1;
  mTerminals.resize(mNumTerminals, nullptr);

  mPowerflowBusType = powerflowBusType;

  // mLog.debug() << "Create " << name << " of type " << this->type() << std::endl;
}

SP::Ph1::VoltageSourceInverter::VoltageSourceInverter(
    String uid, String name, Real power, Real reactivePower,
    PowerflowBusType powerflowBusType, Logger::Level logLevel)
    : VoltageSourceInverter(uid, name, powerflowBusType, logLevel) {
  mPowerflowBusType = powerflowBusType;

  switch (powerflowBusType) {
  case CPS::PowerflowBusType::PQ:
    mPQ = std::make_shared<PQNode>(**mUID, **mName, power, reactivePower,
                                   mLogLevel);
    break;
  default:
    // mLog.debug() << " Power flow bus type other than PQ for inverter were not implemented. " << std::endl;
    break;
  }
};

void SP::Ph1::VoltageSourceInverter::modifyPowerFlowBusType(
    PowerflowBusType powerflowBusType) {

  mPowerflowBusType = powerflowBusType;
  switch (powerflowBusType) {
  case CPS::PowerflowBusType::PV:
    throw std::invalid_argument(
        " inverters currently cannot be set as PV bus.");
    break;
  case CPS::PowerflowBusType::PQ:
    mPQ = std::make_shared<CPS::SP::Ph1::PQNode>(**mUID, **mName, mLogLevel);
    break;
  case CPS::PowerflowBusType::VD:
    throw std::invalid_argument(
        " inverters currently cannot be set as VD bus. ");
    break;
  case CPS::PowerflowBusType::None:
    break;
  default:
    throw std::invalid_argument(" Invalid power flow bus type ");
    break;
  }
}
