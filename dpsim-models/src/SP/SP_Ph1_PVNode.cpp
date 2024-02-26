/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/SP/SP_Ph1_PVNode.h>

using namespace CPS;

SP::Ph1::PVNode::PVNode(String uid, String name, Logger::Level logLevel)
    : SimPowerComp<Complex>(uid, name, logLevel),
      mPowerSetPoint(mAttributes->create<Real>("P_set")),
      mVoltageSetPoint(mAttributes->create<Real>("V_set")),
      mVoltagePerUnit(mAttributes->create<Real>("V_set_pu")) {}

SP::Ph1::PVNode::PVNode(String uid, String name, Real power, Real vSetPoint,
                        Real maxQ, Real ratedU, Real ratedS,
                        Logger::Level logLevel)
    : PVNode(uid, name, logLevel) {

  **mPowerSetPoint = power;
  **mVoltageSetPoint = vSetPoint;

  **mVoltagePerUnit = vSetPoint / ratedU;

  SPDLOG_LOGGER_INFO(mSLog, "Create PV node for {} P={}, V={}", name,
                     **mPowerSetPoint, **mVoltageSetPoint);
}
