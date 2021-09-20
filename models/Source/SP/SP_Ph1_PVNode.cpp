/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <cps/SP/SP_Ph1_PVNode.h>

using namespace CPS;


SP::Ph1::PVNode::PVNode(String uid, String name,
	Logger::Level logLevel) : SimPowerComp<Complex>(uid, name, logLevel) {

	addAttribute<Real>("P_set", &mPowerSetPoint, Flags::read | Flags::write);
	addAttribute<Real>("V_set", &mVoltageSetPoint, Flags::read | Flags::write);
	addAttribute<Real>("V_set_pu", &mVoltagePerUnit, Flags::read | Flags::write);

}


SP::Ph1::PVNode::PVNode(String uid, String name, Real power, Real vSetPoint, Real maxQ, Real ratedU, Real ratedS,
	Logger::Level logLevel)
	:PVNode(uid, name, logLevel) {

	mPowerSetPoint =  power;
	mVoltageSetPoint = vSetPoint;
	mRatedU = ratedU;

	mVoltagePerUnit = vSetPoint / ratedU;

	// maxQ=0 means no limit.
	mQLimit = maxQ;

	mSLog->info("Create PV node for {} P={}, V={}", name, mPowerSetPoint, mVoltageSetPoint);
}



