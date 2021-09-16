/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <cps/SP/SP_Ph1_VDNode.h>

using namespace CPS;


SP::Ph1::VDNode::VDNode(String uid, String name,
	Logger::Level logLevel) : SimPowerComp<Complex>(uid, name, logLevel) {

	addAttribute<Real>("Delta_set", &mDeltaSetPoint, Flags::read | Flags::write);
	addAttribute<Real>("V_set_pu", &mVoltageSetPointPerUnit, Flags::read | Flags::write);

}

SP::Ph1::VDNode::VDNode(String uid, String name, Real vSetPointPerUnit,
	Logger::Level logLevel) :VDNode(uid, name, logLevel) {

	mVoltageSetPointPerUnit = vSetPointPerUnit;
	mDeltaSetPoint = 0.;
}

	SP::Ph1::VDNode::VDNode(String uid, String name, Real vSetPointPerUnit, Real delta,
	Logger::Level logLevel):VDNode(uid,name,logLevel){

	mVoltageSetPointPerUnit = vSetPointPerUnit;
	mDeltaSetPoint = delta;
}
