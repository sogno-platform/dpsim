/* Copyright 2017-2020 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <cps/DP/DP_Ph1_TriggeredSwitch.h>

using namespace CPS;

DP::Ph1::TriggeredSwitch::TriggeredSwitch(String uid, String name, Logger::Level logLevel)
	: varResSwitch(uid, name, logLevel) { 

	addAttribute<Real>("trigger_signal", Flags::read | Flags::write);
	addAttribute<Real>("closed_duration", &mSwitchClosedDuration, Flags::read | Flags::write);
}

void DP::Ph1::TriggeredSwitch::mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
	DP::Ph1::varResSwitch::mnaInitialize(omega, timeStep, leftVector);
	mMnaTasks.push_back(std::make_shared<MnaPreStep>(*this));
}

void DP::Ph1::TriggeredSwitch::mnaAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) {
	// add pre-step dependencies of component itself
	prevStepDependencies.push_back(attribute("i_intf"));
	prevStepDependencies.push_back(attribute("v_intf"));
	attributeDependencies.push_back(attribute("trigger_signal"));
	modifiedAttributes.push_back(attribute("right_vector"));
}

void DP::Ph1::TriggeredSwitch::mnaPreStep(Real time, Int timeStepCount) {
	// Determine whether trigger threshold currently exceeded
	mExceedsThrehold = attribute<Real>("trigger_signal")->get() > mTriggerThreshold;

	// Trigger switch closing if exceeding threshold with rising edge and switch currently open
    if (!mExceedsThreholdPrev && mExceedsThrehold && !attribute<Bool>("is_closed")->get()) {
		attribute<Bool>("is_closed")->set(true);
		mSwitchClosedStartTime = time;
	}

	// Trigger switch opening if switch closed duration over and switch currently closed
	if (time > mSwitchClosedStartTime + attribute<Real>("closed_duration")->get() && attribute<Bool>("is_closed")->get())
		attribute<Bool>("is_closed")->set(false);

	// Remember current state for next time step
    mExceedsThreholdPrev = mExceedsThrehold;
}

