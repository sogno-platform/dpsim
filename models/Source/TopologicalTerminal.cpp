/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <cps/TopologicalTerminal.h>

using namespace CPS;

TopologicalTerminal::TopologicalTerminal(String uid, String name, PhaseType phase)
	: IdentifiedObject(uid, name) {
	setPhaseType(phase);
}

Complex TopologicalTerminal::singlePower() {
	if (mPhaseType == PhaseType::B)
		return mPower(1,0);
	else if (mPhaseType == PhaseType::C)
		return mPower(2,0);
	else // mPhaseType == PhaseType::Single || mPhaseType == PhaseType::A
		return mPower(0,0);
}

MatrixComp TopologicalTerminal::initialVoltage() {
	if (mPhaseType == PhaseType::Single || mPhaseType == PhaseType::A)
		return topologicalNodes()->initialVoltage().block(0,0,1,1);
	else if (mPhaseType == PhaseType::B)
		return topologicalNodes()->initialVoltage().block(1,0,1,1);
	else if (mPhaseType == PhaseType::C)
		return topologicalNodes()->initialVoltage().block(2,0,1,1);
	else
		return topologicalNodes()->initialVoltage();
}

