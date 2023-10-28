/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/TopologicalNode.h>

using namespace CPS;

Bool TopologicalNode::isGround() const { return mIsGround; }

MatrixComp TopologicalNode::initialVoltage() const { return **mInitialVoltage; }

void TopologicalNode::setInitialVoltage(MatrixComp voltage) const { **mInitialVoltage = voltage; }

void TopologicalNode::setInitialVoltage(Complex voltage) const {
	if (mPhaseType == PhaseType::Single) {
		(**mInitialVoltage)(0,0) = voltage;
	} else {
		(**mInitialVoltage)(0,0) = voltage;
		(**mInitialVoltage)(1,0) = SHIFT_TO_PHASE_B * voltage;
		(**mInitialVoltage)(2,0) = SHIFT_TO_PHASE_C * voltage;
	}
}

void TopologicalNode::setInitialVoltage(Complex voltage, Int phaseIndex) const {
	(**mInitialVoltage)(phaseIndex, 0) = voltage;
}

PhaseType TopologicalNode::phaseType() const { return mPhaseType; }

TopologicalNode::TopologicalNode(String uid, String name,
	PhaseType phaseType, const std::vector<Complex> &initialVoltage)
	: IdentifiedObject(uid, name),
		mInitialVoltage(mAttributes->create<MatrixComp>("voltage_init")) {

	mPhaseType = phaseType;
	if (phaseType == PhaseType::ABC) {
		//mMatrixNodeIndex = matrixNodeIndex;
		**mInitialVoltage = MatrixComp::Zero(3, 1);
		**mInitialVoltage << initialVoltage[0], initialVoltage[1], initialVoltage[2];
	}
	else {
		//mMatrixNodeIndex = { matrixNodeIndex[0] };
		**mInitialVoltage = MatrixComp::Zero(1, 1);
		**mInitialVoltage << initialVoltage[0];
	}
}

Complex TopologicalNode::initialSingleVoltage(PhaseType phaseType) {
	if (phaseType == PhaseType::B)
		return (**mInitialVoltage)(1,0);
	else if (phaseType == PhaseType::C)
		return (**mInitialVoltage)(2,0);
	else // phaseType == PhaseType::Single || mPhaseType == PhaseType::A
		return (**mInitialVoltage)(0,0);
}


