/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/DP/DP_Ph1_DPDQInterface.h>

using namespace CPS;

void DP::Ph1::DPDQInterface::setDPShiftFrequency(const Real & omegaShift) {
	mOmegaShift = omegaShift;
}

void DP::Ph1::DPDQInterface::updateDQToDPTransform(const Real & thetaDQ, const Real & simTime) {
	mDQToDPTransform << 	cos(thetaDQ - mOmegaShift * simTime),	-sin(thetaDQ - mOmegaShift * simTime),
							sin(thetaDQ - mOmegaShift * simTime),	cos(thetaDQ - mOmegaShift * simTime);
}

void DP::Ph1::DPDQInterface::updateDPToDQTransform(const Real & thetaDQ, const Real & simTime) {
	mDPToDQTransform << 	cos(thetaDQ - mOmegaShift * simTime),	sin(thetaDQ - mOmegaShift * simTime),
							-sin(thetaDQ - mOmegaShift * simTime),	cos(thetaDQ - mOmegaShift * simTime);
}

Complex DP::Ph1::DPDQInterface::applyDQToDPTransform(const MatrixFixedSize<2,1> & dqMatrix) {
	Complex dpComplex;
	dpComplex = Complex((mDQToDPTransform*dqMatrix)(0,0),(mDQToDPTransform*dqMatrix)(1,0));
	return dpComplex;
}

MatrixFixedSize<2,1> DP::Ph1::DPDQInterface::applyDPToDQTransform(const Complex& dpComplex) {
	MatrixFixedSize<2,1> dqMatrix;
	dqMatrix(0,0) = mDPToDQTransform(0,0) * dpComplex.real() + mDPToDQTransform(0,1) * dpComplex.imag();
	dqMatrix(1,0) = mDPToDQTransform(1,0) * dpComplex.real() + mDPToDQTransform(1,1) * dpComplex.imag();
	return dqMatrix;
}
