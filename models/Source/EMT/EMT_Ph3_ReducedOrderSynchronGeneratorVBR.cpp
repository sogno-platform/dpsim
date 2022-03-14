/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <cps/EMT/EMT_Ph3_ReducedOrderSynchronGeneratorVBR.h>

using namespace CPS;

EMT::Ph3::ReducedOrderSynchronGeneratorVBR::ReducedOrderSynchronGeneratorVBR
    (String uid, String name, Logger::Level logLevel)
	: Base::ReducedOrderSynchronGenerator<Real>(uid, name, logLevel){
	
	mPhaseType = PhaseType::ABC;
	setVirtualNodeNumber(2);
	setTerminalNumber(1);
	
	// model variable
	mIntfVoltage = Matrix::Zero(3, 1);
	mIntfCurrent = Matrix::Zero(3, 1);
	mEvbr = Matrix::Zero(3,1);

    // Register attributes
    addAttribute<Matrix>("Evbr",  &mEvbr, Flags::read);
}

EMT::Ph3::ReducedOrderSynchronGeneratorVBR::ReducedOrderSynchronGeneratorVBR
	(String name, Logger::Level logLevel)
	: ReducedOrderSynchronGeneratorVBR(name, name, logLevel) {
}

EMT::Ph3::ReducedOrderSynchronGeneratorVBR::~ReducedOrderSynchronGeneratorVBR() {
}

void EMT::Ph3::ReducedOrderSynchronGeneratorVBR::calculateResistanceMatrix() {
	Matrix resistanceMatrix =  mDq0ToAbc * mResistanceMatrixDq0 * mAbcToDq0;
	resistanceMatrix = resistanceMatrix * mBase_Z;
	mConductanceMatrix = resistanceMatrix.inverse();
}

void EMT::Ph3::ReducedOrderSynchronGeneratorVBR::mnaApplySystemMatrixStamp(Matrix& systemMatrix) {

	// Stamp voltage source
	Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::A), mVirtualNodes[1]->matrixNodeIndex(PhaseType::A), -1);
	Math::addToMatrixElement(systemMatrix, mVirtualNodes[1]->matrixNodeIndex(PhaseType::A), mVirtualNodes[0]->matrixNodeIndex(PhaseType::A), 1);
	Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::B), mVirtualNodes[1]->matrixNodeIndex(PhaseType::B), -1);
	Math::addToMatrixElement(systemMatrix, mVirtualNodes[1]->matrixNodeIndex(PhaseType::B), mVirtualNodes[0]->matrixNodeIndex(PhaseType::B), 1);
	Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::C), mVirtualNodes[1]->matrixNodeIndex(PhaseType::C), -1);
	Math::addToMatrixElement(systemMatrix, mVirtualNodes[1]->matrixNodeIndex(PhaseType::C), mVirtualNodes[0]->matrixNodeIndex(PhaseType::C), 1);

	// Stamp conductance

	// set upper left block, 3x3 entries
	Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::A), mVirtualNodes[0]->matrixNodeIndex(PhaseType::A), mConductanceMatrix(0, 0));
	Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::A), mVirtualNodes[0]->matrixNodeIndex(PhaseType::B), mConductanceMatrix(0, 1));
	Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::A), mVirtualNodes[0]->matrixNodeIndex(PhaseType::C), mConductanceMatrix(0, 2));
	Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::B), mVirtualNodes[0]->matrixNodeIndex(PhaseType::A), mConductanceMatrix(1, 0));
	Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::B), mVirtualNodes[0]->matrixNodeIndex(PhaseType::B), mConductanceMatrix(1, 1));
	Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::B), mVirtualNodes[0]->matrixNodeIndex(PhaseType::C), mConductanceMatrix(1, 2));
	Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::C), mVirtualNodes[0]->matrixNodeIndex(PhaseType::A), mConductanceMatrix(2, 0));
	Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::C), mVirtualNodes[0]->matrixNodeIndex(PhaseType::B), mConductanceMatrix(2, 1));
	Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::C), mVirtualNodes[0]->matrixNodeIndex(PhaseType::C), mConductanceMatrix(2, 2));

	// set buttom right block, 3x3 entries
	Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 0), matrixNodeIndex(0, 0), mConductanceMatrix(0, 0));
	Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 0), matrixNodeIndex(0, 1), mConductanceMatrix(0, 1));
	Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 0), matrixNodeIndex(0, 2), mConductanceMatrix(0, 2));
	Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 1), matrixNodeIndex(0, 0), mConductanceMatrix(1, 0));
	Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 1), matrixNodeIndex(0, 1), mConductanceMatrix(1, 1));
	Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 1), matrixNodeIndex(0, 2), mConductanceMatrix(1, 2));
	Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 2), matrixNodeIndex(0, 0), mConductanceMatrix(2, 0));
	Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 2), matrixNodeIndex(0, 1), mConductanceMatrix(2, 1));
	Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 2), matrixNodeIndex(0, 2), mConductanceMatrix(2, 2));

	// Set off diagonal blocks
	Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::A), matrixNodeIndex(0, 0), -mConductanceMatrix(0, 0));
	Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::A), matrixNodeIndex(0, 1), -mConductanceMatrix(0, 1));
	Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::A), matrixNodeIndex(0, 2), -mConductanceMatrix(0, 2));
	Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::B), matrixNodeIndex(0, 0), -mConductanceMatrix(1, 0));
	Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::B), matrixNodeIndex(0, 1), -mConductanceMatrix(1, 1));
	Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::B), matrixNodeIndex(0, 2), -mConductanceMatrix(1, 2));
	Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::C), matrixNodeIndex(0, 0), -mConductanceMatrix(2, 0));
	Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::C), matrixNodeIndex(0, 1), -mConductanceMatrix(2, 1));
	Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::C), matrixNodeIndex(0, 2), -mConductanceMatrix(2, 2));

	Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 0), mVirtualNodes[0]->matrixNodeIndex(PhaseType::A), -mConductanceMatrix(0, 0));
	Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 0), mVirtualNodes[0]->matrixNodeIndex(PhaseType::B), -mConductanceMatrix(0, 1));
	Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 0), mVirtualNodes[0]->matrixNodeIndex(PhaseType::C), -mConductanceMatrix(0, 2));
	Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 1), mVirtualNodes[0]->matrixNodeIndex(PhaseType::A), -mConductanceMatrix(1, 0));
	Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 1), mVirtualNodes[0]->matrixNodeIndex(PhaseType::B), -mConductanceMatrix(1, 1));
	Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 1), mVirtualNodes[0]->matrixNodeIndex(PhaseType::C), -mConductanceMatrix(1, 2));
	Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 2), mVirtualNodes[0]->matrixNodeIndex(PhaseType::A), -mConductanceMatrix(2, 0));
	Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 2), mVirtualNodes[0]->matrixNodeIndex(PhaseType::B), -mConductanceMatrix(2, 1));
	Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 2), mVirtualNodes[0]->matrixNodeIndex(PhaseType::C), -mConductanceMatrix(2, 2));
}

void EMT::Ph3::ReducedOrderSynchronGeneratorVBR::mnaApplyRightSideVectorStamp(Matrix& rightVector) {
	Math::setVectorElement(rightVector, mVirtualNodes[1]->matrixNodeIndex(PhaseType::A), mEvbr(0, 0));
	Math::setVectorElement(rightVector, mVirtualNodes[1]->matrixNodeIndex(PhaseType::B), mEvbr(1, 0));
	Math::setVectorElement(rightVector, mVirtualNodes[1]->matrixNodeIndex(PhaseType::C), mEvbr(2, 0));
}

void EMT::Ph3::ReducedOrderSynchronGeneratorVBR::mnaPostStep(const Matrix& leftVector) {
	// update armature voltage
	mIntfVoltage(0, 0) = Math::realFromVectorElement(leftVector, matrixNodeIndex(0, 0));
	mIntfVoltage(1, 0) = Math::realFromVectorElement(leftVector, matrixNodeIndex(0, 1));
	mIntfVoltage(2, 0) = Math::realFromVectorElement(leftVector, matrixNodeIndex(0, 2));
	mVdq0 = mAbcToDq0 * mIntfVoltage / mBase_V;

	// update armature current
	mIntfCurrent(0, 0) = Math::realFromVectorElement(leftVector, mVirtualNodes[1]->matrixNodeIndex(PhaseType::A));
	mIntfCurrent(1, 0) = Math::realFromVectorElement(leftVector, mVirtualNodes[1]->matrixNodeIndex(PhaseType::B));
	mIntfCurrent(2, 0) = Math::realFromVectorElement(leftVector, mVirtualNodes[1]->matrixNodeIndex(PhaseType::C));
	mIdq0 =  mAbcToDq0 * mIntfCurrent / mBase_I;
}

Matrix EMT::Ph3::ReducedOrderSynchronGeneratorVBR::get_parkTransformMatrix() {
	Matrix abcToDq0(3, 3);

	abcToDq0 <<
		2./3.*cos(mThetaMech),  2./3.*cos(mThetaMech - 2.*PI/3.),  2./3.*cos(mThetaMech + 2.*PI/3.),
		-2./3.*sin(mThetaMech), -2./3.*sin(mThetaMech - 2.*PI/3.), -2./3.*sin(mThetaMech + 2.*PI/3.),
		1./3., 			        1./3., 						       1./3.;

	return abcToDq0;
}

Matrix EMT::Ph3::ReducedOrderSynchronGeneratorVBR::get_inverseParkTransformMatrix() {
	Matrix dq0ToAbc(3, 3);

	dq0ToAbc <<
		cos(mThetaMech), 		    -sin(mThetaMech), 		     1.,
		cos(mThetaMech - 2.*PI/3.), -sin(mThetaMech - 2.*PI/3.), 1.,
		cos(mThetaMech + 2.*PI/3.), -sin(mThetaMech + 2.*PI/3.), 1.;

	return dq0ToAbc;
}
