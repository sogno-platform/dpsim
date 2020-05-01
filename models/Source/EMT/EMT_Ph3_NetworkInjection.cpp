/* Copyright 2017-2020 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <cps/EMT/EMT_Ph3_NetworkInjection.h>

using namespace CPS;

EMT::Ph3::NetworkInjection::NetworkInjection(String uid, String name, Logger::Level logLevel)
	: SimPowerComp<Real>(uid, name, logLevel), TopologicalPowerComp(uid, name, logLevel) {
	mPhaseType = PhaseType::ABC;
	setVirtualNodeNumber(1);
	setTerminalNumber(1);
	mIntfVoltage = Matrix::Zero(3, 1);
	mIntfCurrent = Matrix::Zero(3, 1);

	mSLog->info("Create {} {}", this->type(), name);

	addAttribute<MatrixComp>("V_ref", Flags::read | Flags::write);
	addAttribute<Real>("f_src", Flags::read | Flags::write);
	mSLog->flush();
}

SimPowerComp<Real>::Ptr EMT::Ph3::NetworkInjection::clone(String name) {
	auto copy = NetworkInjection::make(name, mLogLevel);
	copy->setParameters(attribute<MatrixComp>("V_ref")->get());
	return copy;
}


void EMT::Ph3::NetworkInjection::setParameters(MatrixComp voltageRef, Real srcFreq) {
	attribute<MatrixComp>("V_ref")->set(voltageRef);
	attribute<Real>("f_src")->set(srcFreq);

	parametersSet = true;
}

void EMT::Ph3::NetworkInjection::initializeFromPowerflow(Real frequency) {
	mVoltageRef = attribute<MatrixComp>("V_ref");
	mSrcFreq = attribute<Real>("f_src");
	mSrcFreq->set(frequency);

	MatrixComp vInitABC = MatrixComp::Zero(3, 1);
	vInitABC(0, 0) = RMS3PH_TO_PEAK1PH * initialSingleVoltage(0);
	vInitABC(1, 0) = vInitABC(0, 0) * SHIFT_TO_PHASE_B;
	vInitABC(2, 0) = vInitABC(0, 0) * SHIFT_TO_PHASE_C;

	mVoltageRef->set(vInitABC);

	mSLog->info(
		"\n--- Initialization from node voltages ---"
		"\nReference voltage: {:s}"
		"\nTerminal 0 voltage: {:s}"
		"\n--- Initialization from node voltages ---",
		Logger::matrixCompToString(mVoltageRef->get()),
		Logger::phasorToString(RMS3PH_TO_PEAK1PH * initialSingleVoltage(0)));
	mSLog->flush();
}

// #### MNA functions ####

void EMT::Ph3::NetworkInjection::mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
	MNAInterface::mnaInitialize(omega, timeStep);
	updateMatrixNodeIndices();

	mIntfVoltage = mVoltageRef->get().real();
	mMnaTasks.push_back(std::make_shared<MnaPreStep>(*this));
	mMnaTasks.push_back(std::make_shared<MnaPostStep>(*this, leftVector));
	mRightVector = Matrix::Zero(leftVector->get().rows(), 1);
}

void EMT::Ph3::NetworkInjection::mnaApplySystemMatrixStamp(Matrix& systemMatrix) {
	Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::A), matrixNodeIndex(0, 0), 1);
	Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 0), mVirtualNodes[0]->matrixNodeIndex(PhaseType::A), 1);
	Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::B), matrixNodeIndex(0, 1), 1);
	Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 1), mVirtualNodes[0]->matrixNodeIndex(PhaseType::B), 1);
	Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::C), matrixNodeIndex(0, 2), 1);
	Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 2), mVirtualNodes[0]->matrixNodeIndex(PhaseType::C), 1);

	mSLog->info("-- Stamp ---");
	mSLog->info("Add {:f} to system at ({:d},{:d})", 1., mVirtualNodes[0]->matrixNodeIndex(PhaseType::A), matrixNodeIndex(0, 0));
	mSLog->info("Add {:f} to system at ({:d},{:d})", 1., matrixNodeIndex(0, 0), mVirtualNodes[0]->matrixNodeIndex(PhaseType::A));
	mSLog->info("Add {:f} to system at ({:d},{:d})", 1., mVirtualNodes[0]->matrixNodeIndex(PhaseType::B), matrixNodeIndex(0, 1));
	mSLog->info("Add {:f} to system at ({:d},{:d})", 1., matrixNodeIndex(0, 1), mVirtualNodes[0]->matrixNodeIndex(PhaseType::B));
	mSLog->info("Add {:f} to system at ({:d},{:d})", 1., mVirtualNodes[0]->matrixNodeIndex(PhaseType::C), matrixNodeIndex(0, 2));
	mSLog->info("Add {:f} to system at ({:d},{:d})", 1., matrixNodeIndex(0, 2), mVirtualNodes[0]->matrixNodeIndex(PhaseType::C));
	mSLog->flush();
}

void EMT::Ph3::NetworkInjection::mnaApplyRightSideVectorStamp(Matrix& rightVector) {
	Math::setVectorElement(rightVector, mVirtualNodes[0]->matrixNodeIndex(PhaseType::A), mIntfVoltage(0, 0));
	Math::setVectorElement(rightVector, mVirtualNodes[0]->matrixNodeIndex(PhaseType::B), mIntfVoltage(1, 0));
	Math::setVectorElement(rightVector, mVirtualNodes[0]->matrixNodeIndex(PhaseType::C), mIntfVoltage(2, 0));
	mSLog->debug("Add phase A {:f} to source vector at {:d}",
		"Add phase B {:f} to source vector at {:d}",
		"Add phase C {:f} to source vector at {:d}",
		mIntfVoltage(0, 0),
		mVirtualNodes[0]->matrixNodeIndex(PhaseType::A),
		mIntfVoltage(1, 0),
		mVirtualNodes[0]->matrixNodeIndex(PhaseType::B),
		mIntfVoltage(2, 0),
		mVirtualNodes[0]->matrixNodeIndex(PhaseType::C));
	mSLog->flush();
}


void EMT::Ph3::NetworkInjection::updateVoltage(Real time) {
	if (mSrcFreq->get() < 0) {
		mIntfVoltage = mVoltageRef->get().real();
	}
	else {
		mIntfVoltage(0, 0) =
			Math::abs(mVoltageRef->get()(0, 0)) * cos(time * 2. * PI * mSrcFreq->get() + Math::phase(mVoltageRef->get())(0, 0));
		mIntfVoltage(1, 0) =
			Math::abs(mVoltageRef->get()(1, 0)) * cos(time * 2. * PI * mSrcFreq->get() + Math::phase(mVoltageRef->get())(1, 0));
		mIntfVoltage(2, 0) =
			Math::abs(mVoltageRef->get()(2, 0)) * cos(time * 2. * PI * mSrcFreq->get() + Math::phase(mVoltageRef->get())(2, 0));
	}
}

void EMT::Ph3::NetworkInjection::MnaPreStep::execute(Real time, Int timeStepCount) {
	mNetworkInjection.updateVoltage(time);
	mNetworkInjection.mnaApplyRightSideVectorStamp(mNetworkInjection.mRightVector);
}


void EMT::Ph3::NetworkInjection::MnaPostStep::execute(Real time, Int timeStepCount) {
	mNetworkInjection.mnaUpdateCurrent(*mLeftVector);
}

void EMT::Ph3::NetworkInjection::mnaUpdateCurrent(const Matrix& leftVector) {
	mIntfCurrent(0, 0) = Math::realFromVectorElement(leftVector, mVirtualNodes[0]->matrixNodeIndex(PhaseType::A));
	mIntfCurrent(1, 0) = Math::realFromVectorElement(leftVector, mVirtualNodes[0]->matrixNodeIndex(PhaseType::B));
	mIntfCurrent(2, 0) = Math::realFromVectorElement(leftVector, mVirtualNodes[0]->matrixNodeIndex(PhaseType::C));
}
