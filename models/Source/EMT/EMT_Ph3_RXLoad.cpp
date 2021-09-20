/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <cps/EMT/EMT_Ph3_RXLoad.h>

using namespace CPS;

EMT::Ph3::RXLoad::RXLoad(String uid, String name,
	Logger::Level logLevel)
	: SimPowerComp<Real>(uid, name, logLevel) {
	mPhaseType = PhaseType::ABC;
	setTerminalNumber(1);

	mSLog->info("Create {} {}", this->type(), name);
	mIntfVoltage = Matrix::Zero(3, 1);
	mIntfCurrent = Matrix::Zero(3, 1);

	addAttribute<Matrix>("P", &mActivePower, Flags::read | Flags::write);
	addAttribute<Matrix>("Q", &mReactivePower, Flags::read | Flags::write);
	addAttribute<Real>("V_nom", &mNomVoltage, Flags::read | Flags::write);
	mSLog->flush();
}

EMT::Ph3::RXLoad::RXLoad(String name,
	Logger::Level logLevel)
	: RXLoad(name, name, logLevel) {
}

EMT::Ph3::RXLoad::RXLoad(String name,
	Matrix activePower, Matrix reactivePower, Real volt,
	Logger::Level logLevel)
	: RXLoad(name, logLevel) {
	mActivePower = activePower;
	mReactivePower = reactivePower;
	mPower = MatrixComp::Zero(3,3);
	mPower <<
		Complex(mActivePower(0, 0), mReactivePower(0, 0)), Complex(mActivePower(0, 1), mReactivePower(0, 1)), Complex(mActivePower(0, 2), mReactivePower(0, 2)),
		Complex(mActivePower(1, 0), mReactivePower(1, 0)), Complex(mActivePower(1, 1), mReactivePower(1, 1)), Complex(mActivePower(1, 2), mReactivePower(1, 2)),
		Complex(mActivePower(2, 0), mReactivePower(2, 0)), Complex(mActivePower(2, 1), mReactivePower(2, 1)), Complex(mActivePower(2, 2), mReactivePower(2, 2));

	mNomVoltage = volt;
	initPowerFromTerminal = false;
}

void EMT::Ph3::RXLoad::setParameters(Matrix activePower, Matrix reactivePower, Real volt) {
	mActivePower = activePower;
	mReactivePower = reactivePower;

	// complex power
	mPower = MatrixComp::Zero(3, 3);
	mPower(0, 0) = { mActivePower(0, 0), mReactivePower(0, 0) };
	mPower(1, 1) = { mActivePower(1, 1), mReactivePower(1, 1) };
	mPower(2, 2) = { mActivePower(2, 2), mReactivePower(2, 2) };

	mNomVoltage = volt;

	mSLog->info("\nActive Power [W]: {}"
			"\nReactive Power [VAr]: {}",
			Logger::matrixToString(mActivePower),
			Logger::matrixToString(mReactivePower));
	mSLog->info("Nominal Voltage={} [V]", mNomVoltage);

	initPowerFromTerminal = false;
}

SimPowerComp<Real>::Ptr EMT::Ph3::RXLoad::clone(String name) {
	// everything set by initializeFromNodesAndTerminals
	return RXLoad::make(name, mLogLevel);
}

void EMT::Ph3::RXLoad::initializeFromNodesAndTerminals(Real frequency) {

		if (initPowerFromTerminal) {
		mActivePower = Matrix::Zero(3, 3);
		mActivePower(0, 0) = mTerminals[0]->singleActivePower() / 3.;
		mActivePower(1, 1) = mTerminals[0]->singleActivePower() / 3.;
		mActivePower(2, 2) = mTerminals[0]->singleActivePower() / 3.;

		mReactivePower = Matrix::Zero(3, 3);
		mReactivePower(0, 0) = mTerminals[0]->singleReactivePower() / 3.;
		mReactivePower(1, 1) = mTerminals[0]->singleReactivePower() / 3.;
		mReactivePower(2, 2) = mTerminals[0]->singleReactivePower() / 3.;

		// complex power
		mPower = MatrixComp::Zero(3, 3);
		mPower(0, 0) = { mActivePower(0, 0), mReactivePower(0, 0) };
		mPower(1, 1) = { mActivePower(1, 1), mReactivePower(1, 1) };
		mPower(2, 2) = { mActivePower(2, 2), mReactivePower(2, 2) };

		mNomVoltage = std::abs(mTerminals[0]->initialSingleVoltage());

		mSLog->info("\nActive Power [W]: {}"
					"\nReactive Power [VAr]: {}",
					Logger::matrixToString(mActivePower),
					Logger::matrixToString(mReactivePower));
		mSLog->info("Nominal Voltage={} [V]", mNomVoltage);
	}

	if (mActivePower(0,0) != 0) {
		mResistance = std::pow(mNomVoltage/sqrt(3), 2) * mActivePower.inverse();
		mConductance = mResistance.inverse();
		mSubResistor = std::make_shared<EMT::Ph3::Resistor>(mName + "_res", mLogLevel);
		mSubResistor->setParameters(mResistance);
		mSubResistor->connect({ SimNode::GND, mTerminals[0]->node() });
		mSubResistor->initialize(mFrequencies);
		mSubResistor->initializeFromNodesAndTerminals(frequency);
	}

	if (mReactivePower(0, 0) != 0)
		mReactance = std::pow(mNomVoltage/sqrt(3), 2) * mReactivePower.inverse();
	else
		mReactance = Matrix::Zero(1, 1);

	if (mReactance(0,0) > 0) {
		mInductance = mReactance / (2 * PI * frequency);

		mSubInductor = std::make_shared<EMT::Ph3::Inductor>(mName + "_ind", mLogLevel);
		mSubInductor->setParameters(mInductance);
		mSubInductor->connect({ SimNode::GND, mTerminals[0]->node() });
		mSubInductor->initialize(mFrequencies);
		mSubInductor->initializeFromNodesAndTerminals(frequency);
	}
	else if (mReactance(0,0) < 0) {
		mCapacitance = -1 / (2 * PI * frequency) * mReactance.inverse();

		mSubCapacitor = std::make_shared<EMT::Ph3::Capacitor>(mName + "_cap", mLogLevel);
		mSubCapacitor->setParameters(mCapacitance);
		mSubCapacitor->connect({ SimNode::GND, mTerminals[0]->node() });
		mSubCapacitor->initialize(mFrequencies);
		mSubCapacitor->initializeFromNodesAndTerminals(frequency);
	}

	MatrixComp vInitABC = MatrixComp::Zero(3, 1);
	vInitABC(0, 0) = RMS3PH_TO_PEAK1PH * mTerminals[0]->initialSingleVoltage();
	vInitABC(1, 0) = vInitABC(0, 0) * SHIFT_TO_PHASE_B;
	vInitABC(2, 0) = vInitABC(0, 0) * SHIFT_TO_PHASE_C;
	mIntfVoltage = vInitABC.real();

	MatrixComp iInitABC = MatrixComp::Zero(3, 1);
	// v i^T* = S
	// v^T v i^T* = v^T S
	// i^T*= (|v|^2)^(-1) v^T S

	Complex v_ = vInitABC(0, 0)*vInitABC(0, 0) + vInitABC(1, 0)*vInitABC(1, 0) + vInitABC(2, 0)*vInitABC(2, 0);
	MatrixComp rhs_ = Complex(1, 0) / v_ * vInitABC.transpose() * mPower;
	iInitABC = rhs_.conjugate().transpose();
	mIntfCurrent = iInitABC.real();

	mSLog->info(
		"\n--- Initialization from powerflow ---"
		"\nVoltage across: {:s}"
		"\nCurrent: {:s}"
		"\nTerminal 0 voltage: {:s}"
		"\nActive Power: {:s}"
		"\nReactive Power: {:s}"
		"\nResistance: {:s}"
		"\nReactance: {:s}"
		"\n--- Initialization from powerflow finished ---",
		Logger::matrixToString(mIntfVoltage),
		Logger::matrixToString(mIntfCurrent),
		Logger::phasorToString(RMS3PH_TO_PEAK1PH * initialSingleVoltage(0)),
		Logger::matrixToString(mActivePower),
		Logger::matrixToString(mReactivePower),
		Logger::matrixToString(mResistance),
		Logger::matrixToString(mReactance));
	mSLog->flush();

}

void EMT::Ph3::RXLoad::mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
	MNAInterface::mnaInitialize(omega, timeStep);
	updateMatrixNodeIndices();
	mRightVector = Matrix::Zero(leftVector->get().rows(), 1);
	if (mSubResistor) {
		mSubResistor->mnaInitialize(omega, timeStep, leftVector);
		for (auto task : mSubResistor->mnaTasks()) {
			mMnaTasks.push_back(task);
		}
	}
	if (mSubInductor) {
		mSubInductor->mnaInitialize(omega, timeStep, leftVector);
		for (auto task : mSubInductor->mnaTasks()) {
			mMnaTasks.push_back(task);
		}
	}
	if (mSubCapacitor) {
		mSubCapacitor->mnaInitialize(omega, timeStep, leftVector);
		for (auto task : mSubCapacitor->mnaTasks()) {
			mMnaTasks.push_back(task);
		}
	}
	mMnaTasks.push_back(std::make_shared<MnaPreStep>(*this));
	mMnaTasks.push_back(std::make_shared<MnaPostStep>(*this, leftVector));
}

void EMT::Ph3::RXLoad::mnaApplyRightSideVectorStamp(Matrix& rightVector) {
	if (mSubResistor)
		mSubResistor->mnaApplyRightSideVectorStamp(rightVector);
	if (mSubInductor)
		mSubInductor->mnaApplyRightSideVectorStamp(rightVector);
	if (mSubCapacitor)
		mSubCapacitor->mnaApplyRightSideVectorStamp(rightVector);
}

void EMT::Ph3::RXLoad::mnaApplySystemMatrixStamp(Matrix& systemMatrix) {
	if (mSubResistor)
		mSubResistor->mnaApplySystemMatrixStamp(systemMatrix);
	if (mSubInductor)
		mSubInductor->mnaApplySystemMatrixStamp(systemMatrix);
	if (mSubCapacitor)
		mSubCapacitor->mnaApplySystemMatrixStamp(systemMatrix);
}

void EMT::Ph3::RXLoad::MnaPreStep::execute(Real time, Int timeStepCount) {
	mLoad.mnaApplyRightSideVectorStamp(mLoad.mRightVector);
}

void EMT::Ph3::RXLoad::MnaPostStep::execute(Real time, Int timeStepCount) {
	mLoad.mnaUpdateVoltage(*mLeftVector);
	mLoad.mnaUpdateCurrent(*mLeftVector);
}

void EMT::Ph3::RXLoad::mnaUpdateVoltage(const Matrix& leftVector) {
	mIntfVoltage = Matrix::Zero(3, 1);
	mIntfVoltage(0, 0) = Math::realFromVectorElement(leftVector, matrixNodeIndex(0, 0));
	mIntfVoltage(1, 0) = Math::realFromVectorElement(leftVector, matrixNodeIndex(0, 1));
	mIntfVoltage(2, 0) = Math::realFromVectorElement(leftVector, matrixNodeIndex(0, 2));
}

void EMT::Ph3::RXLoad::mnaUpdateCurrent(const Matrix& leftVector) {
	mIntfCurrent = Matrix::Zero(3, 1);
	if (mSubResistor)
		mIntfCurrent += mSubResistor->intfCurrent();
	if (mSubInductor)
		mIntfCurrent += mSubInductor->intfCurrent();
	if (mSubCapacitor)
		mIntfCurrent += mSubCapacitor->intfCurrent();
}
