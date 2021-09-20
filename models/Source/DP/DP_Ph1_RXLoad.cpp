/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <cps/DP/DP_Ph1_RXLoad.h>

using namespace CPS;

DP::Ph1::RXLoad::RXLoad(String uid, String name, Logger::Level logLevel)
	: SimPowerComp<Complex>(uid, name, logLevel) {
	setTerminalNumber(1);

	mSLog->info("Create {} {}", this->type(), name);
	mIntfVoltage = MatrixComp::Zero(1, 1);
	mIntfCurrent = MatrixComp::Zero(1, 1);

	addAttribute<Real>("P", &mActivePower, Flags::read | Flags::write);
	addAttribute<Real>("Q", &mReactivePower, Flags::read | Flags::write);
	addAttribute<Real>("V_nom", &mNomVoltage, Flags::read | Flags::write);
}

DP::Ph1::RXLoad::RXLoad(String name, Logger::Level logLevel)
	: RXLoad(name, name, logLevel) {
}

SimPowerComp<Complex>::Ptr DP::Ph1::RXLoad::clone(String name) {
	auto copy = RXLoad::make(name, mLogLevel);
	if (mParametersSet)
		copy->setParameters(mActivePower, mReactivePower, mNomVoltage);
	return copy;
}

void DP::Ph1::RXLoad::initializeFromNodesAndTerminals(Real frequency) {

	if(!mParametersSet){
		setParameters(
			mTerminals[0]->singleActivePower(),
			mTerminals[0]->singleReactivePower(),
			std::abs(mTerminals[0]->initialSingleVoltage()));
	}

	if (mActivePower != 0) {
		mResistance = std::pow(mNomVoltage, 2) / mActivePower;
		mSubResistor = std::make_shared<DP::Ph1::Resistor>(mName + "_res", mLogLevel);
		mSubResistor->setParameters(mResistance);
		mSubResistor->connect({ SimNode::GND, mTerminals[0]->node() });
		mSubResistor->initialize(mFrequencies);
		mSubResistor->initializeFromNodesAndTerminals(frequency);
	}
	else {
		mResistance = 0;
	}

	if (mReactivePower != 0)
		mReactance = std::pow(mNomVoltage, 2) / mReactivePower;
	else
		mReactance = 0;

	if (mReactance > 0) {
		mInductance = mReactance / (2.*PI*frequency);
		mSubInductor = std::make_shared<DP::Ph1::Inductor>(mName + "_ind", mLogLevel);
		mSubInductor->setParameters(mInductance);
		mSubInductor->connect({ SimNode::GND, mTerminals[0]->node() });
		mSubInductor->initialize(mFrequencies);
		mSubInductor->initializeFromNodesAndTerminals(frequency);
	}
	else if (mReactance < 0) {
		mCapacitance = -1. / (2.*PI*frequency) / mReactance;
		mSubCapacitor = std::make_shared<DP::Ph1::Capacitor>(mName + "_cap", mLogLevel);
		mSubCapacitor->setParameters(mCapacitance);
		mSubCapacitor->connect({ SimNode::GND, mTerminals[0]->node() });
		mSubCapacitor->initialize(mFrequencies);
		mSubCapacitor->initializeFromNodesAndTerminals(frequency);
	}

	mIntfVoltage(0, 0) = mTerminals[0]->initialSingleVoltage();
	mIntfCurrent(0, 0) = std::conj(Complex(mActivePower, mReactivePower) / mIntfVoltage(0, 0));

	mSLog->info(
		"\n--- Initialization from powerflow ---"
		"\nVoltage across: {:s}"
		"\nCurrent: {:s}"
		"\nTerminal 0 voltage: {:s}"
		"\nResistance: {:f}"
		"\nReactance: {:f}"
		"\n--- Initialization from powerflow finished ---",
		Logger::phasorToString(mIntfVoltage(0,0)),
		Logger::phasorToString(mIntfCurrent(0,0)),
		Logger::phasorToString(initialSingleVoltage(0)),
		mResistance,
		mReactance);
}

void DP::Ph1::RXLoad::setParameters(Real activePower, Real reactivePower, Real volt) {
	mParametersSet = true;
	mActivePower = activePower;
	mReactivePower = reactivePower;
	mPower = { mActivePower, mReactivePower};
	mNomVoltage = volt;

	mSLog->info("Active Power={} [W] Reactive Power={} [VAr]", mActivePower, mReactivePower);
	mSLog->info("Nominal Voltage={} [V]", mNomVoltage);
}

void DP::Ph1::RXLoad::mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
	MNAInterface::mnaInitialize(omega, timeStep);
	updateMatrixNodeIndices();

	if (mSubResistor) {
		mSubResistor->mnaInitialize(omega, timeStep, leftVector);
	}
	if (mSubInductor) {
		mSubInductor->mnaInitialize(omega, timeStep, leftVector);
		mRightVectorStamps.push_back(&mSubInductor->attribute<Matrix>("right_vector")->get());
	}
	if (mSubCapacitor) {
		mSubCapacitor->mnaInitialize(omega, timeStep, leftVector);
		mRightVectorStamps.push_back(&mSubInductor->attribute<Matrix>("right_vector")->get());
	}

	mMnaTasks.push_back(std::make_shared<MnaPreStep>(*this));
	mMnaTasks.push_back(std::make_shared<MnaPostStep>(*this, leftVector));
	mRightVector = Matrix::Zero(leftVector->get().rows(), 1);
}

void DP::Ph1::RXLoad::mnaApplyRightSideVectorStamp(Matrix& rightVector) {
	if (mSubResistor)
		mSubResistor->mnaApplyRightSideVectorStamp(rightVector);
	if (mSubInductor)
		mSubInductor->mnaApplyRightSideVectorStamp(rightVector);
	if (mSubCapacitor)
		mSubCapacitor->mnaApplyRightSideVectorStamp(rightVector);
}

void DP::Ph1::RXLoad::mnaApplySystemMatrixStamp(Matrix& systemMatrix) {
	if (mSubResistor)
		mSubResistor->mnaApplySystemMatrixStamp(systemMatrix);
	if (mSubInductor)
		mSubInductor->mnaApplySystemMatrixStamp(systemMatrix);
	if (mSubCapacitor)
		mSubCapacitor->mnaApplySystemMatrixStamp(systemMatrix);
}

void DP::Ph1::RXLoad::mnaUpdateVoltage(const Matrix& leftVector) {
	mIntfVoltage(0, 0) = Math::complexFromVectorElement(leftVector, matrixNodeIndex(0));
}

void DP::Ph1::RXLoad::mnaUpdateCurrent(const Matrix& leftVector) {
	mIntfCurrent(0, 0) = 0;
	if (mSubResistor)
		mIntfCurrent(0, 0) += mSubResistor->intfCurrent()(0,0);
	if (mSubInductor)
		mIntfCurrent(0, 0) += mSubInductor->intfCurrent()(0,0);
	if (mSubCapacitor)
		mIntfCurrent(0, 0) += mSubCapacitor->intfCurrent()(0,0);
}

void DP::Ph1::RXLoad::mnaAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) {
	// add pre-step dependencies of subcomponents
	if (mSubInductor)
		mSubInductor->mnaAddPreStepDependencies(prevStepDependencies, attributeDependencies, modifiedAttributes);
	if (mSubCapacitor)
		mSubCapacitor->mnaAddPreStepDependencies(prevStepDependencies, attributeDependencies, modifiedAttributes);

	// add pre-step dependencies of component itself
	modifiedAttributes.push_back(this->attribute("right_vector"));
}

void DP::Ph1::RXLoad::mnaPreStep(Real time, Int timeStepCount) {
	// pre-step of subcomponents
	if (mSubInductor)
		mSubInductor->mnaPreStep(time, timeStepCount);
	if (mSubCapacitor)
		mSubCapacitor->mnaPreStep(time, timeStepCount);

	// pre-step of component itself
	mnaApplyRightSideVectorStamp(mRightVector);
}

void DP::Ph1::RXLoad::mnaAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector) {
	// add post-step dependencies of subcomponents
	if (mSubResistor)
		mSubResistor->mnaAddPostStepDependencies(prevStepDependencies, attributeDependencies, modifiedAttributes, leftVector);
	if (mSubInductor)
		mSubInductor->mnaAddPostStepDependencies(prevStepDependencies, attributeDependencies, modifiedAttributes, leftVector);
	if (mSubCapacitor)
		mSubCapacitor->mnaAddPostStepDependencies(prevStepDependencies, attributeDependencies, modifiedAttributes, leftVector);

	// add post-step dependencies of component itself
	attributeDependencies.push_back(leftVector);
	modifiedAttributes.push_back(attribute("v_intf"));
	modifiedAttributes.push_back(attribute("i_intf"));
}

void DP::Ph1::RXLoad::mnaPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
	// post-step of subcomponents
	if (mSubResistor)
		mSubResistor->mnaPostStep(time, timeStepCount, leftVector);
	if (mSubInductor)
		mSubInductor->mnaPostStep(time, timeStepCount, leftVector);
	if (mSubCapacitor)
		mSubCapacitor->mnaPostStep(time, timeStepCount, leftVector);

	// post-step of component itself
	mnaUpdateVoltage(*leftVector);
	mnaUpdateCurrent(*leftVector);
}
