/* Copyright 2017-2020 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <cps/EMT/EMT_Ph3_ControlledVoltageSource.h>
#include <cps/CIM/Reader.h>


using namespace CPS;

EMT::Ph3::ControlledVoltageSource::ControlledVoltageSource(String uid, String name, Logger::Level logLevel)
	: SimPowerComp<Real>(uid, name, logLevel) {
	mPhaseType = PhaseType::ABC;
	setVirtualNodeNumber(0);
	setTerminalNumber(2);
	mIntfVoltage = Matrix::Zero(3, 1);
	mIntfCurrent = Matrix::Zero(3, 1);

	mSLog->info("Create {} {}", this->type(), name);
	mSubVoltageSource = std::make_shared<EMT::Ph3::VoltageSource>(mName + "_vs", mLogLevel);
	mSubComponents.push_back(mSubVoltageSource);
	mSLog->info("Electrical subcomponents: ");
	for (auto subcomp: mSubComponents)
		mSLog->info("- {}", subcomp->name());

	addAttribute<MatrixComp>("V_ref", Flags::read | Flags::write);  // rms-value, phase-to-phase
	addAttribute<Real>("f_src", Flags::read | Flags::write);
}

SimPowerComp<Real>::Ptr EMT::Ph3::ControlledVoltageSource::clone(String name) {
	auto copy = ControlledVoltageSource::make(name, mLogLevel);
	return copy;
}

void EMT::Ph3::ControlledVoltageSource::initializeFromNodesAndTerminals(Real frequency) {
	// Connect electrical subcomponents
	mSubVoltageSource->connect({ node(0), node(1) });
	
	// Initialize electrical subcomponents
	mSubVoltageSource->initialize(mFrequencies);
	// mSubVoltageSource->initializeFromNodesAndTerminals(frequency);
}

// #### MNA functions ####

void EMT::Ph3::ControlledVoltageSource::mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
	MNAInterface::mnaInitialize(omega, timeStep);
	updateMatrixNodeIndices();

	// initialize electrical subcomponents
	mSubVoltageSource->mnaInitialize(omega, timeStep, leftVector);

	// collect right side vectors of subcomponents
	mRightVectorStamps.push_back(&mSubVoltageSource->attribute<Matrix>("right_vector")->get());

	// collect tasks
	mMnaTasks.push_back(std::make_shared<MnaPreStep>(*this));
	mMnaTasks.push_back(std::make_shared<MnaPostStep>(*this, leftVector));

	mRightVector = Matrix::Zero(leftVector->get().rows(), 1);
}

void EMT::Ph3::ControlledVoltageSource::mnaApplySystemMatrixStamp(Matrix& systemMatrix) {
	mSubVoltageSource->mnaApplySystemMatrixStamp(systemMatrix);
}

void EMT::Ph3::ControlledVoltageSource::mnaApplyRightSideVectorStamp(Matrix& rightVector) {
	rightVector.setZero();
	for (auto stamp : mRightVectorStamps)
		rightVector += *stamp;

	mSLog->debug("Right Side Vector: {:s}",
				Logger::matrixToString(rightVector));
}

void EMT::Ph3::ControlledVoltageSource::mnaAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) {
	mSubVoltageSource->mnaAddPreStepDependencies(prevStepDependencies, attributeDependencies, modifiedAttributes);
	// add pre-step dependencies of component itself
	prevStepDependencies.push_back(attribute("i_intf"));
	prevStepDependencies.push_back(attribute("v_intf"));
	modifiedAttributes.push_back(attribute("right_vector"));
}

void EMT::Ph3::ControlledVoltageSource::updateSignalReference(Real time, Int timeStepCount) {
	MatrixComp vref = MatrixComp::Zero(3,1);
	
	// TODO: currently support only hard-coded sinusoid
	// later signal components can be used here
	MatrixComp vrefcos = CIM::Reader::singlePhaseVariableToThreePhase(CPS::Math::polar(100000, 0));
	Real fref = 50;

	vref(0, 0) = Math::abs(vrefcos(0, 0)) * cos(time * 2. * PI * fref + Math::phase(vrefcos(0, 0)));
	vref(1, 0) = Math::abs(vrefcos(1, 0)) * cos(time * 2. * PI * fref + Math::phase(vrefcos(1, 0)));
	vref(2, 0) = Math::abs(vrefcos(2, 0)) * cos(time * 2. * PI * fref + Math::phase(vrefcos(2, 0)));
	
	mSubVoltageSource->attribute<MatrixComp>("V_ref")->set(vref);
}

void EMT::Ph3::ControlledVoltageSource::mnaPreStep(Real time, Int timeStepCount) {
	updateSignalReference(time, timeStepCount);
	// pre-step of subcomponents
	mSubVoltageSource->mnaPreStep(time, timeStepCount);
	// pre-step of component itself
	mnaApplyRightSideVectorStamp(mRightVector);
}

void EMT::Ph3::ControlledVoltageSource::mnaAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector) {
	// add post-step dependencies of subcomponents
	mSubVoltageSource->mnaAddPostStepDependencies(prevStepDependencies, attributeDependencies, modifiedAttributes, leftVector);
	// add post-step dependencies of component itself
	attributeDependencies.push_back(leftVector);
	modifiedAttributes.push_back(attribute("v_intf"));
	modifiedAttributes.push_back(attribute("i_intf"));
};

void EMT::Ph3::ControlledVoltageSource::mnaPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
	// post-step of subcomponents
	mSubVoltageSource->mnaPostStep(time, timeStepCount, leftVector);
	// post-step of component itself
	mnaUpdateCurrent(*leftVector);
	mnaUpdateVoltage(*leftVector);
}

void EMT::Ph3::ControlledVoltageSource::mnaUpdateVoltage(const Matrix& leftVector) {
	mIntfVoltage = mSubVoltageSource->attribute<Matrix>("v_intf")->get();
}

void EMT::Ph3::ControlledVoltageSource::mnaUpdateCurrent(const Matrix& leftVector) {
	mIntfCurrent = mSubVoltageSource->attribute<Matrix>("i_intf")->get();
}
