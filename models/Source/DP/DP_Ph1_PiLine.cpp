/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <cps/DP/DP_Ph1_PiLine.h>

using namespace CPS;

DP::Ph1::PiLine::PiLine(String uid, String name, Logger::Level logLevel)
	: SimPowerComp<Complex>(uid, name, logLevel) {
	setVirtualNodeNumber(1);
	setTerminalNumber(2);

	mSLog->info("Create {} {}", this->type(), name);
	mIntfVoltage = MatrixComp::Zero(1,1);
	mIntfCurrent = MatrixComp::Zero(1,1);

	addAttribute<Real>("R_series", &mSeriesRes, Flags::read | Flags::write);
	addAttribute<Real>("L_series", &mSeriesInd, Flags::read | Flags::write);
	addAttribute<Real>("C_parallel", &mParallelCap, Flags::read | Flags::write);
	addAttribute<Real>("G_parallel", &mParallelCond, Flags::read | Flags::write);
}

SimPowerComp<Complex>::Ptr DP::Ph1::PiLine::clone(String name) {
	auto copy = PiLine::make(name, mLogLevel);
	copy->setParameters(mSeriesRes, mSeriesInd, mParallelCap, mParallelCond);
	return copy;
}

void DP::Ph1::PiLine::initializeFromNodesAndTerminals(Real frequency) {

	// Static calculation
	Real omega = 2.*PI * frequency;
	Complex impedance = { mSeriesRes, omega * mSeriesInd };
	mIntfVoltage(0,0) = initialSingleVoltage(1) - initialSingleVoltage(0);
	mIntfCurrent(0,0) = mIntfVoltage(0,0) / impedance;

	// Initialization of virtual node
	mVirtualNodes[0]->setInitialVoltage( initialSingleVoltage(0) + mIntfCurrent(0,0) * mSeriesRes );

	// Create series sub components
	mSubSeriesResistor = std::make_shared<DP::Ph1::Resistor>(mName + "_res", mLogLevel);
	mSubSeriesResistor->setParameters(mSeriesRes);
	mSubSeriesResistor->connect({ mTerminals[0]->node(), mVirtualNodes[0] });
	mSubSeriesResistor->initialize(mFrequencies);
	mSubSeriesResistor->initializeFromNodesAndTerminals(frequency);

	mSubSeriesInductor = std::make_shared<DP::Ph1::Inductor>(mName + "_ind", mLogLevel);
	mSubSeriesInductor->setParameters(mSeriesInd);
	mSubSeriesInductor->connect({ mVirtualNodes[0], mTerminals[1]->node() });
	mSubSeriesInductor->initialize(mFrequencies);
	mSubSeriesInductor->initializeFromNodesAndTerminals(frequency);

	// By default there is always a small conductance to ground to
	// avoid problems with floating nodes.
	Real defaultParallelCond = 1e-6;
	mParallelCond = (mParallelCond > 0) ? mParallelCond : defaultParallelCond;

	// Create parallel sub components
	mSubParallelResistor0 = std::make_shared<DP::Ph1::Resistor>(mName + "_con0", mLogLevel);
	mSubParallelResistor0->setParameters(2./mParallelCond);
	mSubParallelResistor0->connect(SimNode::List{ SimNode::GND, mTerminals[0]->node() });
	mSubParallelResistor0->initialize(mFrequencies);
	mSubParallelResistor0->initializeFromNodesAndTerminals(frequency);

	mSubParallelResistor1 = std::make_shared<DP::Ph1::Resistor>(mName + "_con1", mLogLevel);
	mSubParallelResistor1->setParameters(2./mParallelCond);
	mSubParallelResistor1->connect(SimNode::List{ SimNode::GND, mTerminals[1]->node() });
	mSubParallelResistor1->initialize(mFrequencies);
	mSubParallelResistor1->initializeFromNodesAndTerminals(frequency);


	if (mParallelCap >= 0) {
		mSubParallelCapacitor0 = std::make_shared<DP::Ph1::Capacitor>(mName + "_cap0", mLogLevel);
		mSubParallelCapacitor0->setParameters(mParallelCap/2.);
		mSubParallelCapacitor0->connect(SimNode::List{ SimNode::GND, mTerminals[0]->node() });
		mSubParallelCapacitor0->initialize(mFrequencies);
		mSubParallelCapacitor0->initializeFromNodesAndTerminals(frequency);

		mSubParallelCapacitor1 = std::make_shared<DP::Ph1::Capacitor>(mName + "_cap1", mLogLevel);
		mSubParallelCapacitor1->setParameters(mParallelCap/2.);
		mSubParallelCapacitor1->connect(SimNode::List{ SimNode::GND, mTerminals[1]->node() });
		mSubParallelCapacitor1->initialize(mFrequencies);
		mSubParallelCapacitor1->initializeFromNodesAndTerminals(frequency);
	}

	mSLog->info(
		"\n--- Initialization from powerflow ---"
		"\nVoltage across: {:s}"
		"\nCurrent: {:s}"
		"\nTerminal 0 voltage: {:s}"
		"\nTerminal 1 voltage: {:s}"
		"\nVirtual Node 1 voltage: {:s}"
		"\n--- Initialization from powerflow finished ---",
		Logger::phasorToString(mIntfVoltage(0,0)),
		Logger::phasorToString(mIntfCurrent(0,0)),
		Logger::phasorToString(initialSingleVoltage(0)),
		Logger::phasorToString(initialSingleVoltage(1)),
		Logger::phasorToString(mVirtualNodes[0]->initialSingleVoltage()));
}

void DP::Ph1::PiLine::mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
	MNAInterface::mnaInitialize(omega, timeStep);
	updateMatrixNodeIndices();
	MNAInterface::List subComps({mSubSeriesResistor, mSubSeriesInductor});

	mSubSeriesResistor->mnaInitialize(omega, timeStep, leftVector);
	mSubSeriesInductor->mnaInitialize(omega, timeStep, leftVector);
	mRightVectorStamps.push_back(&mSubSeriesInductor->attribute<Matrix>("right_vector")->get());

	mSubParallelResistor0->mnaInitialize(omega, timeStep, leftVector);
	mSubParallelResistor1->mnaInitialize(omega, timeStep, leftVector);
	subComps.push_back(mSubParallelResistor0);
	subComps.push_back(mSubParallelResistor1);

	if (mParallelCap >= 0) {
		mSubParallelCapacitor0->mnaInitialize(omega, timeStep, leftVector);
		mSubParallelCapacitor1->mnaInitialize(omega, timeStep, leftVector);
		mRightVectorStamps.push_back(&mSubParallelCapacitor0->attribute<Matrix>("right_vector")->get());
		mRightVectorStamps.push_back(&mSubParallelCapacitor1->attribute<Matrix>("right_vector")->get());
		subComps.push_back(mSubParallelCapacitor0);
		subComps.push_back(mSubParallelCapacitor1);
	}
	mMnaTasks.push_back(std::make_shared<MnaPreStep>(*this));
	mMnaTasks.push_back(std::make_shared<MnaPostStep>(*this, leftVector));
	mRightVector = Matrix::Zero(leftVector->get().rows(), 1);
}

void DP::Ph1::PiLine::mnaApplySystemMatrixStamp(Matrix& systemMatrix) {
	mSubSeriesResistor->mnaApplySystemMatrixStamp(systemMatrix);
	mSubSeriesInductor->mnaApplySystemMatrixStamp(systemMatrix);

	mSubParallelResistor0->mnaApplySystemMatrixStamp(systemMatrix);
	mSubParallelResistor1->mnaApplySystemMatrixStamp(systemMatrix);

	if (mParallelCap >= 0) {
		mSubParallelCapacitor0->mnaApplySystemMatrixStamp(systemMatrix);
		mSubParallelCapacitor1->mnaApplySystemMatrixStamp(systemMatrix);
	}
}

void DP::Ph1::PiLine::mnaApplyRightSideVectorStamp(Matrix& rightVector) {
	rightVector.setZero();
	for (auto stamp : mRightVectorStamps)
		rightVector += *stamp;
}

void DP::Ph1::PiLine::mnaAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) {
	// add pre-step dependencies of subcomponents
	this->mSubSeriesInductor->mnaAddPreStepDependencies(prevStepDependencies, attributeDependencies, modifiedAttributes);
	if (this->mParallelCap >= 0) {
		this->mSubParallelCapacitor0->mnaAddPreStepDependencies(prevStepDependencies, attributeDependencies, modifiedAttributes);
		this->mSubParallelCapacitor1->mnaAddPreStepDependencies(prevStepDependencies, attributeDependencies, modifiedAttributes);
	}
	// add pre-step dependencies of component itself
	prevStepDependencies.push_back(this->attribute("i_intf"));
	prevStepDependencies.push_back(this->attribute("v_intf"));
	modifiedAttributes.push_back(this->attribute("right_vector"));
}

void DP::Ph1::PiLine::mnaPreStep(Real time, Int timeStepCount) {
	// pre-step of subcomponents
	this->mSubSeriesInductor->mnaPreStep(time, timeStepCount);
	if (this->mParallelCap >= 0) {
		this->mSubParallelCapacitor0->mnaPreStep(time, timeStepCount);
		this->mSubParallelCapacitor1->mnaPreStep(time, timeStepCount);
	}
	// pre-step of component itself
	this->mnaApplyRightSideVectorStamp(this->mRightVector);
}

void DP::Ph1::PiLine::mnaAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector) {
	// add post-step dependencies of subcomponents
	this->mSubSeriesResistor->mnaAddPostStepDependencies(prevStepDependencies, attributeDependencies, modifiedAttributes, leftVector);
	this->mSubSeriesInductor->mnaAddPostStepDependencies(prevStepDependencies, attributeDependencies, modifiedAttributes, leftVector);
	if (this->mParallelCap >= 0) {
		this->mSubParallelCapacitor0->mnaAddPostStepDependencies(prevStepDependencies, attributeDependencies, modifiedAttributes, leftVector);
		this->mSubParallelCapacitor1->mnaAddPostStepDependencies(prevStepDependencies, attributeDependencies, modifiedAttributes, leftVector);
	}

	this->mSubParallelResistor0->mnaAddPostStepDependencies(prevStepDependencies, attributeDependencies, modifiedAttributes, leftVector);
	this->mSubParallelResistor1->mnaAddPostStepDependencies(prevStepDependencies, attributeDependencies, modifiedAttributes, leftVector);

	// add post-step dependencies of component itself
	attributeDependencies.push_back(leftVector);
	modifiedAttributes.push_back(this->attribute("v_intf"));
	modifiedAttributes.push_back(this->attribute("i_intf"));
}

void DP::Ph1::PiLine::mnaPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
	// post-step of subcomponents
	this->mSubSeriesResistor->mnaPostStep(time, timeStepCount, leftVector);
	this->mSubSeriesInductor->mnaPostStep(time, timeStepCount, leftVector);
	if (this->mParallelCap >= 0) {
		this->mSubParallelCapacitor0->mnaPostStep(time, timeStepCount, leftVector);
		this->mSubParallelCapacitor1->mnaPostStep(time, timeStepCount, leftVector);
	}

	this->mSubParallelResistor0->mnaPostStep(time, timeStepCount, leftVector);
	this->mSubParallelResistor1->mnaPostStep(time, timeStepCount, leftVector);

	// post-step of component itself
	this->mnaUpdateVoltage(*leftVector);
	this->mnaUpdateCurrent(*leftVector);
}

void DP::Ph1::PiLine::mnaUpdateVoltage(const Matrix& leftVector) {
	mIntfVoltage(0, 0) = 0;
	if (terminalNotGrounded(1))
		mIntfVoltage(0,0) = Math::complexFromVectorElement(leftVector, matrixNodeIndex(1));
	if (terminalNotGrounded(0))
		mIntfVoltage(0,0) = mIntfVoltage(0,0) - Math::complexFromVectorElement(leftVector, matrixNodeIndex(0));
}

void DP::Ph1::PiLine::mnaUpdateCurrent(const Matrix& leftVector) {
	mIntfCurrent(0,0) = mSubSeriesInductor->intfCurrent()(0, 0);
}

MNAInterface::List DP::Ph1::PiLine::mnaTearGroundComponents() {
	MNAInterface::List gndComponents;

	gndComponents.push_back(mSubParallelResistor0);
	gndComponents.push_back(mSubParallelResistor1);

	if (mParallelCap >= 0) {
		gndComponents.push_back(mSubParallelCapacitor0);
		gndComponents.push_back(mSubParallelCapacitor1);
	}

	return gndComponents;
}

void DP::Ph1::PiLine::mnaTearInitialize(Real omega, Real timeStep) {
	mSubSeriesResistor->mnaTearSetIdx(mTearIdx);
	mSubSeriesResistor->mnaTearInitialize(omega, timeStep);
	mSubSeriesInductor->mnaTearSetIdx(mTearIdx);
	mSubSeriesInductor->mnaTearInitialize(omega, timeStep);
}

void DP::Ph1::PiLine::mnaTearApplyMatrixStamp(Matrix& tearMatrix) {
	mSubSeriesResistor->mnaTearApplyMatrixStamp(tearMatrix);
	mSubSeriesInductor->mnaTearApplyMatrixStamp(tearMatrix);
}

void DP::Ph1::PiLine::mnaTearApplyVoltageStamp(Matrix& voltageVector) {
	mSubSeriesInductor->mnaTearApplyVoltageStamp(voltageVector);
}

void DP::Ph1::PiLine::mnaTearPostStep(Complex voltage, Complex current) {
	mSubSeriesInductor->mnaTearPostStep(voltage - current * mSeriesRes, current);
}
