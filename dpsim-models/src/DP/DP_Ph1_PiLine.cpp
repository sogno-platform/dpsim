/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/DP/DP_Ph1_PiLine.h>

using namespace CPS;

DP::Ph1::PiLine::PiLine(String uid, String name, Logger::Level logLevel)
	: Base::Ph1::PiLine(mAttributes), CompositePowerComp<Complex>(uid, name, true, true, logLevel) {
	setTerminalNumber(2);

	SPDLOG_LOGGER_INFO(mSLog, "Create {} {}", this->type(), name);
	**mIntfVoltage = MatrixComp::Zero(1,1);
	**mIntfCurrent = MatrixComp::Zero(1,1);
}

///DEPRECATED: Remove method
SimPowerComp<Complex>::Ptr DP::Ph1::PiLine::clone(String name) {
	auto copy = PiLine::make(name, mLogLevel);
	copy->setParameters(**mSeriesRes, **mSeriesInd, **mParallelCap, **mParallelCond);
	return copy;
}

void DP::Ph1::PiLine::initializeFromNodesAndTerminals(Real frequency) {

	// Static calculation
	Real omega = 2.*PI * frequency;
	Complex impedance = { **mSeriesRes, omega * **mSeriesInd };
	(**mIntfVoltage)(0,0) = initialSingleVoltage(1) - initialSingleVoltage(0);
	(**mIntfCurrent)(0,0) = (**mIntfVoltage)(0,0) / impedance;

	// Create series rl sub component
	mSubSeriesElement = std::make_shared<DP::Ph1::ResIndSeries>(**mName + "_ResIndSeries", mLogLevel);
	mSubSeriesElement->connect({ mTerminals[0]->node(), mTerminals[1]->node() });
	mSubSeriesElement->setParameters(**mSeriesRes, **mSeriesInd);
	mSubSeriesElement->initialize(mFrequencies);
	mSubSeriesElement->initializeFromNodesAndTerminals(frequency);
	addMNASubComponent(mSubSeriesElement, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);

	// By default there is always a small conductance to ground to
	// avoid problems with floating nodes.
	Real defaultParallelCond = 1e-6;
	**mParallelCond = (**mParallelCond > 0) ? **mParallelCond : defaultParallelCond;

	// Create parallel sub components
	mSubParallelResistor0 = std::make_shared<DP::Ph1::Resistor>(**mName + "_con0", mLogLevel);
	mSubParallelResistor0->setParameters(2. / **mParallelCond);
	mSubParallelResistor0->connect(SimNode::List{ SimNode::GND, mTerminals[0]->node() });
	mSubParallelResistor0->initialize(mFrequencies);
	mSubParallelResistor0->initializeFromNodesAndTerminals(frequency);
	addMNASubComponent(mSubParallelResistor0, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, false);

	mSubParallelResistor1 = std::make_shared<DP::Ph1::Resistor>(**mName + "_con1", mLogLevel);
	mSubParallelResistor1->setParameters(2. / **mParallelCond);
	mSubParallelResistor1->connect(SimNode::List{ SimNode::GND, mTerminals[1]->node() });
	mSubParallelResistor1->initialize(mFrequencies);
	mSubParallelResistor1->initializeFromNodesAndTerminals(frequency);
	addMNASubComponent(mSubParallelResistor1, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, false);


	if (**mParallelCap >= 0) {
		mSubParallelCapacitor0 = std::make_shared<DP::Ph1::Capacitor>(**mName + "_cap0", mLogLevel);
		mSubParallelCapacitor0->setParameters(**mParallelCap / 2.);
		mSubParallelCapacitor0->connect(SimNode::List{ SimNode::GND, mTerminals[0]->node() });
		mSubParallelCapacitor0->initialize(mFrequencies);
		mSubParallelCapacitor0->initializeFromNodesAndTerminals(frequency);
		addMNASubComponent(mSubParallelCapacitor0, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);

		mSubParallelCapacitor1 = std::make_shared<DP::Ph1::Capacitor>(**mName + "_cap1", mLogLevel);
		mSubParallelCapacitor1->setParameters(**mParallelCap / 2.);
		mSubParallelCapacitor1->connect(SimNode::List{ SimNode::GND, mTerminals[1]->node() });
		mSubParallelCapacitor1->initialize(mFrequencies);
		mSubParallelCapacitor1->initializeFromNodesAndTerminals(frequency);
		addMNASubComponent(mSubParallelCapacitor1, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);
	}

	SPDLOG_LOGGER_INFO(mSLog, 
		"\n--- Initialization from powerflow ---"
		"\nVoltage across: {:s}"
		"\nCurrent: {:s}"
		"\nTerminal 0 voltage: {:s}"
		"\nTerminal 1 voltage: {:s}"
		"\n--- Initialization from powerflow finished ---",
		Logger::phasorToString((**mIntfVoltage)(0,0)),
		Logger::phasorToString((**mIntfCurrent)(0,0)),
		Logger::phasorToString(initialSingleVoltage(0)),
		Logger::phasorToString(initialSingleVoltage(1)));
}

void DP::Ph1::PiLine::mnaParentAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) {
	// add pre-step dependencies of component itself
	prevStepDependencies.push_back(mIntfCurrent);
	prevStepDependencies.push_back(mIntfVoltage);
	modifiedAttributes.push_back(mRightVector);
}

void DP::Ph1::PiLine::mnaParentPreStep(Real time, Int timeStepCount) {
	// pre-step of component itself
	this->mnaApplyRightSideVectorStamp(**mRightVector);
}

void DP::Ph1::PiLine::mnaParentAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector) {
	// add post-step dependencies of component itself
	attributeDependencies.push_back(leftVector);
	modifiedAttributes.push_back(mIntfVoltage);
	modifiedAttributes.push_back(mIntfCurrent);
}

void DP::Ph1::PiLine::mnaParentPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
	// post-step of component itself
	this->mnaUpdateVoltage(**leftVector);
	this->mnaUpdateCurrent(**leftVector);
}

void DP::Ph1::PiLine::mnaCompUpdateVoltage(const Matrix& leftVector) {
	(**mIntfVoltage)(0, 0) = 0;
	if (terminalNotGrounded(1))
		(**mIntfVoltage)(0,0) = Math::complexFromVectorElement(leftVector, matrixNodeIndex(1));
	if (terminalNotGrounded(0))
		(**mIntfVoltage)(0,0) = (**mIntfVoltage)(0,0) - Math::complexFromVectorElement(leftVector, matrixNodeIndex(0));
}

void DP::Ph1::PiLine::mnaCompUpdateCurrent(const Matrix& leftVector) {
	(**mIntfCurrent)(0,0) = mSubSeriesElement->intfCurrent()(0, 0);
}

MNAInterface::List DP::Ph1::PiLine::mnaTearGroundComponents() {
	MNAInterface::List gndComponents;

	gndComponents.push_back(mSubParallelResistor0);
	gndComponents.push_back(mSubParallelResistor1);

	if (**mParallelCap >= 0) {
		gndComponents.push_back(mSubParallelCapacitor0);
		gndComponents.push_back(mSubParallelCapacitor1);
	}

	return gndComponents;
}

void DP::Ph1::PiLine::mnaTearInitialize(Real omega, Real timeStep) {
	mSubSeriesElement->mnaTearSetIdx(mTearIdx);
	mSubSeriesElement->mnaTearInitialize(omega, timeStep);
}

void DP::Ph1::PiLine::mnaTearApplyMatrixStamp(SparseMatrixRow& tearMatrix) {
	mSubSeriesElement->mnaTearApplyMatrixStamp(tearMatrix);
}

void DP::Ph1::PiLine::mnaTearApplyVoltageStamp(Matrix& voltageVector) {
	mSubSeriesElement->mnaTearApplyVoltageStamp(voltageVector);
}

void DP::Ph1::PiLine::mnaTearPostStep(Complex voltage, Complex current) {
	mSubSeriesElement->mnaTearPostStep(voltage - current * **mSeriesRes, current);
}
