/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/SP/SP_Ph1_VSIVoltageControlDQ.h>

using namespace CPS;

SP::Ph1::VSIVoltageControlDQ::VSIVoltageControlDQ(String uid, String name, Logger::Level logLevel,
	Bool modelAsCurrentSource, Bool withInterfaceResistor) :
	CompositePowerComp<Complex>(uid, name, true, true, logLevel),
	VSIVoltageSourceInverterDQ<Complex>(this->mSLog, mAttributes, modelAsCurrentSource, withInterfaceResistor) {
	
	setTerminalNumber(1);
	setVirtualNodeNumber(this->determineNumberOfVirtualNodes());
	
	**mIntfVoltage = MatrixComp::Zero(1, 1);
	**mIntfCurrent = MatrixComp::Zero(1, 1);
}

void SP::Ph1::VSIVoltageControlDQ::createSubComponents() {
	// voltage source
	if (!mModelAsCurrentSource) {
		mSubCtrledVoltageSource = SP::Ph1::VoltageSource::make(**mName + "_src", mLogLevel);
		addMNASubComponent(mSubCtrledVoltageSource, MNA_SUBCOMP_TASK_ORDER::NO_TASK, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);
	}

	// RL Element as part of the LC filter
	mSubFilterRL = SP::Ph1::ResIndSeries::make(**mName + "_FilterRL", mLogLevel);
	mSubFilterRL->setParameters(mRf, mLf);
	addMNASubComponent(mSubFilterRL, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, false);

	// Capacitor as part of the LC filter
	mSubCapacitorF = SP::Ph1::Capacitor::make(**mName + "_CapF", mLogLevel);
	mSubCapacitorF->setParameters(mCf);
	addMNASubComponent(mSubCapacitorF, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, false);

	// optinal: interface resistor
	if (mWithInterfaceResistor) {
		mSubResistorC = SP::Ph1::Resistor::make(**mName + "_ResC", mLogLevel);
		mSubCapacitorF->setParameters(mCf);
		addMNASubComponent(mSubResistorC, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, false);
	}
}

void SP::Ph1::VSIVoltageControlDQ::connectSubComponents() {
	// TODO: COULD WE MOVE THIS FUNCTION TO THE BASE CLASS?
	if (!mModelAsCurrentSource)
		mSubCtrledVoltageSource->connect({ SimNode::GND, mVirtualNodes[0] });
	if (mWithInterfaceResistor) {
		mSubFilterRL->connect({ mVirtualNodes[1], mVirtualNodes[0] });
		mSubCapacitorF->connect({ SimNode::GND, mVirtualNodes[1] });
		mSubResistorC->connect({ mTerminals[0]->node(), mVirtualNodes[1]});
	} else {
		// without interface resistor
		mSubFilterRL->connect({ mTerminals[0]->node(), mVirtualNodes[0] });
		mSubCapacitorF->connect({ SimNode::GND, mTerminals[0]->node() });
	}
}

void SP::Ph1::VSIVoltageControlDQ::initializeFromNodesAndTerminals(Real frequency) {
	// terminal powers in consumer system -> convert to generator system
	**mPower = -terminal(0)->singlePower();

	// set initial interface quantities --> Current flowing into the inverter is positive
	(**mIntfVoltage)(0, 0) = initialSingleVoltage(0);
	(**mIntfCurrent)(0, 0) = std::conj(**mPower / (**mIntfVoltage)(0,0));

	//
	Complex filterInterfaceInitialVoltage = (**mIntfVoltage)(0, 0);
	Complex filterInterfaceInitialCurrent = (**mIntfCurrent)(0, 0);

	// derive initialization quantities of filter
	/// initial filter capacitor voltage
	Complex vcInit;
	if (mWithInterfaceResistor)
		vcInit = filterInterfaceInitialVoltage + filterInterfaceInitialCurrent * mRc;
	else
		vcInit = (**mIntfVoltage)(0, 0);
	/// initial filter capacitor current 
	Complex icfInit = vcInit * Complex(0., mOmegaNom * mCf);
	/// initial voltage of voltage source
	Complex vsInit = vcInit + (filterInterfaceInitialCurrent + icfInit) * Complex(mRf, mOmegaNom * mLf);

	// initialize voltage of virtual nodes
	mVirtualNodes[0]->setInitialVoltage(vsInit);
	if (mWithInterfaceResistor) {
		// filter capacitor is connected to mVirtualNodes[1], the second
		// node of the interface resistor is mTerminals[0]
		mVirtualNodes[1]->setInitialVoltage(vcInit);
	}

	// Set parameters voltage source
	(**mSourceValue)(0,0) = vsInit;

	// Connect & Initialize electrical subcomponents
	this->connectSubComponents();
	for (auto subcomp: mSubComponents) {
		subcomp->initialize(mFrequencies);
		subcomp->initializeFromNodesAndTerminals(frequency);
	}

	// TODO: droop
	**mOmega = mOmegaNom;

	// initialize angles
	**mThetaSys = 0;
	**mThetaInv = std::arg((**mSubCapacitorF->mIntfVoltage)(0,0));

	// Initialie voltage controller variables
	**mVcap_dq = Math::rotatingFrame2to1((**mSubCapacitorF->mIntfVoltage)(0,0), **mThetaInv, **mThetaSys);
	if (mWithInterfaceResistor)
		// TODO: CHECK!
		**mIfilter_dq = Math::rotatingFrame2to1((**mSubResistorC->mIntfCurrent)(0, 0), **mThetaInv, **mThetaSys);
	else
		**mIfilter_dq = Math::rotatingFrame2to1((**mSubFilterRL->mIntfCurrent)(0, 0), **mThetaInv, **mThetaSys);
	**mSourceValue_dq = Math::rotatingFrame2to1((**mSourceValue)(0,0), **mThetaInv, **mThetaSys);

	SPDLOG_LOGGER_INFO(mSLog, 
		"\n--- Initialization from powerflow ---"
		"\nTerminal 0 connected to {} = sim node {}"
		"\nInverter terminal voltage: {}[V]"
		"\nInverter d-axis terminal voltage: {}[V]"
		"\nInverter q-axis terminal voltage: {}[V]"
		"\nInverter output current: {}[A]"
		"\nInverter d-axis filter current: {}[A]"
		"\nInverter q-axis filter current: {}[A]"
		"\nInitial voltage source: {}[V]",
		mTerminals[0]->node()->name(), mTerminals[0]->node()->matrixNodeIndex(),
		Logger::phasorToString((**mIntfVoltage)(0, 0)),
		(**mVcap_dq).real(), (**mVcap_dq).imag(),
		Logger::phasorToString((**mIntfCurrent)(0, 0)),
		(**mIfilter_dq).real(), (**mIfilter_dq).imag(),
		(**mSourceValue)(0,0));
	mSLog->flush();
}

void SP::Ph1::VSIVoltageControlDQ::mnaParentInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
	this->updateMatrixNodeIndices();
	mTimeStep = timeStep;
	if (mWithControl)
		mVSIController->initialize(**mSourceValue_dq, **mVcap_dq, **mIfilter_dq, mTimeStep, mModelAsCurrentSource);
}

void SP::Ph1::VSIVoltageControlDQ::mnaParentAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) {
	modifiedAttributes.push_back(mRightVector);
	prevStepDependencies.push_back(mIntfVoltage);
}

void SP::Ph1::VSIVoltageControlDQ::mnaParentPreStep(Real time, Int timeStepCount) {
	// Transformation interface forward
	**mVcap_dq = Math::rotatingFrame2to1((**mSubCapacitorF->mIntfVoltage)(0,0), **mThetaInv, **mThetaSys);
	if (mWithInterfaceResistor)
		// TODO: CHECK
		**mIfilter_dq = Math::rotatingFrame2to1((**mSubResistorC->mIntfCurrent)(0, 0), **mThetaInv, **mThetaSys);
	else
		**mIfilter_dq = Math::rotatingFrame2to1((**mSubFilterRL->mIntfCurrent)(0, 0), **mThetaInv, **mThetaSys);

	// TODO: droop
	//if (mWithDroop)
	//	mDroop->signalStep(time, timeStepCount);

	//  VCO Step
	**mThetaInv = **mThetaInv + mTimeStep * **mOmega;

	// Update nominal system angle
	**mThetaSys = **mThetaSys + mTimeStep * mOmegaNom;

	//
	//mWithControl = false;
	//**mSourceValue_dq = Math::rotatingFrame2to1((**mSubFilterRL->mIntfCurrent)(0, 0), **mThetaInv, **mThetaSys); 
	if (mWithControl)
		**mSourceValue_dq = mVSIController->step(**mVcap_dq, **mIfilter_dq);

	// Transformation interface backward
	(**mSourceValue)(0,0) = Math::rotatingFrame2to1(**mSourceValue_dq, **mThetaSys, **mThetaInv);

	mnaApplyRightSideVectorStamp(**mRightVector);
}

void SP::Ph1::VSIVoltageControlDQ::mnaCompApplyRightSideVectorStamp(Matrix& rightVector) {
	// TODO
	Math::setVectorElement(**mRightVector, mVirtualNodes[0]->matrixNodeIndex(), (**mSourceValue)(0,0)); 
}

void SP::Ph1::VSIVoltageControlDQ::mnaParentAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector) {
	attributeDependencies.push_back(leftVector);
	modifiedAttributes.push_back(mIntfVoltage);
	modifiedAttributes.push_back(mIntfCurrent);
}

void SP::Ph1::VSIVoltageControlDQ::mnaParentPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
	mnaCompUpdateCurrent(**leftVector);
	mnaCompUpdateVoltage(**leftVector);
}

void SP::Ph1::VSIVoltageControlDQ::mnaCompUpdateCurrent(const Matrix& leftvector) {
	// TODO: CHECK
	if (mWithInterfaceResistor)
		**mIntfCurrent = mSubResistorC->mIntfCurrent->get();
	else
		**mIntfCurrent = mSubCapacitorF->mIntfCurrent->get() + mSubFilterRL->mIntfCurrent->get();
}

void SP::Ph1::VSIVoltageControlDQ::mnaCompUpdateVoltage(const Matrix& leftVector) {
	// update voltage of virtual nodes
	for (auto virtualNode : mVirtualNodes)
		// CHECK: Is it really necessary?
		virtualNode->mnaUpdateVoltage(leftVector);

	(**mIntfVoltage)(0,0) = Math::complexFromVectorElement(leftVector, matrixNodeIndex(0));

	// Update Power
	**mPower = (**mIntfVoltage)(0,0) * std::conj((**mIntfCurrent)(0,0));
}
