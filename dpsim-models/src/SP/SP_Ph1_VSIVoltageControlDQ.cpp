/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/SP/SP_Ph1_VSIVoltageControlDQ.h>

using namespace CPS;

SP::Ph1::VSIVoltageControlDQ::VSIVoltageControlDQ(String uid, String name, Logger::Level logLevel, Bool withTrafo) :
	CompositePowerComp<Complex>(uid, name, true, true, logLevel),
	VSIVoltageSourceInverterDQ(this->mSLog, mAttributes, withTrafo) {
	
	setTerminalNumber(1);
	if (mWithConnectionTransformer)
		setVirtualNodeNumber(3);
	else
		setVirtualNodeNumber(2);
	
	**mIntfVoltage = MatrixComp::Zero(1, 1);
	**mIntfCurrent = MatrixComp::Zero(1, 1);
}

void SP::Ph1::VSIVoltageControlDQ::createSubComponents() {
	// create electrical subcomponents
	mSubCtrledVoltageSource = SP::Ph1::VoltageSource::make(**mName + "_src", mLogLevel);
	mSubFilterRL = SP::Ph1::ResIndSeries::make(**mName + "_FilterRL", mLogLevel);
	mSubCapacitorF = SP::Ph1::Capacitor::make(**mName + "_CapF", mLogLevel);
	// TODO: ADD IF FOR INTERFACE RESISTOR?
	mSubResistorC = SP::Ph1::Resistor::make(**mName + "_ResC", mLogLevel);

	addMNASubComponent(mSubCtrledVoltageSource, MNA_SUBCOMP_TASK_ORDER::NO_TASK, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);
	addMNASubComponent(mSubFilterRL, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, false);
	addMNASubComponent(mSubCapacitorF, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, false);
	addMNASubComponent(mSubResistorC, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, false);
	if (mWithConnectionTransformer) {
		mConnectionTransformer = SP::Ph1::Transformer::make(**mName + "_trans", **mName + "_trans", mLogLevel);
		addMNASubComponent(mConnectionTransformer, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);
	}

	// set parameters of electrical subcomponents
	mSubFilterRL->setParameters(mRf, mLf);
	mSubCapacitorF->setParameters(mCf);
	mSubResistorC->setParameters(mRc);
}

void SP::Ph1::VSIVoltageControlDQ::connectSubComponents() {
	mSubCtrledVoltageSource->connect({ SimNode::GND, mVirtualNodes[0] });
	mSubFilterRL->connect({ mVirtualNodes[1], mVirtualNodes[0] });
	mSubCapacitorF->connect({ SimNode::GND, mVirtualNodes[1] });
	if (mWithConnectionTransformer) {
		// TODO: COULD WE REMOVE THE TRANSFORMER?
		mSubResistorC->connect({ mVirtualNodes[2],  mVirtualNodes[1]});
		mConnectionTransformer->connect({ mTerminals[0]->node(), mVirtualNodes[2]});
	} else {
		mSubResistorC->connect({ mTerminals[0]->node(), mVirtualNodes[1]});
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
	if (mWithConnectionTransformer) {
		// calculate quantities of low voltage side of transformer (being the interface quantities of the filter)
		filterInterfaceInitialVoltage = ((**mIntfVoltage)(0, 0) - Complex(mTransformerResistance, mTransformerInductance * mOmegaNom) * (**mIntfCurrent)(0, 0)) / Complex(mTransformerRatioAbs, mTransformerRatioPhase);
		filterInterfaceInitialCurrent = (**mIntfCurrent)(0, 0) * Complex(mTransformerRatioAbs, mTransformerRatioPhase);		
	}

	// derive initialization quantities of filter
	Complex vcInit = filterInterfaceInitialVoltage + filterInterfaceInitialCurrent * mRc;
	Complex icfInit = vcInit * Complex(0., mOmegaNom * mCf);
	Complex vsInit = vcInit + (filterInterfaceInitialCurrent + icfInit) * Complex(mRf, mOmegaNom * mLf);

	// initialize voltage of virtual nodes
	mVirtualNodes[0]->setInitialVoltage(vsInit);
	mVirtualNodes[1]->setInitialVoltage(vcInit);
	if (mWithConnectionTransformer)
		mVirtualNodes[2]->setInitialVoltage(filterInterfaceInitialVoltage);

	// Set parameters electrical subcomponents
	(**mVsref)(0,0) = vsInit;

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
	**mIfilter_dq = Math::rotatingFrame2to1((**mSubFilterRL->mIntfCurrent)(0, 0), **mThetaInv, **mThetaSys);
	**mVsref_dq = Math::rotatingFrame2to1((**mVsref)(0,0), **mThetaInv, **mThetaSys);

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
		(**mVsref)(0,0));
	mSLog->flush();
}

void SP::Ph1::VSIVoltageControlDQ::mnaParentInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
	this->updateMatrixNodeIndices();
	mTimeStep = timeStep;
	if (mWithControl)
		mVSIController->initialize(**mVsref_dq, **mVcap_dq, **mIfilter_dq, mTimeStep);
}

void SP::Ph1::VSIVoltageControlDQ::mnaParentAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) {
	modifiedAttributes.push_back(mRightVector);
	prevStepDependencies.push_back(mIntfVoltage);
}

void SP::Ph1::VSIVoltageControlDQ::mnaParentPreStep(Real time, Int timeStepCount) {
	// Transformation interface forward
	**mVcap_dq = Math::rotatingFrame2to1((**mSubCapacitorF->mIntfVoltage)(0,0), **mThetaInv, **mThetaSys);
	**mIfilter_dq = Math::rotatingFrame2to1((**mSubFilterRL->mIntfCurrent)(0, 0), **mThetaInv, **mThetaSys);

	// TODO: droop
	//if (mWithDroop)
	//	mDroop->signalStep(time, timeStepCount);

	//  VCO Step
	**mThetaInv = **mThetaInv + mTimeStep * **mOmega;

	// Update nominal system angle
	**mThetaSys = **mThetaSys + mTimeStep * mOmegaNom;

	//
	//if (mWithControl)
	//	**mVsref_dq = mVSIController->step(**mVcap_dq, **mIfilter_dq);

	// Transformation interface backward
	(**mVsref)(0,0) = Math::rotatingFrame2to1(**mVsref_dq, **mThetaSys, **mThetaInv);

	// pre-step of subcomponents - controlled source
	if (mWithControl)
		**mSubCtrledVoltageSource->mVoltageRef = (**mVsref)(0,0);

	std::dynamic_pointer_cast<MNAInterface>(mSubCtrledVoltageSource)->mnaPreStep(time, timeStepCount);
	
	// pre-step of component itself
	//mnaApplyRightSideVectorStamp(**mRightVector);
	mnaCompApplyRightSideVectorStamp(**mRightVector);
	SPDLOG_LOGGER_INFO(mSLog, "rightVector =  {:s}",
		Logger::matrixCompToString(**mRightVector));
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
	if (mWithConnectionTransformer)
		**mIntfCurrent = mConnectionTransformer->mIntfCurrent->get();
	else
		**mIntfCurrent = mSubResistorC->mIntfCurrent->get();
}

void SP::Ph1::VSIVoltageControlDQ::mnaCompUpdateVoltage(const Matrix& leftVector) {
	for (auto virtualNode : mVirtualNodes)
		virtualNode->mnaUpdateVoltage(leftVector);
	(**mIntfVoltage)(0,0) = Math::complexFromVectorElement(leftVector, matrixNodeIndex(0));

	// Update Power
	**mPower = (**mIntfVoltage)(0,0) * std::conj((**mIntfCurrent)(0,0));
}
