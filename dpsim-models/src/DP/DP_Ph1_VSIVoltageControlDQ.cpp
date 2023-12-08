/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/DP/DP_Ph1_VSIVoltageControlDQ.h>

using namespace CPS;


DP::Ph1::VSIVoltageControlDQ::VSIVoltageControlDQ(String uid, String name, Logger::Level logLevel, Bool withTrafo) :
	CompositePowerComp<Complex>(uid, name, true, true, logLevel),
	VSIVoltageSourceInverterDQ(this->mSLog, mAttributes) {
	
	setTerminalNumber(1);
	mWithConnectionTransformer = withTrafo;
	if (mWithConnectionTransformer)
		setVirtualNodeNumber(3);
	else
		setVirtualNodeNumber(2);
	
	**mIntfVoltage = MatrixComp::Zero(1, 1);
	**mIntfCurrent = MatrixComp::Zero(1, 1);
}

void DP::Ph1::VSIVoltageControlDQ::createSubComponents() {

	// create electrical subcomponents
	mSubFilterRL = DP::Ph1::ResIndSeries::make(**mName + "_FilterRL", mLogLevel);
	mSubCapacitorF = DP::Ph1::Capacitor::make(**mName + "_capF", mLogLevel);
	mSubCtrledVoltageSource = DP::Ph1::VoltageSource::make(**mName + "_src", mLogLevel);
	addMNASubComponent(mSubFilterRL, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);
	addMNASubComponent(mSubCapacitorF, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);
	addMNASubComponent(mSubCtrledVoltageSource, MNA_SUBCOMP_TASK_ORDER::NO_TASK, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);
	if (mWithConnectionTransformer) {
		mConnectionTransformer = DP::Ph1::Transformer::make(**mName + "_trans", **mName + "_trans", mLogLevel);
		addMNASubComponent(mConnectionTransformer, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);
	}

	// set parameters of electrical subcomponents
	mSubFilterRL->setParameters(mRf, mLf);
	mSubCapacitorF->setParameters(mCf);

	if (mWithConnectionTransformer)
		mConnectionTransformer->connect({ mTerminals[0]->node(), mVirtualNodes[2]});

	// Create control sub components
	mVCO = Signal::VCO::make(**mName + "_VCO", mLogLevel);
	mVoltageControllerVSI = Signal::VoltageControllerVSI::make(**mName + "_VoltageControllerVSI", mLogLevel);

	// VCO
	mVCO->mInputRef->setReference(mOmegaN);
	mVCOOutput->setReference(mVCO->mOutputCurr);

	// Sub voltage source
	mVs->setReference(mSubCtrledVoltageSource->mIntfVoltage);

	// Voltage controller
	// input references
	mVoltageControllerVSI->mVc_d->setReference(mVcd);
	mVoltageControllerVSI->mVc_q->setReference(mVcq);
	mVoltageControllerVSI->mIrc_d->setReference(mIrcd);
	mVoltageControllerVSI->mIrc_q->setReference(mIrcq);

	// input, state and output vector for logging
	mVoltagectrlInputs->setReference(mVoltageControllerVSI->mInputCurr);
	mVoltagectrlStates->setReference(mVoltageControllerVSI->mStateCurr);
	mVoltagectrlOutputs->setReference(mVoltageControllerVSI->mOutputCurr);

	// set controller parameters
	mVCO->setParameters(**mOmegaN);
	mVoltageControllerVSI->setControllerParameters(mKpVoltageCtrl, mKiVoltageCtrl, mKpCurrCtrl, mKiCurrCtrl, mOmegaVSI);

}

void DP::Ph1::VSIVoltageControlDQ::initializeFromNodesAndTerminals(Real frequency) {
	// terminal powers in consumer system -> convert to generator system
	**mActivePower = -terminal(0)->singlePower().real();
	**mReactivePower = -terminal(0)->singlePower().imag();

	// set initial interface quantities --> Current flowing into the inverter is positive
	(**mIntfVoltage)(0, 0) = initialSingleVoltage(0);
	(**mIntfCurrent)(0, 0) = -std::conj(Complex(**mActivePower, **mReactivePower) / (**mIntfVoltage)(0,0));

	//
	Complex filterInterfaceInitialVoltage = (**mIntfVoltage)(0, 0);
	Complex filterInterfaceInitialCurrent = (**mIntfCurrent)(0, 0);
	if (mWithConnectionTransformer) {
		// calculate quantities of low voltage side of transformer (being the interface quantities of the filter)
		filterInterfaceInitialVoltage = ((**mIntfVoltage)(0, 0) - Complex(mTransformerResistance, mTransformerInductance * mOmegaNom) * (**mIntfCurrent)(0, 0)) / Complex(mTransformerRatioAbs, mTransformerRatioPhase);
		filterInterfaceInitialCurrent = (**mIntfCurrent)(0, 0) * Complex(mTransformerRatioAbs, mTransformerRatioPhase);		
	}

	// derive initialization quantities of filter
	Complex vcInit = filterInterfaceInitialVoltage - filterInterfaceInitialCurrent * mRc;
	Complex icfInit = vcInit * Complex(0., mOmegaNom * mCf);
	Complex vsInit = vcInit - (filterInterfaceInitialCurrent - icfInit) * Complex(mRf, mOmegaNom * mLf);
	
	// initialize voltage of virtual nodes
	mVirtualNodes[0]->setInitialVoltage(vsInit);
	mVirtualNodes[1]->setInitialVoltage(vcInit);
	if (mWithConnectionTransformer)
		mVirtualNodes[2]->setInitialVoltage(filterInterfaceInitialVoltage);

	// Set parameters electrical subcomponents
	(**mVsref)(0,0) = mVirtualNodes[0]->initialSingleVoltage();

	// Initialize electrical subcomponents
	for (auto subcomp: mSubComponents) {
		subcomp->initialize(mFrequencies);
		subcomp->initializeFromNodesAndTerminals(frequency);
	}

	// Set references VCO 
    mVCO->mInputRef->setReference(mOmega);
	mTheta->setReference(mVCO->mOutputRef);

	// Initialie voltage controller variables
	Real init_theta;
	Complex vcdq, ircdq;
	if (mWithConnectionTransformer) {
		init_theta = std::arg(mVirtualNodes[3]->initialSingleVoltage());
		vcdq = Math::rotatingFrame2to1(mVirtualNodes[3]->initialSingleVoltage(), init_theta, **mThetaSys);
		ircdq = Math::rotatingFrame2to1(-1. * (**mSubResistorF->mIntfCurrent)(0, 0), init_theta, **mThetaSys);
	} else {
		init_theta = std::arg(mVirtualNodes[2]->initialSingleVoltage());
		vcdq = Math::rotatingFrame2to1(mVirtualNodes[2]->initialSingleVoltage(), init_theta, **mThetaSys);
		ircdq = Math::rotatingFrame2to1(-(**mSubResistorF->mIntfCurrent)(0, 0), init_theta, **mThetaSys);
	}
	**mVcap_d = vcdq.real();
	**mVcap_q = vcdq.imag();
	**mIfilter_d = ircdq.real();
	**mIfilter_q = ircdq.imag();
	
	// Initialie voltage controller variables
	**mThetaSys = 0.0;
	**mThetaInv = std::arg(mTerminals[0]->initialSingleVoltage());
	Complex Vcap = Math::rotatingFrame2to1(mTerminals[0]->initialSingleVoltage(), **mThetaInv, **mThetaSys);
	**mVcap_d = Vcap.real();
	**mVcap_q = Vcap.imag();
	Complex Ifilter = Math::rotatingFrame2to1((**mSubFilterRL->mIntfCurrent)(0, 0), **mThetaInv, **mThetaSys);
	**mIfilter_d = Ifilter.real();
	**mIfilter_q = Ifilter.imag();
	this->initializeControllerStates();

	Matrix matrixInputInit = Matrix::Zero(3,1);
	Matrix matrixStateInit = Matrix::Zero(1,1);
	Matrix matrixOutputInit = Matrix::Zero(1,1);

	// Droop and VCO input
	// Input: [PowerInst, PowerSet, omegaNom] //State: [omega] // Output: [omega]
	if (mWithDroop)
		mDroop->setInitialStateValues(matrixInputInit, matrixStateInit, matrixOutputInit);
	// Input: [OmegaSet] //State: [theta] // Output: [theta]
	//mVCO->setInitialValues(mOmegaNom, init_theta, init_theta);

	// initialize states
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
		**mVcap_d, **mVcap_q,
		Logger::phasorToString((**mIntfCurrent)(0, 0)),
		 **mIfilter_d, **mIfilter_q,
		 (**mVsref)(0,0));
	mSLog->flush();
}

void DP::Ph1::VSIVoltageControlDQ::mnaParentInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
	mTimeStep = timeStep;

	// initialize state space controller
	mVoltageControllerVSI->initializeStateSpaceModel(omega, timeStep, leftVector);
	mVCO->setSimulationParameters(timeStep);

	// TODO: these are actually no MNA tasks
	mMnaTasks.push_back(std::make_shared<ControlPreStep>(*this));
	mMnaTasks.push_back(std::make_shared<ControlStep>(*this));
}

void DP::Ph1::VSIVoltageControlDQ::addControlPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) {
	// add pre-step dependencies of subcomponents
	mVCO->signalAddPreStepDependencies(prevStepDependencies, attributeDependencies, modifiedAttributes);
	mVoltageControllerVSI->signalAddPreStepDependencies(prevStepDependencies, attributeDependencies, modifiedAttributes);
}

void DP::Ph1::VSIVoltageControlDQ::controlPreStep(Real time, Int timeStepCount) {
	// add pre-step of subcomponents
	mVCO->signalPreStep(time, timeStepCount);
	mVoltageControllerVSI->signalPreStep(time, timeStepCount);
}

void DP::Ph1::VSIVoltageControlDQ::addControlStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) {
	// add step dependencies of subcomponents
	mVCO->signalAddStepDependencies(prevStepDependencies, attributeDependencies, modifiedAttributes);
	mVoltageControllerVSI->signalAddStepDependencies(prevStepDependencies, attributeDependencies, modifiedAttributes);
	// add step dependencies of component itself
	attributeDependencies.push_back(mIntfCurrent);
	attributeDependencies.push_back(mIntfVoltage);
	modifiedAttributes.push_back(mVsref);
}

void DP::Ph1::VSIVoltageControlDQ::controlStep(Real time, Int timeStepCount) {
	// Transformation interface forward
	Complex vcdq, ircdq;

	if(mWithConnectionTransformer) {
		vcdq = Math::rotatingFrame2to1(mVirtualNodes[3]->singleVoltage(), (**mVCO->mOutputPrev)(0,0), mThetaN);
		ircdq = Math::rotatingFrame2to1(-1. * (**mSubResistorC->mIntfCurrent)(0, 0), (**mVCO->mOutputPrev)(0,0), mThetaN);
	} else {
		vcdq = Math::rotatingFrame2to1(mVirtualNodes[2]->singleVoltage(), (**mVCO->mOutputPrev)(0,0), mThetaN);
		ircdq = Math::rotatingFrame2to1(-1. * (**mSubResistorC->mIntfCurrent)(0, 0), (**mVCO->mOutputPrev)(0,0), mThetaN);
	}

	**mVcd = vcdq.real();
	**mVcq = vcdq.imag();
	**mIrcd = ircdq.real();
	**mIrcq = ircdq.imag();

	// add step of subcomponents
	mVCO->signalStep(time, timeStepCount);
	mVoltageControllerVSI->signalStep(time, timeStepCount);

	// Transformation interface backward
	(**mVsref)(0,0) = Math::rotatingFrame2to1(Complex(mVoltageControllerVSI->attributeTyped<Matrix>("output_curr")->get()(0, 0), mVoltageControllerVSI->attributeTyped<Matrix>("output_curr")->get()(1, 0)), mThetaN, (**mVCO->mOutputPrev)(0,0));

	// Update nominal system angle
	mThetaN = mThetaN + mTimeStep * **mOmegaN;
}

void DP::Ph1::VSIVoltageControlDQ::mnaParentAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) {
	prevStepDependencies.push_back(mVsref);
	prevStepDependencies.push_back(mIntfCurrent);
	prevStepDependencies.push_back(mIntfVoltage);
	attributeDependencies.push_back(mVoltageControllerVSI->attributeTyped<Matrix>("output_prev"));
	attributeDependencies.push_back(mVCO->attributeTyped<Matrix>("output_prev"));
	modifiedAttributes.push_back(mRightVector);
}

void DP::Ph1::VSIVoltageControlDQ::mnaParentPreStep(Real time, Int timeStepCount) {
	// pre-step of subcomponents - controlled source
	if (mWithControl)
		**mSubCtrledVoltageSource->mVoltageRef = (**mVsref)(0,0);

	std::dynamic_pointer_cast<MNAInterface>(mSubCtrledVoltageSource)->mnaPreStep(time, timeStepCount);
	// pre-step of component itself
	mnaApplyRightSideVectorStamp(**mRightVector);
}

void DP::Ph1::VSIVoltageControlDQ::mnaParentAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector) {
	attributeDependencies.push_back(leftVector);
	modifiedAttributes.push_back(mIntfVoltage);
	modifiedAttributes.push_back(mIntfCurrent);
}

void DP::Ph1::VSIVoltageControlDQ::mnaParentPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
	mnaCompUpdateCurrent(**leftVector);
	mnaCompUpdateVoltage(**leftVector);
}

void DP::Ph1::VSIVoltageControlDQ::mnaCompUpdateCurrent(const Matrix& leftvector) {
	if (mWithConnectionTransformer)
		**mIntfCurrent = mConnectionTransformer->mIntfCurrent->get();
	else
		**mIntfCurrent = mSubResistorC->mIntfCurrent->get();
}

void DP::Ph1::VSIVoltageControlDQ::mnaCompUpdateVoltage(const Matrix& leftVector) {
	for (auto virtualNode : mVirtualNodes)
		virtualNode->mnaUpdateVoltage(leftVector);
	(**mIntfVoltage)(0,0) = Math::complexFromVectorElement(leftVector, matrixNodeIndex(0));
}
