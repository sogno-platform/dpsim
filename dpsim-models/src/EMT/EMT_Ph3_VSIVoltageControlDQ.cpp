/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/EMT/EMT_Ph3_VSIVoltageControlDQ.h>

using namespace CPS;

EMT::Ph3::VSIVoltageControlDQ::VSIVoltageControlDQ(String uid, String name, Logger::Level logLevel, Bool withTrafo) :
	CompositePowerComp<Real>(uid, name, true, true, logLevel),
	VSIVoltageSourceInverterDQ(this->mSLog, mAttributes, withTrafo) {
	
	mPhaseType = PhaseType::ABC;

	setTerminalNumber(1);
	if (mWithConnectionTransformer)
		setVirtualNodeNumber(3);
	else
		setVirtualNodeNumber(2);

	**mIntfVoltage = Matrix::Zero(3, 1);
	**mIntfCurrent = Matrix::Zero(3, 1);
}	

void EMT::Ph3::VSIVoltageControlDQ::createSubComponents() {
	this->updateMatrixNodeIndices();
	
	// create electrical subcomponents
	mSubCtrledVoltageSource = EMT::Ph3::VoltageSource::make(**mName + "_src", mLogLevel);
	mSubFilterRL = EMT::Ph3::ResIndSeries::make(**mName + "_FilterRL", mLogLevel);
	mSubCapacitorF = EMT::Ph3::Capacitor::make(**mName + "_CapF", mLogLevel);
	// TODO: ADD IF FOR INTERFACE RESISTOR?
	mSubResistorC = EMT::Ph3::Resistor::make(**mName + "_ResC", mLogLevel);
	addMNASubComponent(mSubCtrledVoltageSource, MNA_SUBCOMP_TASK_ORDER::NO_TASK, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);
	addMNASubComponent(mSubFilterRL, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);
	addMNASubComponent(mSubCapacitorF, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);
	addMNASubComponent(mSubResistorC, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);
	if (mWithConnectionTransformer) {
		mConnectionTransformer = EMT::Ph3::Transformer::make(**mName + "_trans", **mName + "_trans", mLogLevel);
		addMNASubComponent(mConnectionTransformer, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);
	}

	// set parameters of electrical subcomponents
	mSubFilterRL->setParameters(Math::singlePhaseParameterToThreePhase(mRf), Math::singlePhaseParameterToThreePhase(mLf));
	mSubCapacitorF->setParameters(Math::singlePhaseParameterToThreePhase(mCf));
	mSubResistorC->setParameters(Math::singlePhaseParameterToThreePhase(mRc));

	// Connect sub componetes
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

void EMT::Ph3::VSIVoltageControlDQ::initializeFromNodesAndTerminals(Real frequency) {
	// terminal powers in consumer system -> convert to generator system
	**mPower = -terminal(0)->singlePower();

	// set initial interface quantities --> Current flowing into the inverter is positive
	Complex intfVoltageComplex =  initialSingleVoltage(0);
	Complex intfCurrentComplex = std::conj(**mPower / intfVoltageComplex);
	**mIntfVoltage = Math::singlePhaseVariableToThreePhase(RMS3PH_TO_PEAK1PH * intfVoltageComplex).real();
	**mIntfCurrent = Math::singlePhaseVariableToThreePhase(RMS3PH_TO_PEAK1PH * intfCurrentComplex).real();

	// TODO
	/*
	Complex filterInterfaceInitialVoltage = intfVoltageComplex(0, 0);
	Complex filterInterfaceInitialCurrent = intfCurrentComplex(0, 0);
	if (mWithConnectionTransformer) {
		// TODO: CHECK THIS
		// calculate quantities of low voltage side of transformer (being the interface quantities of the filter)
		filterInterfaceInitialVoltage = ((**mIntfVoltage)(0, 0) - Complex(mTransformerResistance, mTransformerInductance * mOmegaNom) * (**mIntfCurrent)(0, 0)) / Complex(mTransformerRatioAbs, mTransformerRatioPhase);
		filterInterfaceInitialCurrent = (**mIntfCurrent)(0, 0) * Complex(mTransformerRatioAbs, mTransformerRatioPhase);		
	}
	*/

	// derive initialization quantities of filter
	/// initial filter capacitor voltage
	Complex vcInit = intfVoltageComplex + intfCurrentComplex * mRc;
	/// initial filter capacitor current 
	Complex icfInit = vcInit * Complex(0., mOmegaNom * mCf);
	/// initial voltage of voltage source
	Complex vsInit = vcInit + (intfCurrentComplex + icfInit) * Complex(mRf, mOmegaNom * mLf);

	SPDLOG_LOGGER_INFO(mSLog, 
		"\n--- Part Initialization from powerflow ---"
		"\nvcInit  = {}"
		"\nicfInit = {}"
		"\nvsInit  = {}"
		"\nInit If  = {}"
		"\nInit Vfilter  = {}",
		vcInit, icfInit, vsInit, intfCurrentComplex + icfInit, vsInit-vcInit);
	mSLog->flush();

	// initialize voltage of virtual nodes
	mVirtualNodes[0]->setInitialVoltage(vsInit);
	mVirtualNodes[1]->setInitialVoltage(vcInit);
	if (mWithConnectionTransformer)
		mVirtualNodes[2]->setInitialVoltage(intfVoltageComplex);

	// Set parameters electrical subcomponents
	**mVsref = Math::singlePhaseVariableToThreePhase(vsInit).real();

	// Create & Initialize electrical subcomponents
	this->createSubComponents();
	for (auto subcomp: mSubComponents) {
		subcomp->initialize(mFrequencies);
		subcomp->initializeFromNodesAndTerminals(frequency);
	}

	// droop
	**mOmega = mOmegaNom;

	// initialize angles
	**mThetaSys = 0;
	**mThetaInv = std::arg(vcInit);

	// Initialie voltage controller variables
	**mVcap_dq = Math::rotatingFrame2to1(vcInit, **mThetaInv, **mThetaSys);
	**mIfilter_dq = Math::rotatingFrame2to1(intfCurrentComplex + icfInit, **mThetaInv, **mThetaSys);
	**mVsref_dq = Math::rotatingFrame2to1(vsInit, **mThetaInv, **mThetaSys);

	SPDLOG_LOGGER_INFO(mSLog, 
		"\n--- Part Initialization 2 from powerflow ---"
		"\n**mVcap_dq  = {}"
		"\n**mIfilter_dq = {}"
		"\n**mVsref_dq  = {}",
		**mVcap_dq, **mIfilter_dq, **mVsref_dq);
	mSLog->flush();

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

void EMT::Ph3::VSIVoltageControlDQ::mnaParentInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
	this->updateMatrixNodeIndices();
	mTimeStep = timeStep;
	if (mWithControl)
		mVSIController->initialize(**mVsref_dq, **mVcap_dq, **mIfilter_dq, mTimeStep);
}

void EMT::Ph3::VSIVoltageControlDQ::mnaParentAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) {
	modifiedAttributes.push_back(mRightVector);
	prevStepDependencies.push_back(mIntfVoltage);
}

void EMT::Ph3::VSIVoltageControlDQ::mnaParentPreStep(Real time, Int timeStepCount) {
	// Transformation interface forward
	**mVcap_dq = parkTransformPowerInvariant(**mThetaInv, **mSubCapacitorF->mIntfVoltage);
	//**mIfilter_dq = Math::rotatingFrame2to1((**mSubFilterRL->mIntfCurrent)(0, 0), **mThetaInv, **mThetaSys);
	**mIfilter_dq = parkTransformPowerInvariant(**mThetaInv, **mSubFilterRL->mIntfCurrent);

	// TODO: droop
	//if (mWithDroop)
	//	mDroop->signalStep(time, timeStepCount);

	//  VCO Step
	**mThetaInv = **mThetaInv + mTimeStep * **mOmega;

	// Update nominal system angle
	**mThetaSys = **mThetaSys + mTimeStep * mOmegaNom;

	//
	if (mWithControl)
		**mVsref_dq = mVSIController->step(**mVcap_dq, **mIfilter_dq);

	// Transformation interface backward
	**mVsref = inverseParkTransformPowerInvariant(**mThetaInv, **mVsref_dq) * sqrt(3/2);

	// pre-step of subcomponents - controlled source
	if (mWithControl)
		**mSubCtrledVoltageSource->mVoltageRef = **mVsref * PEAK1PH_TO_RMS3PH;

	std::dynamic_pointer_cast<MNAInterface>(mSubCtrledVoltageSource)->mnaPreStep(time, timeStepCount);
	
	// pre-step of component itself
	mnaApplyRightSideVectorStamp(**mRightVector);
}

void EMT::Ph3::VSIVoltageControlDQ::mnaParentAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector) {
	attributeDependencies.push_back(leftVector);
	modifiedAttributes.push_back(mIntfVoltage);
	modifiedAttributes.push_back(mIntfCurrent);
}

void EMT::Ph3::VSIVoltageControlDQ::mnaParentPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
	mnaCompUpdateCurrent(**leftVector);
	mnaCompUpdateVoltage(**leftVector);
}

void EMT::Ph3::VSIVoltageControlDQ::mnaCompUpdateCurrent(const Matrix& leftvector) {
	if (mWithConnectionTransformer)
		**mIntfCurrent = mConnectionTransformer->mIntfCurrent->get();
	else
		**mIntfCurrent = mSubResistorC->mIntfCurrent->get();
}

void EMT::Ph3::VSIVoltageControlDQ::mnaCompUpdateVoltage(const Matrix& leftVector) {
	for (auto virtualNode : mVirtualNodes)
		virtualNode->mnaUpdateVoltage(leftVector);
	(**mIntfVoltage)(0,0) = Math::realFromVectorElement(leftVector, matrixNodeIndex(0,0));
	(**mIntfVoltage)(1,0) = Math::realFromVectorElement(leftVector, matrixNodeIndex(0,1));
	(**mIntfVoltage)(2,0) = Math::realFromVectorElement(leftVector, matrixNodeIndex(0,2));

	// Update Power
	//**mPower = (**mIntfVoltage)(0,0) * std::conj((**mIntfCurrent)(0,0));
}

Complex EMT::Ph3::VSIVoltageControlDQ::parkTransformPowerInvariant(Real theta, const Matrix &fabc) {
	// Calculates fdq = Tdq * fabc
	// Assumes that d-axis starts aligned with phase a
	Matrix Tdq = getParkTransformMatrixPowerInvariant(theta);
	Matrix dqvector = Tdq * fabc;

	return Complex(dqvector(0,0), dqvector(1,0));
}

Matrix EMT::Ph3::VSIVoltageControlDQ::getParkTransformMatrixPowerInvariant(Real theta) {
	// Return park matrix for theta
	// Assumes that d-axis starts aligned with phase a
	Matrix Tdq = Matrix::Zero(2, 3);
	Real k = sqrt(2. / 3.);
	Tdq <<
		k * cos(theta), k * cos(theta - 2. * M_PI / 3.), k * cos(theta + 2. * M_PI / 3.),
		-k * sin(theta), -k * sin(theta - 2. * M_PI / 3.), -k * sin(theta + 2. * M_PI / 3.);
	return Tdq;
}

Matrix EMT::Ph3::VSIVoltageControlDQ::inverseParkTransformPowerInvariant(Real theta, const Complex &fdq) {
	// Calculates fabc = Tabc * fdq
	// with d-axis starts aligned with phase a
	Matrix Fdq = Matrix::Zero(2, 1);
	Fdq << fdq.real(), fdq.imag();
	Matrix Tabc = getInverseParkTransformMatrixPowerInvariant(theta);

	return Tabc * Fdq;
}


Matrix EMT::Ph3::VSIVoltageControlDQ::getInverseParkTransformMatrixPowerInvariant(Real theta) {
	// Return inverse park matrix for theta
	/// with d-axis starts aligned with phase a
	Matrix Tabc = Matrix::Zero(3, 2);
	Real k = sqrt(2. / 3.);
	Tabc <<
		k * cos(theta), - k * sin(theta),
		k * cos(theta - 2. * M_PI / 3.), - k * sin(theta - 2. * M_PI / 3.),
		k * cos(theta + 2. * M_PI / 3.), - k * sin(theta + 2. * M_PI / 3.);
	return Tabc;
}