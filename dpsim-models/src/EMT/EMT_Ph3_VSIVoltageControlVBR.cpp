/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/EMT/EMT_Ph3_VSIVoltageControlVBR.h>
#include <dpsim-models/Signal/VSIControlType1.h>

using namespace CPS;

EMT::Ph3::VSIVoltageControlVBR::VSIVoltageControlVBR(String uid, String name, Logger::Level logLevel,
													 Bool modelAsCurrentSource) : CompositePowerComp<Real>(uid, name, true, true, logLevel),
																				  VSIVoltageSourceInverterDQ(this->mSLog, mAttributes, modelAsCurrentSource, false)
{

	mPhaseType = PhaseType::ABC;
	setTerminalNumber(1);
	setVirtualNodeNumber(this->determineNumberOfVirtualNodes());

	**mIntfVoltage = Matrix::Zero(3, 1);
	**mIntfCurrent = Matrix::Zero(3, 1);
}

void EMT::Ph3::VSIVoltageControlVBR::createSubComponents()
{
	// Capacitor as part of the LC filter
	mSubCapacitorF = EMT::Ph3::Capacitor::make(**mName + "_CapF", mLogLevel);
	addMNASubComponent(mSubCapacitorF, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);
	mSubCapacitorF->setParameters(Math::singlePhaseParameterToThreePhase(mCf));
}

void EMT::Ph3::VSIVoltageControlVBR::initializeFromNodesAndTerminals(Real frequency)
{
	// terminal powers in consumer system -> convert to generator system
	**mPower = -terminal(0)->singlePower();

	// set initial interface quantities --> Current flowing into the inverter is positive
	Complex intfVoltageComplex = initialSingleVoltage(0);
	Complex intfCurrentComplex = std::conj(**mPower / intfVoltageComplex);
	**mIntfVoltage = Math::singlePhaseVariableToThreePhase(RMS3PH_TO_PEAK1PH * intfVoltageComplex).real();
	**mIntfCurrent = Math::singlePhaseVariableToThreePhase(RMS3PH_TO_PEAK1PH * intfCurrentComplex).real();

	// initialize filter variables and set initial voltage of virtual nodes
	initializeFilterVariables(intfVoltageComplex, intfCurrentComplex, mVirtualNodes);

	// calculate initial source value
	// FIXME: later is mSourceValue used to store the history term
	**mSourceValue = inverseParkTransformPowerInvariant(**mThetaInv, **mSourceValue_dq).real();

	// Connect & Initialize electrical subcomponents
	mSubCapacitorF->connect({SimNode::GND, mTerminals[0]->node()});
	for (auto subcomp : mSubComponents)
	{
		subcomp->initialize(mFrequencies);
		subcomp->initializeFromNodesAndTerminals(frequency);
	}

	// droop
	**mOmega = mOmegaNom;

	/// initialize filter current in dp domain
	mFilterCurrent = inverseParkTransformPowerInvariant(**mThetaInv, **mIfilter_dq);

	SPDLOG_LOGGER_INFO(mSLog,
					   "\n--- Initialization from powerflow ---"
					   "\nTerminal 0 connected to {} = sim node {}"
					   "\nInverter terminal voltage: {}[V]"
					   "\nInverter output current: {}[A]",
					   mTerminals[0]->node()->name(), mTerminals[0]->node()->matrixNodeIndex(),
					   (**mIntfVoltage)(0, 0),
					   (**mIntfCurrent)(0, 0));
	mSLog->flush();
}

void EMT::Ph3::VSIVoltageControlVBR::initVars(Real timeStep)
{
	// Assumption: symmetric R and L matrix
	Real a = timeStep * mRf / (2. * mLf);
	Real b = timeStep / (2. * mLf);

	mEquivCond = b / (1. + a);
	mPrevCurrFac = (1. - a) / (1. + a);

	//
	mEquivCurrent = mEquivCond * ((**mSourceValue).real() - **mIntfVoltage) + mPrevCurrFac * mFilterCurrent;

	// initialize auxiliar
	mA = Matrix::Zero(2, 2);
	mA << std::dynamic_pointer_cast<CPS::Signal::VSIControlType1>(mVSIController)
			  ->mA_VBR,
		0, 0,
		std::dynamic_pointer_cast<CPS::Signal::VSIControlType1>(mVSIController)
			->mA_VBR;

	mE = Matrix::Zero(2, 2);
	mE << std::dynamic_pointer_cast<CPS::Signal::VSIControlType1>(mVSIController)
			  ->mE_VBR,
		0, 0,
		std::dynamic_pointer_cast<CPS::Signal::VSIControlType1>(mVSIController)
			->mE_VBR;
}

void EMT::Ph3::VSIVoltageControlVBR::mnaParentInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector)
{
	mTimeStep = timeStep;
	if (mWithControl)
	{
		mVSIController->initialize(**mSourceValue_dq, **mVcap_dq, **mIfilter_dq, mTimeStep, mModelAsCurrentSource);
		mVSIController->calculateVBRconstants();
	}
	initVars(timeStep);

	// get matrix dimension to properly set variable entries
	// upper left block
	mVariableSystemMatrixEntries.push_back(std::make_pair<UInt, UInt>(mVirtualNodes[0]->matrixNodeIndex(PhaseType::A), mVirtualNodes[0]->matrixNodeIndex(PhaseType::A)));
	mVariableSystemMatrixEntries.push_back(std::make_pair<UInt, UInt>(mVirtualNodes[0]->matrixNodeIndex(PhaseType::A), mVirtualNodes[0]->matrixNodeIndex(PhaseType::B)));
	mVariableSystemMatrixEntries.push_back(std::make_pair<UInt, UInt>(mVirtualNodes[0]->matrixNodeIndex(PhaseType::A), mVirtualNodes[0]->matrixNodeIndex(PhaseType::C)));
	mVariableSystemMatrixEntries.push_back(std::make_pair<UInt, UInt>(mVirtualNodes[0]->matrixNodeIndex(PhaseType::B), mVirtualNodes[0]->matrixNodeIndex(PhaseType::A)));
	mVariableSystemMatrixEntries.push_back(std::make_pair<UInt, UInt>(mVirtualNodes[0]->matrixNodeIndex(PhaseType::B), mVirtualNodes[0]->matrixNodeIndex(PhaseType::B)));
	mVariableSystemMatrixEntries.push_back(std::make_pair<UInt, UInt>(mVirtualNodes[0]->matrixNodeIndex(PhaseType::B), mVirtualNodes[0]->matrixNodeIndex(PhaseType::C)));
	mVariableSystemMatrixEntries.push_back(std::make_pair<UInt, UInt>(mVirtualNodes[0]->matrixNodeIndex(PhaseType::C), mVirtualNodes[0]->matrixNodeIndex(PhaseType::A)));
	mVariableSystemMatrixEntries.push_back(std::make_pair<UInt, UInt>(mVirtualNodes[0]->matrixNodeIndex(PhaseType::C), mVirtualNodes[0]->matrixNodeIndex(PhaseType::B)));
	mVariableSystemMatrixEntries.push_back(std::make_pair<UInt, UInt>(mVirtualNodes[0]->matrixNodeIndex(PhaseType::C), mVirtualNodes[0]->matrixNodeIndex(PhaseType::C)));

	// off diagonal blocks
	mVariableSystemMatrixEntries.push_back(std::make_pair<UInt, UInt>(mVirtualNodes[0]->matrixNodeIndex(PhaseType::A), matrixNodeIndex(0, 0)));
	mVariableSystemMatrixEntries.push_back(std::make_pair<UInt, UInt>(mVirtualNodes[0]->matrixNodeIndex(PhaseType::A), matrixNodeIndex(0, 1)));
	mVariableSystemMatrixEntries.push_back(std::make_pair<UInt, UInt>(mVirtualNodes[0]->matrixNodeIndex(PhaseType::A), matrixNodeIndex(0, 2)));
	mVariableSystemMatrixEntries.push_back(std::make_pair<UInt, UInt>(mVirtualNodes[0]->matrixNodeIndex(PhaseType::B), matrixNodeIndex(0, 0)));
	mVariableSystemMatrixEntries.push_back(std::make_pair<UInt, UInt>(mVirtualNodes[0]->matrixNodeIndex(PhaseType::B), matrixNodeIndex(0, 1)));
	mVariableSystemMatrixEntries.push_back(std::make_pair<UInt, UInt>(mVirtualNodes[0]->matrixNodeIndex(PhaseType::B), matrixNodeIndex(0, 2)));
	mVariableSystemMatrixEntries.push_back(std::make_pair<UInt, UInt>(mVirtualNodes[0]->matrixNodeIndex(PhaseType::C), matrixNodeIndex(0, 0)));
	mVariableSystemMatrixEntries.push_back(std::make_pair<UInt, UInt>(mVirtualNodes[0]->matrixNodeIndex(PhaseType::C), matrixNodeIndex(0, 1)));
	mVariableSystemMatrixEntries.push_back(std::make_pair<UInt, UInt>(mVirtualNodes[0]->matrixNodeIndex(PhaseType::C), matrixNodeIndex(0, 2)));

	SPDLOG_LOGGER_INFO(mSLog, "List of index pairs of varying matrix entries: ");
	for (auto indexPair : mVariableSystemMatrixEntries)
		SPDLOG_LOGGER_INFO(mSLog, "({}, {})", indexPair.first, indexPair.second);
	mSLog->flush();
}

void EMT::Ph3::VSIVoltageControlVBR::calculateResistanceMatrix()
{
	mAMatrix = mDqToABC * mA * mABCToDq * mEquivCond;
	mAMatrix_ = Matrix::Identity(3, 3) + mAMatrix;
	mEMatrix = mDqToABC * mE * mABCToDq;
}

void EMT::Ph3::VSIVoltageControlVBR::mnaParentApplySystemMatrixStamp(
	SparseMatrixRow &systemMatrix)
{
	//
	mABCToDq = getParkTransformMatrixPowerInvariant(**mThetaInv);
	mDqToABC = getInverseParkTransformMatrixPowerInvariant(**mThetaInv);

	// calculate resistance matrix at t=k+1
	calculateResistanceMatrix();

	// Stamp filter current: I = (Vc - Vsource) * Admittance
	Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 0), matrixNodeIndex(0, 0), mEquivCond);
	Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 1), matrixNodeIndex(0, 1), mEquivCond);
	Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 2), matrixNodeIndex(0, 2), mEquivCond);
	Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 0), mVirtualNodes[0]->matrixNodeIndex(PhaseType::A), -mEquivCond);
	Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 1), mVirtualNodes[0]->matrixNodeIndex(PhaseType::B), -mEquivCond);
	Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 2), mVirtualNodes[0]->matrixNodeIndex(PhaseType::C), -mEquivCond);

	// Stamp voltage source equation
	Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::A), mVirtualNodes[0]->matrixNodeIndex(PhaseType::A), mAMatrix_(0, 0));
	Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::A), mVirtualNodes[0]->matrixNodeIndex(PhaseType::B), mAMatrix_(0, 1));
	Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::A), mVirtualNodes[0]->matrixNodeIndex(PhaseType::C), mAMatrix_(0, 2));
	Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::B), mVirtualNodes[0]->matrixNodeIndex(PhaseType::A), mAMatrix_(1, 0));
	Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::B), mVirtualNodes[0]->matrixNodeIndex(PhaseType::B), mAMatrix_(1, 1));
	Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::B), mVirtualNodes[0]->matrixNodeIndex(PhaseType::C), mAMatrix_(1, 2));
	Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::C), mVirtualNodes[0]->matrixNodeIndex(PhaseType::A), mAMatrix_(2, 0));
	Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::C), mVirtualNodes[0]->matrixNodeIndex(PhaseType::B), mAMatrix_(2, 1));
	Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::C), mVirtualNodes[0]->matrixNodeIndex(PhaseType::C), mAMatrix_(2, 2));

	Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::A), matrixNodeIndex(0, 0), -mEMatrix(0, 0) - mAMatrix(0, 0));
	Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::A), matrixNodeIndex(0, 1), -mEMatrix(0, 1) - mAMatrix(0, 1));
	Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::A), matrixNodeIndex(0, 2), -mEMatrix(0, 2) - mAMatrix(0, 2));
	Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::B), matrixNodeIndex(0, 0), -mEMatrix(1, 0) - mAMatrix(1, 0));
	Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::B), matrixNodeIndex(0, 1), -mEMatrix(1, 1) - mAMatrix(1, 1));
	Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::B), matrixNodeIndex(0, 2), -mEMatrix(1, 2) - mAMatrix(1, 2));
	Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::C), matrixNodeIndex(0, 0), -mEMatrix(2, 0) - mAMatrix(2, 0));
	Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::C), matrixNodeIndex(0, 1), -mEMatrix(2, 1) - mAMatrix(2, 1));
	Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::C), matrixNodeIndex(0, 2), -mEMatrix(2, 2) - mAMatrix(2, 2));
}

void EMT::Ph3::VSIVoltageControlVBR::mnaParentAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes)
{
	modifiedAttributes.push_back(mRightVector);
	prevStepDependencies.push_back(mIntfVoltage);
	prevStepDependencies.push_back(mIntfCurrent);
}

// TODO
void EMT::Ph3::VSIVoltageControlVBR::mnaParentPreStep(Real time, Int timeStepCount)
{
	// Transformation interface forward
	**mVcap_dq = parkTransformPowerInvariant(**mThetaInv, **mSubCapacitorF->mIntfVoltage);
	**mIfilter_dq = parkTransformPowerInvariant(**mThetaInv, mFilterCurrent);

	// TODO: droop
	// if (mWithDroop)
	//	mDroop->signalStep(time, timeStepCount);

	// Update nominal system angle
	**mThetaSys = **mThetaSys + mTimeStep * mOmegaNom;

	//  VCO Step
	**mThetaInv = **mThetaInv + mTimeStep * **mOmega;

	//
	if (mWithControl)
		mVhist_dq = mVSIController->step(**mVcap_dq, **mIfilter_dq);

	// Transformation interface backward
	mVhist = inverseParkTransformPowerInvariant(**mThetaInv, mVhist_dq);

	// stamp right side vector
	mnaApplyRightSideVectorStamp(**mRightVector);
}

void EMT::Ph3::VSIVoltageControlVBR::mnaParentApplyRightSideVectorStamp(Matrix &rightVector)
{
	mEquivCurrent = mEquivCond * ((**mSourceValue).real() - **mIntfVoltage) + mPrevCurrFac * mFilterCurrent;
	Matrix res = mVhist - mAMatrix * mEquivCurrent;
	Math::addToVectorElement(**mRightVector, mVirtualNodes[0]->matrixNodeIndex(CPS::PhaseType::A), res(0, 0));
	Math::addToVectorElement(**mRightVector, mVirtualNodes[0]->matrixNodeIndex(CPS::PhaseType::B), res(1, 0));
	Math::addToVectorElement(**mRightVector, mVirtualNodes[0]->matrixNodeIndex(CPS::PhaseType::C), res(2, 0));

	Math::addToVectorElement(**mRightVector, matrixNodeIndex(0, 0), mEquivCurrent(0, 0));
	Math::addToVectorElement(**mRightVector, matrixNodeIndex(0, 1), mEquivCurrent(1, 0));
	Math::addToVectorElement(**mRightVector, matrixNodeIndex(0, 2), mEquivCurrent(2, 0));
}

void EMT::Ph3::VSIVoltageControlVBR::mnaParentAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector)
{
	attributeDependencies.push_back(leftVector);
	attributeDependencies.push_back(mRightVector);
	modifiedAttributes.push_back(mIntfVoltage);
	modifiedAttributes.push_back(mIntfCurrent);
}

void EMT::Ph3::VSIVoltageControlVBR::mnaParentPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector)
{
	mnaCompUpdateVoltage(**leftVector);
	mnaCompUpdateCurrent(**leftVector);
	updatePower();
}

void EMT::Ph3::VSIVoltageControlVBR::mnaCompUpdateVoltage(const Matrix &leftVector)
{
	(**mIntfVoltage)(0, 0) = Math::realFromVectorElement(leftVector, matrixNodeIndex(0, 0));
	(**mIntfVoltage)(1, 0) = Math::realFromVectorElement(leftVector, matrixNodeIndex(0, 1));
	(**mIntfVoltage)(2, 0) = Math::realFromVectorElement(leftVector, matrixNodeIndex(0, 2));
	(**mSourceValue)(0, 0) = Math::realFromVectorElement(
		leftVector, mVirtualNodes[0]->matrixNodeIndex(PhaseType::A));
	(**mSourceValue)(1, 0) = Math::realFromVectorElement(
		leftVector, mVirtualNodes[0]->matrixNodeIndex(PhaseType::B));
	(**mSourceValue)(2, 0) = Math::realFromVectorElement(
		leftVector, mVirtualNodes[0]->matrixNodeIndex(PhaseType::C));
}

void EMT::Ph3::VSIVoltageControlVBR::mnaCompUpdateCurrent(const Matrix &leftvector)
{
	mFilterCurrent = mEquivCond * ((**mSourceValue).real() - **mIntfVoltage) + mEquivCurrent;
	**mIntfCurrent = mSubCapacitorF->mIntfCurrent->get() + mFilterCurrent;
}

void EMT::Ph3::VSIVoltageControlVBR::updatePower()
{
	Complex intfVoltageDQ = parkTransformPowerInvariant(**mThetaInv, **mIntfVoltage);
	Complex intfCurrentDQ = parkTransformPowerInvariant(**mThetaInv, **mIntfCurrent);
	**mPower = Complex(intfVoltageDQ.real() * intfCurrentDQ.real() + intfVoltageDQ.imag() * intfCurrentDQ.imag(),
					   intfVoltageDQ.imag() * intfCurrentDQ.real() - intfVoltageDQ.real() * intfCurrentDQ.imag());
}

Complex EMT::Ph3::VSIVoltageControlVBR::parkTransformPowerInvariant(Real theta, const Matrix &fabc)
{
	// Calculates fdq = Tdq * fabc
	// Assumes that d-axis starts aligned with phase a
	Matrix Tdq = getParkTransformMatrixPowerInvariant(theta);
	Matrix dqvector = Tdq * fabc;

	return Complex(dqvector(0, 0), dqvector(1, 0));
}

Matrix EMT::Ph3::VSIVoltageControlVBR::getParkTransformMatrixPowerInvariant(Real theta)
{
	// Return park matrix for theta
	// Assumes that d-axis starts aligned with phase a
	Matrix Tdq = Matrix::Zero(2, 3);
	Real k = sqrt(2. / 3.);
	Tdq << k * cos(theta), k * cos(theta - 2. * M_PI / 3.), k * cos(theta + 2. * M_PI / 3.),
		-k * sin(theta), -k * sin(theta - 2. * M_PI / 3.), -k * sin(theta + 2. * M_PI / 3.);
	return Tdq;
}

Matrix EMT::Ph3::VSIVoltageControlVBR::inverseParkTransformPowerInvariant(Real theta, const Complex &fdq)
{
	// Calculates fabc = Tabc * fdq
	// with d-axis starts aligned with phase a
	Matrix Fdq = Matrix::Zero(2, 1);
	Fdq << fdq.real(), fdq.imag();
	Matrix Tabc = getInverseParkTransformMatrixPowerInvariant(theta);

	return Tabc * Fdq;
}

Matrix EMT::Ph3::VSIVoltageControlVBR::getInverseParkTransformMatrixPowerInvariant(Real theta)
{
	// Return inverse park matrix for theta
	/// with d-axis starts aligned with phase a
	Matrix Tabc = Matrix::Zero(3, 2);
	Real k = sqrt(2. / 3.);
	Tabc << k * cos(theta), -k * sin(theta),
		k * cos(theta - 2. * M_PI / 3.), -k * sin(theta - 2. * M_PI / 3.),
		k * cos(theta + 2. * M_PI / 3.), -k * sin(theta + 2. * M_PI / 3.);
	return Tabc;
}
