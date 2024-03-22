/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/SP/SP_Ph1_VSIVoltageControlVBR.h>
#include <dpsim-models/Signal/VSIControlType1.h>

using namespace CPS;

SP::Ph1::VSIVoltageControlVBR::VSIVoltageControlVBR(String uid, String name,
													Logger::Level logLevel,
													Bool modelAsCurrentSource)
	: CompositePowerComp<Complex>(uid, name, true, true, logLevel),
	  VSIVoltageSourceInverterDQ<Complex>(this->mSLog, mAttributes,
										  modelAsCurrentSource,
										  false)
{

	setTerminalNumber(1);
	setVirtualNodeNumber(this->determineNumberOfVirtualNodes());

	**mIntfVoltage = MatrixComp::Zero(1, 1);
	**mIntfCurrent = MatrixComp::Zero(1, 1);
	mDqToComplexA = Matrix::Zero(2, 2);
}

void SP::Ph1::VSIVoltageControlVBR::createSubComponents()
{
	// Capacitor as part of the LC filter
	mSubCapacitorF = SP::Ph1::Capacitor::make(**mName + "_CapF", mLogLevel);
	mSubCapacitorF->setParameters(mCf);
	addMNASubComponent(mSubCapacitorF, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
					   MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, false);
}

void SP::Ph1::VSIVoltageControlVBR::initializeFromNodesAndTerminals(
	Real frequency)
{
	// terminal powers in consumer system -> convert to generator system
	**mPower = -terminal(0)->singlePower();

	// set initial interface quantities --> Current flowing into the inverter is positive
	(**mIntfVoltage)(0, 0) = initialSingleVoltage(0);
	(**mIntfCurrent)(0, 0) = std::conj(**mPower / (**mIntfVoltage)(0, 0));

	// initialize filter variables and set initial voltage of virtual nodes
	initializeFilterVariables((**mIntfVoltage)(0, 0), (**mIntfCurrent)(0, 0),
							  mVirtualNodes);

	// calculate initial source value
	(**mSourceValue)(0, 0) =
		Math::rotatingFrame2to1(**mSourceValue_dq, **mThetaSys, **mThetaInv);

	// Connect & Initialize electrical subcomponents
	mSubCapacitorF->connect({SimNode::GND, mTerminals[0]->node()});
	for (auto subcomp : mSubComponents)
	{
		subcomp->initialize(mFrequencies);
		subcomp->initializeFromNodesAndTerminals(frequency);
	}

	// TODO: droop
	**mOmega = mOmegaNom;

	//
	mAdmitance = Complex(1, 0) / Complex(mRf, **mOmega * mLf);

	/// initialize filter current in dp domain
	mFilterCurrent = Matrix::Zero(1, 1);
	mFilterCurrent(0, 0) = Math::rotatingFrame2to1(**mIfilter_dq, **mThetaSys, **mThetaInv);

	SPDLOG_LOGGER_INFO(mSLog,
					   "\n--- Initialization from powerflow ---"
					   "\nTerminal 0 connected to {} = sim node {}"
					   "\nInverter terminal voltage: {}[V]"
					   "\nInverter output current: {}[A]",
					   mTerminals[0]->node()->name(),
					   mTerminals[0]->node()->matrixNodeIndex(),
					   Logger::phasorToString((**mIntfVoltage)(0, 0)),
					   Logger::phasorToString((**mIntfCurrent)(0, 0)));
	mSLog->flush();
}

void SP::Ph1::VSIVoltageControlVBR::mnaParentInitialize(
	Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector)
{
	mTimeStep = timeStep;
	if (mWithControl)
	{
		mVSIController->initialize(**mSourceValue_dq, **mVcap_dq, **mIfilter_dq,
								   mTimeStep, false);
		mVSIController->calculateVBRconstants();
	}

	// initialize auxialiar Matrix
	mA = Matrix::Zero(2, 2);
	mE = Matrix::Zero(2, 2);
	mY = Matrix::Zero(2, 2);

	mY << mRf, -mLf * mOmegaNom, mLf * mOmegaNom, mRf;
	mY = mY.inverse();

	mA << std::dynamic_pointer_cast<CPS::Signal::VSIControlType1>(mVSIController)
			  ->mA_VBR,
		0, 0,
		std::dynamic_pointer_cast<CPS::Signal::VSIControlType1>(mVSIController)
			->mA_VBR;

	mE << std::dynamic_pointer_cast<CPS::Signal::VSIControlType1>(mVSIController)
			  ->mE_VBR,
		0, 0,
		std::dynamic_pointer_cast<CPS::Signal::VSIControlType1>(mVSIController)
			->mE_VBR;

	// get matrix dimension to properly set variable entries
	auto n = leftVector->asRawPointer()->rows();
	auto complexOffset = (UInt)(n / 2);
	// upper left
	mVariableSystemMatrixEntries.push_back(
		std::make_pair<UInt, UInt>(mVirtualNodes[0]->matrixNodeIndex(),
								   mVirtualNodes[0]->matrixNodeIndex()));
	mVariableSystemMatrixEntries.push_back(std::make_pair<UInt, UInt>(
		mVirtualNodes[0]->matrixNodeIndex() + complexOffset,
		mVirtualNodes[0]->matrixNodeIndex()));
	mVariableSystemMatrixEntries.push_back(std::make_pair<UInt, UInt>(
		mVirtualNodes[0]->matrixNodeIndex(),
		mVirtualNodes[0]->matrixNodeIndex() + complexOffset));
	mVariableSystemMatrixEntries.push_back(std::make_pair<UInt, UInt>(
		mVirtualNodes[0]->matrixNodeIndex() + complexOffset,
		mVirtualNodes[0]->matrixNodeIndex() + complexOffset));

	// off diagonal
	mVariableSystemMatrixEntries.push_back(std::make_pair<UInt, UInt>(
		mVirtualNodes[0]->matrixNodeIndex(), matrixNodeIndex(0, 0)));
	mVariableSystemMatrixEntries.push_back(std::make_pair<UInt, UInt>(
		mVirtualNodes[0]->matrixNodeIndex() + complexOffset,
		matrixNodeIndex(0, 0)));
	mVariableSystemMatrixEntries.push_back(
		std::make_pair<UInt, UInt>(mVirtualNodes[0]->matrixNodeIndex(),
								   matrixNodeIndex(0, 0) + complexOffset));
	mVariableSystemMatrixEntries.push_back(std::make_pair<UInt, UInt>(
		mVirtualNodes[0]->matrixNodeIndex() + complexOffset,
		matrixNodeIndex(0, 0) + complexOffset));

	SPDLOG_LOGGER_INFO(mSLog, "List of index pairs of varying matrix entries: ");
	for (auto indexPair : mVariableSystemMatrixEntries)
		SPDLOG_LOGGER_INFO(mSLog, "({}, {})", indexPair.first, indexPair.second);
	mSLog->flush();
}

void SP::Ph1::VSIVoltageControlVBR::calculateResistanceMatrix()
{
	mEMatrix = mDqToComplexA * mE * mComplexAToDq;
	mAMatrix = mDqToComplexA * mA * mComplexAToDq * mY;
}

void SP::Ph1::VSIVoltageControlVBR::mnaParentApplySystemMatrixStamp(
	SparseMatrixRow &systemMatrix)
{
	update_DqToComplexATransformMatrix();
	mComplexAToDq = mDqToComplexA.transpose();
	calculateResistanceMatrix();

	// Stamp filter current: I = (Vc - Vsource) * Admittance
	Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0), matrixNodeIndex(0),
							 mAdmitance);
	Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0),
							 mVirtualNodes[0]->matrixNodeIndex(), -mAdmitance);

	// Stamp voltage source equation
	Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(),
							 mVirtualNodes[0]->matrixNodeIndex(),
							 Matrix::Identity(2, 2) + mAMatrix);
	Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(),
							 matrixNodeIndex(0),
							 -mEMatrix - mAMatrix);
}

void SP::Ph1::VSIVoltageControlVBR::mnaParentAddPreStepDependencies(
	AttributeBase::List &prevStepDependencies,
	AttributeBase::List &attributeDependencies,
	AttributeBase::List &modifiedAttributes)
{
	modifiedAttributes.push_back(mRightVector);
	prevStepDependencies.push_back(mIntfVoltage);
	prevStepDependencies.push_back(mIntfCurrent);
}

void SP::Ph1::VSIVoltageControlVBR::mnaParentPreStep(Real time,
													 Int timeStepCount)
{
	// get measurements
	**mVcap_dq = Math::rotatingFrame2to1((**mSubCapacitorF->mIntfVoltage)(0, 0),
										 **mThetaInv, **mThetaSys);
	**mIfilter_dq =
		Math::rotatingFrame2to1(mFilterCurrent(0, 0), **mThetaInv, **mThetaSys);

	// TODO: droop
	// if (mWithDroop)
	//	mDroop->signalStep(time, timeStepCount);

	//  VCO Step
	**mThetaInv = **mThetaInv + mTimeStep * **mOmega;

	// Update nominal system angle
	**mThetaSys = **mThetaSys + mTimeStep * mOmegaNom;

	// calculat history term
	if (mWithControl)
		mVhist = mVSIController->stepVBR(**mVcap_dq, **mIfilter_dq);

	update_DqToComplexATransformMatrix();
	mComplexAToDq = mDqToComplexA.transpose();

	// calculate resistance matrix at t=k+1
	calculateResistanceMatrix();

	mnaApplyRightSideVectorStamp(**mRightVector);
}

void SP::Ph1::VSIVoltageControlVBR::mnaParentApplyRightSideVectorStamp(
	Matrix &rightVector)
{
	Complex dqToComplexA = Complex(mDqToComplexA(0, 0), -mDqToComplexA(0, 1));
	Complex eqVoltageSource = dqToComplexA * mVhist;

	Math::setVectorElement(rightVector, mVirtualNodes[0]->matrixNodeIndex(),
						   eqVoltageSource);
}

void SP::Ph1::VSIVoltageControlVBR::mnaParentAddPostStepDependencies(
	AttributeBase::List &prevStepDependencies,
	AttributeBase::List &attributeDependencies,
	AttributeBase::List &modifiedAttributes,
	Attribute<Matrix>::Ptr &leftVector)
{
	attributeDependencies.push_back(leftVector);
	attributeDependencies.push_back(mRightVector);
	modifiedAttributes.push_back(mIntfVoltage);
	modifiedAttributes.push_back(mIntfCurrent);
}

void SP::Ph1::VSIVoltageControlVBR::mnaParentPostStep(
	Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector)
{
	mnaCompUpdateVoltage(**leftVector);
	mnaCompUpdateCurrent(**leftVector);
	updatePower();
}

void SP::Ph1::VSIVoltageControlVBR::mnaCompUpdateCurrent(
	const Matrix &leftvector)
{
	mFilterCurrent = (**mSourceValue - **mIntfVoltage) * mAdmitance;
	**mIntfCurrent =
		mSubCapacitorF->mIntfCurrent->get() + mFilterCurrent;
}

void SP::Ph1::VSIVoltageControlVBR::mnaCompUpdateVoltage(
	const Matrix &leftVector)
{
	(**mIntfVoltage)(0, 0) =
		Math::complexFromVectorElement(leftVector, matrixNodeIndex(0));
	(**mSourceValue)(0, 0) = Math::complexFromVectorElement(
		leftVector, mVirtualNodes[0]->matrixNodeIndex());
}

void SP::Ph1::VSIVoltageControlVBR::updatePower()
{
	**mPower = (**mIntfVoltage)(0, 0) * std::conj((**mIntfCurrent)(0, 0));
}

void SP::Ph1::VSIVoltageControlVBR::update_DqToComplexATransformMatrix()
{
	mDqToComplexA << cos(**mThetaInv - **mThetaSys), -sin(**mThetaInv - **mThetaSys),
		sin(**mThetaInv - **mThetaSys), cos(**mThetaInv - **mThetaSys);
}
