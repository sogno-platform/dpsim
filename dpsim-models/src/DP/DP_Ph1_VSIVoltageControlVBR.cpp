/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/DP/DP_Ph1_VSIVoltageControlVBR.h>
#include <dpsim-models/Signal/VSIControlType1.h>

using namespace CPS;

DP::Ph1::VSIVoltageControlVBR::VSIVoltageControlVBR(String uid, String name,
                                                    Logger::Level logLevel,
                                                    Bool modelAsCurrentSource)
    : CompositePowerComp<Complex>(uid, name, true, true, logLevel),
      VSIVoltageSourceInverterDQ<Complex>(this->mSLog, mAttributes,
                                          modelAsCurrentSource, false)
{

  setTerminalNumber(1);
  setVirtualNodeNumber(this->determineNumberOfVirtualNodes());

  **mIntfVoltage = MatrixComp::Zero(1, 1);
  **mIntfCurrent = MatrixComp::Zero(1, 1);
  mDqToComplexA = Matrix::Zero(2, 2);
}

void DP::Ph1::VSIVoltageControlVBR::createSubComponents()
{
  // Capacitor as part of the LC filter
  mSubCapacitorF = DP::Ph1::Capacitor::make(**mName + "_CapF", mLogLevel);
  mSubCapacitorF->setParameters(mCf);
  addMNASubComponent(mSubCapacitorF, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);
}

void DP::Ph1::VSIVoltageControlVBR::initializeFromNodesAndTerminals(
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
  // FIXME: later is mSourceValue used to store the history term
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

void DP::Ph1::VSIVoltageControlVBR::initVars(Real timeStep)
{
  for (Int freq = 0; freq < mNumFreqs; freq++)
  {
    Real a = timeStep / (2. * mLf);
    Real b = timeStep * 2. * PI * mFrequencies(freq, 0) / 2.;

    Real equivCondReal = (a + mRf * std::pow(a, 2)) / (std::pow(1. + mRf * a, 2) + std::pow(b, 2));
    Real equivCondImag = -a * b / (std::pow(1. + mRf * a, 2) + std::pow(b, 2));
    mEquivCond(freq, 0) = {equivCondReal, equivCondImag};

    Real preCurrFracReal = (1. - std::pow(b, 2) + -std::pow(mRf * a, 2)) / (std::pow(1. + mRf * a, 2) + std::pow(b, 2));
    Real preCurrFracImag = (-2. * b) / (std::pow(1. + mRf * a, 2) + std::pow(b, 2));
    mPrevCurrFac(freq, 0) = {preCurrFracReal, preCurrFracImag};

    // TODO: implement it for mNumFreqs>1
    mEquivCurrent(freq, 0) = mEquivCond(freq, 0) * ((**mSourceValue)(0, 0) - (**mIntfVoltage)(0, 0)) + mPrevCurrFac(freq, 0) * mFilterCurrent(0, 0);
  }

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

  mZ = Matrix::Zero(2, 2);
  mZ << mEquivCond(0, 0).real(), -mEquivCond(0, 0).imag(), mEquivCond(0, 0).imag(), mEquivCond(0, 0).real();
}

void DP::Ph1::VSIVoltageControlVBR::mnaParentInitialize(
    Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector)
{
  mTimeStep = timeStep;

  // initialize RL Element matrix
  mEquivCurrent = MatrixComp::Zero(mNumFreqs, 1);
  mEquivCond = MatrixComp::Zero(mNumFreqs, 1);
  mPrevCurrFac = MatrixComp::Zero(mNumFreqs, 1);

  if (mWithControl)
  {
    mVSIController->initialize(**mSourceValue_dq, **mVcap_dq, **mIfilter_dq,
                               mTimeStep, mModelAsCurrentSource);
    mVSIController->calculateVBRconstants();
  }
  if (mWithDroopControl)
  {
    mDroopController->initialize(mTimeStep, **mOmega);
  }
  initVars(timeStep);

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
}

void DP::Ph1::VSIVoltageControlVBR::calculateResistanceMatrix()
{
  mAMatrix = mDqToComplexA * mA * mComplexAToDq * mZ;
  mEMatrix = mDqToComplexA * mE * mComplexAToDq;
}

void DP::Ph1::VSIVoltageControlVBR::mnaParentApplySystemMatrixStamp(
    SparseMatrixRow &systemMatrix)
{
  // TODO
  update_DqToComplexATransformMatrix();
  mComplexAToDq = mDqToComplexA.transpose();

  // calculate resistance matrix at t=k+1
  calculateResistanceMatrix();

  // Stamp filter current: I = (Vc - Vsource) * Admittance
  // TODO: number of frequencies >1?
  Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0), matrixNodeIndex(0),
                           mEquivCond(0, 0));
  Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0),
                           mVirtualNodes[0]->matrixNodeIndex(), -mEquivCond(0, 0));

  // Stamp voltage source equation
  Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(),
                           mVirtualNodes[0]->matrixNodeIndex(),
                           Matrix::Identity(2, 2) + mAMatrix);
  Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(),
                           matrixNodeIndex(0),
                           -mEMatrix - mAMatrix);
}

void DP::Ph1::VSIVoltageControlVBR::mnaParentAddPreStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes)
{
  modifiedAttributes.push_back(mRightVector);
  prevStepDependencies.push_back(mIntfVoltage);
  prevStepDependencies.push_back(mIntfCurrent);
}

void DP::Ph1::VSIVoltageControlVBR::mnaParentPreStep(Real time,
                                                     Int timeStepCount)
{
  // get measurements at time t=k
  **mVcap_dq = Math::rotatingFrame2to1((**mSubCapacitorF->mIntfVoltage)(0, 0),
                                       **mThetaInv, **mThetaSys);
  **mIfilter_dq =
      Math::rotatingFrame2to1(mFilterCurrent(0, 0), **mThetaInv, **mThetaSys);

  // Droop contoller step
  if (mWithDroopControl)
  {
    mDroopController->step((**mPower).real());
  }

  // VCO Step
  **mThetaInv = **mThetaInv + mTimeStep * **mOmega;

  // Update nominal system angle
  **mThetaSys = **mThetaSys + mTimeStep * mOmegaNom;

  // calculat history term
  if (mWithControl)
    mVhist = mVSIController->stepVBR(**mVcap_dq, **mIfilter_dq);

  mnaApplyRightSideVectorStamp(**mRightVector);
}

void DP::Ph1::VSIVoltageControlVBR::mnaParentApplyRightSideVectorStamp(
    Matrix &rightVector)
{
  // TODO: implement it for mNumFreqs>1
  mEquivCurrent(0, 0) = mEquivCond(0, 0) * ((**mSourceValue)(0, 0) - (**mIntfVoltage)(0, 0)) + mPrevCurrFac(0, 0) * mFilterCurrent(0, 0);

  // in complex:
  Complex dqToComplexA = Complex(mDqToComplexA(0, 0), -mDqToComplexA(0, 1));
  Complex eqVoltageSource = dqToComplexA * mVhist - std::dynamic_pointer_cast<CPS::Signal::VSIControlType1>(mVSIController)
                                                            ->mA_VBR *
                                                        mEquivCurrent(0, 0);
  Math::setVectorElement(rightVector, mVirtualNodes[0]->matrixNodeIndex(), eqVoltageSource);

  // stamp equivalent current RL element
  // TODO: Implement function addToVectorElement for multiple frequencies
  // for (Int freq = 0; freq < mNumFreqs; freq++)
  //{
  //  Math::addToVectorElement(rightVector, matrixNodeIndex(0), mEquivCurrent(freq, 0), 1, 0, freq);
  //}
  //
  Math::addToVectorElement(rightVector, matrixNodeIndex(0), mEquivCurrent(0, 0));
}

void DP::Ph1::VSIVoltageControlVBR::mnaParentAddPostStepDependencies(
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

void DP::Ph1::VSIVoltageControlVBR::mnaParentPostStep(
    Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector)
{
  mnaCompUpdateVoltage(**leftVector);
  mnaCompUpdateCurrent(**leftVector);
  updatePower();
}

void DP::Ph1::VSIVoltageControlVBR::mnaCompUpdateVoltage(
    const Matrix &leftVector)
{
  (**mIntfVoltage)(0, 0) =
      Math::complexFromVectorElement(leftVector, matrixNodeIndex(0));
  (**mSourceValue)(0, 0) = Math::complexFromVectorElement(
      leftVector, mVirtualNodes[0]->matrixNodeIndex());
}

void DP::Ph1::VSIVoltageControlVBR::mnaCompUpdateCurrent(
    const Matrix &leftvector)
{
  // TODO: calculations for number_frequencies>1
  mFilterCurrent(0, 0) = mEquivCond(0, 0) * ((**mSourceValue)(0, 0) - (**mIntfVoltage)(0, 0)) + mEquivCurrent(0, 0);
  **mIntfCurrent =
      mSubCapacitorF->mIntfCurrent->get() + mFilterCurrent;
}

void DP::Ph1::VSIVoltageControlVBR::updatePower()
{
  **mPower = (**mIntfVoltage)(0, 0) * std::conj((**mIntfCurrent)(0, 0));
}

void DP::Ph1::VSIVoltageControlVBR::update_DqToComplexATransformMatrix()
{
  mDqToComplexA << cos(**mThetaInv - **mThetaSys), -sin(**mThetaInv - **mThetaSys),
      sin(**mThetaInv - **mThetaSys), cos(**mThetaInv - **mThetaSys);
}