// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <dpsim-models/EMT/EMT_VTypeSplitSSNComp.h>

#include <stdexcept>

using namespace CPS;

EMT::VTypeSplitSSNComp::VTypeSplitSSNComp(String uid, String name,
                                          Int inputSize, Int outputSize,
                                          Int controllerStateSize,
                                          Int controllerInputSize,
                                          Int controllerOutputSize,
                                          Logger::Level logLevel)
    : VTypeSSNComp(uid, name, inputSize, outputSize, logLevel),
      mControllerStateSize(controllerStateSize),
      mControllerInputSize(controllerInputSize),
      mControllerOutputSize(controllerOutputSize),
      mMeasurementState(Matrix::Zero(controllerInputSize, 0)),
      mMeasurementTerminal(Matrix::Zero(controllerInputSize, inputSize)),
      mSplitDiscreteA(Matrix::Zero(0, 0)),
      mSplitDiscreteB(Matrix::Zero(0, inputSize)),
      mSplitHistoryC(Matrix::Zero(outputSize, 0)),
      mControllerBdUsed(Matrix::Zero(controllerStateSize, controllerInputSize)),
      mBControllerOutput(Matrix::Zero(0, controllerOutputSize)),
      mdBControllerOutput(Matrix::Zero(0, controllerOutputSize)),
      mControllerState(Matrix::Zero(controllerStateSize, 1)),
      mControllerMeasurementOld(Matrix::Zero(controllerInputSize, 1)),
      mControllerA(Matrix::Zero(controllerStateSize, controllerStateSize)),
      mControllerB(Matrix::Zero(controllerStateSize, controllerInputSize)),
      mControllerC(Matrix::Zero(controllerOutputSize, controllerStateSize)),
      mControllerD(Matrix::Zero(controllerOutputSize, controllerInputSize)),
      mControllerE(Matrix::Zero(controllerStateSize, 1)),
      mControllerF(Matrix::Zero(controllerOutputSize, 1)),
      mControllerAd(Matrix::Zero(controllerStateSize, controllerStateSize)),
      mControllerBd(Matrix::Zero(controllerStateSize, controllerInputSize)),
      mControllerEd(Matrix::Zero(controllerStateSize, 1)),
      mControllerOutput(Matrix::Zero(controllerOutputSize, 1)),
      mDelayedControllerOutput(Matrix::Zero(controllerOutputSize, 1)) {
  if (controllerStateSize < 0 || controllerInputSize < 0 ||
      controllerOutputSize <= 0)
    throw std::invalid_argument("Invalid split SSN controller dimensions.");
}

void EMT::VTypeSplitSSNComp::setSplitParameters(
    const Matrix &plantA, const Matrix &plantB, const Matrix &plantC,
    const Matrix &plantD, const Matrix &controllerOutputB,
    const Matrix &measurementState, const Matrix &measurementTerminal) {
  const Int plantStateSize = plantA.rows();

  if (controllerOutputB.rows() != plantStateSize ||
      controllerOutputB.cols() != mControllerOutputSize)
    throw std::invalid_argument(
        "Controller-output input matrix has invalid dimensions.");

  if (measurementState.rows() != mControllerInputSize ||
      measurementState.cols() != plantStateSize)
    throw std::invalid_argument(
        "Measurement state matrix has invalid dimensions.");

  if (measurementTerminal.rows() != mControllerInputSize ||
      measurementTerminal.cols() != plantB.cols())
    throw std::invalid_argument(
        "Measurement terminal matrix has invalid dimensions.");

  SSNComp::setParameters(plantA, plantB, plantC, plantD);

  mBControllerOutput = controllerOutputB;
  mdBControllerOutput = Matrix::Zero(plantStateSize, mControllerOutputSize);
  mMeasurementState = measurementState;
  mMeasurementTerminal = measurementTerminal;

  mControllerState.setZero();
  mControllerMeasurementOld.setZero();
  mControllerOutput.setZero();
  mDelayedControllerOutput.setZero();

  buildControllerStateSpaceModel(mControllerState, mControllerMeasurementOld,
                                 mControllerA, mControllerB, mControllerC,
                                 mControllerD, mControllerE, mControllerF);

  const Int splitStateSize =
      mControllerStateSize + plantStateSize + mControllerOutputSize;
  mSplitDiscreteA = Matrix::Zero(splitStateSize, splitStateSize);
  mSplitDiscreteB = Matrix::Zero(splitStateSize, plantB.cols());
  mSplitHistoryC = Matrix::Zero(plantC.rows(), splitStateSize);
}

Matrix EMT::VTypeSplitSSNComp::buildControllerMeasurement(
    const Matrix &plantState, const Matrix &terminalVoltage) const {
  if (plantState.rows() != mA.rows() || plantState.cols() != 1)
    throw std::invalid_argument(
        "Split SSN plant state has invalid dimensions.");

  if (terminalVoltage.rows() != mB.cols() || terminalVoltage.cols() != 1)
    throw std::invalid_argument(
        "Split SSN terminal voltage has invalid dimensions.");

  return mMeasurementState * plantState +
         mMeasurementTerminal * terminalVoltage;
}

void EMT::VTypeSplitSSNComp::recomputeControllerDiscreteModel() {
  if (mTimeStep <= 0.0)
    return;

  Math::calculateStateSpaceTrapezoidalMatrices(
      mControllerA, mControllerB, mControllerE, mTimeStep, mControllerAd,
      mControllerBd, mControllerEd);
}

void EMT::VTypeSplitSSNComp::rebuildSplitDiscreteModel(
    const Matrix &controllerBdUsed) {
  mControllerBdUsed = controllerBdUsed;

  const UInt controllerOffset = 0;
  const UInt plantOffset = static_cast<UInt>(mControllerStateSize);
  const UInt delayOffset = plantOffset + static_cast<UInt>(mA.rows());

  mSplitDiscreteA.setZero();
  mSplitDiscreteB.setZero();
  mSplitHistoryC.setZero();

  const Matrix controllerInputUpdate =
      mControllerAd * controllerBdUsed + mControllerBd;

  const Matrix measurementFromPlantHistory = mMeasurementState;
  const Matrix measurementFromDelay =
      2.0 * mMeasurementState * mdBControllerOutput;
  const Matrix measurementFromTerminal =
      mMeasurementState * mdB + mMeasurementTerminal;

  const Matrix delayedOutputFromMeasurement =
      mControllerC * controllerBdUsed + mControllerD;

  mSplitDiscreteA.block(controllerOffset, controllerOffset,
                        mControllerStateSize, mControllerStateSize) =
      mControllerAd;
  mSplitDiscreteA.block(controllerOffset, plantOffset, mControllerStateSize,
                        mA.rows()) =
      controllerInputUpdate * measurementFromPlantHistory;
  mSplitDiscreteA.block(controllerOffset, delayOffset, mControllerStateSize,
                        mControllerOutputSize) =
      controllerInputUpdate * measurementFromDelay;
  mSplitDiscreteB.block(controllerOffset, 0, mControllerStateSize, mB.cols()) =
      controllerInputUpdate * measurementFromTerminal;

  mSplitDiscreteA.block(plantOffset, plantOffset, mA.rows(), mA.cols()) = mdA;
  mSplitDiscreteA.block(plantOffset, delayOffset, mA.rows(),
                        mControllerOutputSize) =
      2.0 * mdA * mdBControllerOutput;
  mSplitDiscreteB.block(plantOffset, 0, mA.rows(), mB.cols()) =
      (mdA + Matrix::Identity(mA.rows(), mA.cols())) * mdB;

  mSplitDiscreteA.block(delayOffset, controllerOffset, mControllerOutputSize,
                        mControllerStateSize) = mControllerC;
  mSplitDiscreteA.block(delayOffset, plantOffset, mControllerOutputSize,
                        mA.rows()) =
      delayedOutputFromMeasurement * measurementFromPlantHistory;
  mSplitDiscreteA.block(delayOffset, delayOffset, mControllerOutputSize,
                        mControllerOutputSize) =
      delayedOutputFromMeasurement * measurementFromDelay;
  mSplitDiscreteB.block(delayOffset, 0, mControllerOutputSize, mB.cols()) =
      delayedOutputFromMeasurement * measurementFromTerminal;

  mSplitHistoryC.block(0, plantOffset, mC.rows(), mC.cols()) = mC;
  mSplitHistoryC.block(0, delayOffset, mC.rows(), mControllerOutputSize) =
      2.0 * mC * mdBControllerOutput;
}

void EMT::VTypeSplitSSNComp::recomputeDiscreteModel() {
  SSNComp::recomputeDiscreteModel();

  const Matrix identity = Matrix::Identity(mA.rows(), mA.cols());
  const Matrix lhs = identity - 0.5 * mTimeStep * mA;
  mdBControllerOutput =
      lhs.fullPivLu().solve(0.5 * mTimeStep * mBControllerOutput);

  recomputeControllerDiscreteModel();
  rebuildSplitDiscreteModel(mControllerBd);
}

Matrix EMT::VTypeSplitSSNComp::calculateHistoryVector() const {
  return mC * (mdA * (**mX) + mdB * (**mIntfVoltage) +
               2.0 * mdBControllerOutput * mDelayedControllerOutput);
}

void EMT::VTypeSplitSSNComp::updateState(const Matrix &uOld,
                                         const Matrix &uNew) {
  const Matrix delayedControllerOutputUsedThisStep = mDelayedControllerOutput;
  const Matrix controllerBdUsedThisStep = mControllerBd;

  **mX = mdA * (**mX) + mdB * (uNew + uOld) +
         2.0 * mdBControllerOutput * delayedControllerOutputUsedThisStep;

  const Matrix measurementNew = buildControllerMeasurement(**mX, uNew);

  mControllerState =
      mControllerAd * mControllerState +
      mControllerBd * (measurementNew + mControllerMeasurementOld) +
      mControllerEd;
  mControllerMeasurementOld = measurementNew;

  buildControllerStateSpaceModel(mControllerState, measurementNew, mControllerA,
                                 mControllerB, mControllerC, mControllerD,
                                 mControllerE, mControllerF);
  recomputeControllerDiscreteModel();

  rebuildSplitDiscreteModel(controllerBdUsedThisStep);

  evaluateControllerOutput(mControllerState, measurementNew, mControllerOutput);
  mDelayedControllerOutput = mControllerOutput;
}

UInt EMT::VTypeSplitSSNComp::getSplitStateCount() const {
  return static_cast<UInt>(mSplitDiscreteA.rows());
}

const Matrix &EMT::VTypeSplitSSNComp::getSplitDiscreteA() const {
  return mSplitDiscreteA;
}

const Matrix &EMT::VTypeSplitSSNComp::getSplitDiscreteB() const {
  return mSplitDiscreteB;
}

const Matrix &EMT::VTypeSplitSSNComp::getSplitHistoryC() const {
  return mSplitHistoryC;
}

const Matrix &EMT::VTypeSplitSSNComp::getControllerDiscreteA() const {
  return mControllerAd;
}

const Matrix &EMT::VTypeSplitSSNComp::getControllerDiscreteB() const {
  return mControllerBd;
}

const Matrix &EMT::VTypeSplitSSNComp::getControllerDiscreteBUsed() const {
  return mControllerBdUsed;
}

const Matrix &EMT::VTypeSplitSSNComp::getDiscreteControllerOutputB() const {
  return mdBControllerOutput;
}

Attribute<Matrix>::Ptr EMT::VTypeSplitSSNComp::getSplitStateAttribute() const {
  return mX;
}
