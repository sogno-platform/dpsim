// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <stdexcept>

#include <dpsim-models/DP/DP_VTypeSplitSSNComp.h>
#include <dpsim-models/MathUtils.h>

using namespace CPS;

DP::VTypeSplitSSNComp::VTypeSplitSSNComp(
    String uid, String name, Int packedInputSize, Int packedOutputSize,
    Int controllerStateSize, Int controllerInputSize, Int controllerOutputSize,
    Logger::Level logLevel)
    : MNASimPowerComp<Complex>(uid, name, true, true, logLevel),
      mPackedInputSize(packedInputSize), mPackedOutputSize(packedOutputSize),
      mControllerStateSize(controllerStateSize),
      mControllerInputSize(controllerInputSize),
      mControllerOutputSize(controllerOutputSize),
      mMeasurementState(Matrix::Zero(controllerInputSize, 0)),
      mMeasurementTerminal(Matrix::Zero(controllerInputSize, packedInputSize)),
      mSplitDiscreteA(Matrix::Zero(0, 0)),
      mSplitDiscreteB(Matrix::Zero(0, packedInputSize)),
      mSplitHistoryC(Matrix::Zero(packedOutputSize, 0)),
      mControllerBdUsed(Matrix::Zero(controllerStateSize, controllerInputSize)),
      mTimeStep(0.0), mW(Matrix::Zero(packedOutputSize, packedInputSize)),
      mYHist(MatrixComp::Zero(packedOutputSize / 2, 1)),
      mX(mAttributes->create<Matrix>("x")),
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
  if (packedInputSize <= 0 || packedInputSize % 2 != 0 ||
      packedOutputSize <= 0 || packedOutputSize % 2 != 0 ||
      controllerStateSize < 0 || controllerInputSize < 0 ||
      controllerOutputSize <= 0)
    throw std::invalid_argument("Invalid DP split SSN dimensions.");

  **mIntfVoltage = MatrixComp::Zero(packedInputSize / 2, 1);
  **mIntfCurrent = MatrixComp::Zero(packedOutputSize / 2, 1);
  mParametersSet = false;
}

Matrix DP::VTypeSplitSSNComp::packComplex(const MatrixComp &values) {
  Matrix result(2 * values.rows(), 1);
  for (Eigen::Index idx = 0; idx < values.rows(); ++idx) {
    result(2 * idx, 0) = values(idx, 0).real();
    result(2 * idx + 1, 0) = values(idx, 0).imag();
  }
  return result;
}

MatrixComp DP::VTypeSplitSSNComp::unpackComplex(const Matrix &values) {
  if (values.cols() != 1 || values.rows() % 2 != 0)
    throw std::invalid_argument(
        "Packed complex vector has invalid dimensions.");

  MatrixComp result(values.rows() / 2, 1);
  for (Eigen::Index idx = 0; idx < result.rows(); ++idx)
    result(idx, 0) = Complex(values(2 * idx, 0), values(2 * idx + 1, 0));
  return result;
}

void DP::VTypeSplitSSNComp::setSplitParameters(
    const Matrix &plantA, const Matrix &plantB, const Matrix &plantC,
    const Matrix &plantD, const Matrix &controllerOutputB,
    const Matrix &measurementState, const Matrix &measurementTerminal) {
  const Int plantStateSize = plantA.rows();
  if (plantA.cols() != plantStateSize || plantB.rows() != plantStateSize ||
      plantB.cols() != mPackedInputSize || plantC.rows() != mPackedOutputSize ||
      plantC.cols() != plantStateSize || plantD.rows() != mPackedOutputSize ||
      plantD.cols() != mPackedInputSize)
    throw std::invalid_argument(
        "DP split SSN plant matrices have invalid dimensions.");
  if (controllerOutputB.rows() != plantStateSize ||
      controllerOutputB.cols() != mControllerOutputSize)
    throw std::invalid_argument(
        "Controller-output matrix has invalid dimensions.");
  if (measurementState.rows() != mControllerInputSize ||
      measurementState.cols() != plantStateSize ||
      measurementTerminal.rows() != mControllerInputSize ||
      measurementTerminal.cols() != mPackedInputSize)
    throw std::invalid_argument(
        "Controller measurement matrices have invalid dimensions.");

  mA = plantA;
  mB = plantB;
  mC = plantC;
  mD = plantD;
  mBControllerOutput = controllerOutputB;
  mMeasurementState = measurementState;
  mMeasurementTerminal = measurementTerminal;

  **mX = Matrix::Zero(plantStateSize, 1);
  mdA = Matrix::Zero(plantStateSize, plantStateSize);
  mdB = Matrix::Zero(plantStateSize, mPackedInputSize);
  mdBControllerOutput = Matrix::Zero(plantStateSize, mControllerOutputSize);
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
  mSplitDiscreteB = Matrix::Zero(splitStateSize, mPackedInputSize);
  mSplitHistoryC = Matrix::Zero(mPackedOutputSize, splitStateSize);
  mW.setZero();
  mYHist.setZero();
  mParametersSet = true;
}

Matrix DP::VTypeSplitSSNComp::buildControllerMeasurement(
    const Matrix &plantState, const Matrix &terminalVoltage) const {
  return mMeasurementState * plantState +
         mMeasurementTerminal * terminalVoltage;
}

void DP::VTypeSplitSSNComp::recomputeControllerDiscreteModel() {
  if (mTimeStep <= 0.0)
    return;
  Math::calculateStateSpaceTrapezoidalMatrices(
      mControllerA, mControllerB, mControllerE, mTimeStep, mControllerAd,
      mControllerBd, mControllerEd);
}

void DP::VTypeSplitSSNComp::rebuildSplitDiscreteModel(
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
  mSplitDiscreteB.block(controllerOffset, 0, mControllerStateSize,
                        mPackedInputSize) =
      controllerInputUpdate * measurementFromTerminal;

  mSplitDiscreteA.block(plantOffset, plantOffset, mA.rows(), mA.cols()) = mdA;
  mSplitDiscreteA.block(plantOffset, delayOffset, mA.rows(),
                        mControllerOutputSize) =
      2.0 * mdA * mdBControllerOutput;
  mSplitDiscreteB.block(plantOffset, 0, mA.rows(), mPackedInputSize) =
      (mdA + Matrix::Identity(mA.rows(), mA.cols())) * mdB;

  mSplitDiscreteA.block(delayOffset, controllerOffset, mControllerOutputSize,
                        mControllerStateSize) = mControllerC;
  mSplitDiscreteA.block(delayOffset, plantOffset, mControllerOutputSize,
                        mA.rows()) =
      delayedOutputFromMeasurement * measurementFromPlantHistory;
  mSplitDiscreteA.block(delayOffset, delayOffset, mControllerOutputSize,
                        mControllerOutputSize) =
      delayedOutputFromMeasurement * measurementFromDelay;
  mSplitDiscreteB.block(delayOffset, 0, mControllerOutputSize,
                        mPackedInputSize) =
      delayedOutputFromMeasurement * measurementFromTerminal;

  mSplitHistoryC.block(0, plantOffset, mC.rows(), mC.cols()) = mC;
  mSplitHistoryC.block(0, delayOffset, mC.rows(), mControllerOutputSize) =
      2.0 * mC * mdBControllerOutput;
}

void DP::VTypeSplitSSNComp::recomputeDiscreteModel() {
  Math::calculateStateSpaceTrapezoidalMatrices(mA, mB, mTimeStep, mdA, mdB);
  const Matrix identity = Matrix::Identity(mA.rows(), mA.cols());
  const Matrix lhs = identity - 0.5 * mTimeStep * mA;
  mdBControllerOutput =
      lhs.fullPivLu().solve(0.5 * mTimeStep * mBControllerOutput);
  mW = mC * mdB + mD;
  recomputeControllerDiscreteModel();
  rebuildSplitDiscreteModel(mControllerBd);
}

void DP::VTypeSplitSSNComp::updateLogAttributes(const Matrix &) const {}

UInt DP::VTypeSplitSSNComp::getSplitStateCount() const {
  return static_cast<UInt>(mSplitDiscreteA.rows());
}
const Matrix &DP::VTypeSplitSSNComp::getSplitDiscreteA() const {
  return mSplitDiscreteA;
}
const Matrix &DP::VTypeSplitSSNComp::getSplitDiscreteB() const {
  return mSplitDiscreteB;
}
const Matrix &DP::VTypeSplitSSNComp::getSplitHistoryC() const {
  return mSplitHistoryC;
}
const Matrix &DP::VTypeSplitSSNComp::getControllerDiscreteA() const {
  return mControllerAd;
}
const Matrix &DP::VTypeSplitSSNComp::getControllerDiscreteB() const {
  return mControllerBd;
}
const Matrix &DP::VTypeSplitSSNComp::getControllerDiscreteBUsed() const {
  return mControllerBdUsed;
}
const Matrix &DP::VTypeSplitSSNComp::getDiscreteControllerOutputB() const {
  return mdBControllerOutput;
}
Attribute<Matrix>::Ptr DP::VTypeSplitSSNComp::getSplitStateAttribute() const {
  return mX;
}

void DP::VTypeSplitSSNComp::mnaCompInitialize(Real, Real timeStep,
                                              Attribute<Matrix>::Ptr) {
  if (!mParametersSet)
    throw std::logic_error(
        "setSplitParameters() must be called before initialization.");
  if (mNumFreqs != 1)
    throw std::logic_error(
        "DP split SSN components currently support one carrier frequency.");
  mTimeStep = timeStep;
  updateMatrixNodeIndices();
  recomputeDiscreteModel();
  const Matrix history =
      mC * (mdA * (**mX) + mdB * packComplex(**mIntfVoltage) +
            2.0 * mdBControllerOutput * mDelayedControllerOutput);
  mYHist = unpackComplex(history);
}

void DP::VTypeSplitSSNComp::mnaCompAddPreStepDependencies(
    AttributeBase::List &prevStepDependencies, AttributeBase::List &,
    AttributeBase::List &modifiedAttributes) {
  modifiedAttributes.push_back(mRightVector);
  prevStepDependencies.push_back(mX);
  prevStepDependencies.push_back(mIntfVoltage);
}

void DP::VTypeSplitSSNComp::mnaCompPreStep(Real, Int) {
  const Matrix history =
      mC * (mdA * (**mX) + mdB * packComplex(**mIntfVoltage) +
            2.0 * mdBControllerOutput * mDelayedControllerOutput);
  mYHist = unpackComplex(history);
  mnaCompApplyRightSideVectorStamp(**mRightVector);
}

void DP::VTypeSplitSSNComp::mnaCompAddPostStepDependencies(
    AttributeBase::List &, AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes,
    Attribute<Matrix>::Ptr &leftVector) {
  attributeDependencies.push_back(leftVector);
  modifiedAttributes.push_back(mIntfVoltage);
  modifiedAttributes.push_back(mIntfCurrent);
  modifiedAttributes.push_back(mX);
}

void DP::VTypeSplitSSNComp::mnaCompUpdateCurrent(const Matrix &) {
  **mIntfCurrent = unpackComplex(mW * packComplex(**mIntfVoltage)) + mYHist;
}

void DP::VTypeSplitSSNComp::mnaCompPostStep(
    Real, Int, Attribute<Matrix>::Ptr &leftVector) {
  const MatrixComp uOldComplex = **mIntfVoltage;
  mnaCompUpdateVoltage(**leftVector);
  mnaCompUpdateCurrent(**leftVector);

  const Matrix uOld = packComplex(uOldComplex);
  const Matrix uNew = packComplex(**mIntfVoltage);
  const Matrix delayedOutputUsed = mDelayedControllerOutput;
  const Matrix controllerBdUsed = mControllerBd;

  **mX = mdA * (**mX) + mdB * (uNew + uOld) +
         2.0 * mdBControllerOutput * delayedOutputUsed;
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
  rebuildSplitDiscreteModel(controllerBdUsed);
  evaluateControllerOutput(mControllerState, measurementNew, mControllerOutput);
  mDelayedControllerOutput = mControllerOutput;
  updateLogAttributes(uNew);
}
