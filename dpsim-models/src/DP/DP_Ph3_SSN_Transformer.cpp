/* SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
 * SPDX-License-Identifier: MPL-2.0
 */

#include <dpsim-models/DP/DP_Ph3_SSN_Transformer.h>

#include <cmath>
#include <stdexcept>

using namespace CPS;

namespace {
constexpr Int kVoltage0 = 0;
constexpr Int kCurrent0 = 3;
constexpr Int kVoltage1 = 6;
constexpr Int kCurrent1 = 9;
constexpr Int kVoltageMag = 12;
constexpr Int kCurrentMag = 15;

using PhaseVector = Eigen::Matrix<Complex, 3, 1>;

PhaseVector phaseVoltage(Complex phaseA) {
  PhaseVector voltage;
  voltage(0) = phaseA;
  voltage(1) = phaseA * SHIFT_TO_PHASE_B;
  voltage(2) = phaseA * SHIFT_TO_PHASE_C;
  return voltage;
}
} // namespace

DP::Ph3::SSN::Transformer::Transformer(String uid, String name,
                                       Logger::Level logLevel,
                                       Bool withResistiveLosses)
    : MNASimPowerComp<Complex>(uid, name, true, true, logLevel),
      Base::Ph3::Transformer(mAttributes),
      mState(mAttributes->create<MatrixComp>("state")),
      mSeriesResistanceHalf(Matrix::Zero(3, 3)),
      mSeriesInductanceHalf(Matrix::Zero(3, 3)),
      mInductorAdmittance(MatrixComp::Zero(3, 3)),
      mSeriesSystemInverse(MatrixComp::Identity(3, 3)),
      mSeriesAdmittance(MatrixComp::Zero(3, 3)),
      mMagnetizingInductorAdmittance(MatrixComp::Zero(3, 3)),
      mMagnetizingConductance(MatrixComp::Zero(3, 3)),
      mMidpointInverse(MatrixComp::Zero(3, 3)),
      mPortAdmittance00(MatrixComp::Zero(3, 3)),
      mPortAdmittance01(MatrixComp::Zero(3, 3)),
      mPortAdmittance10(MatrixComp::Zero(3, 3)),
      mPortAdmittance11(MatrixComp::Zero(3, 3)),
      mHistory0(MatrixComp::Zero(3, 1)), mHistory1(MatrixComp::Zero(3, 1)),
      mHistoryMagnetizing(MatrixComp::Zero(3, 1)),
      mMagnetizingResistance(Matrix::Zero(3, 3)),
      mMagnetizingInductance(Matrix::Zero(3, 3)),
      mWithResistiveLosses(withResistiveLosses) {
  mPhaseType = PhaseType::ABC;
  setTerminalNumber(2);
  **mIntfVoltage = MatrixComp::Zero(3, 1);
  **mIntfCurrent = MatrixComp::Zero(3, 1);
  **mState = MatrixComp::Zero(18, 1);
}

SimPowerComp<Complex>::Ptr DP::Ph3::SSN::Transformer::clone(String copySuffix) {
  auto copy = SharedFactory<Transformer>::make(copySuffix, copySuffix,
                                               mLogLevel, mWithResistiveLosses);
  copy->setParameters(mNominalVoltagePrimary, mNominalVoltageSecondary,
                      mRatedPower, std::abs(**mRatio), std::arg(**mRatio),
                      mResistance, mInductance);
  copy->setTapRatio(tapRatio());
  copy->setMagnetizingBranch(mNoLoadCurrent, mNoLoadLoss);
  copy->setReferenceWinding(mReferenceWinding);
  return copy;
}

void DP::Ph3::SSN::Transformer::setParameters(
    Real nomVoltagePrimary, Real nomVoltageSecondary, Real ratedPower,
    Real ratioAbs, Real ratioPhase, Matrix resistance, Matrix inductance) {
  if (resistance.rows() != 3 || resistance.cols() != 3 ||
      inductance.rows() != 3 || inductance.cols() != 3)
    throw std::invalid_argument("Transformer matrices must be 3 x 3.");
  Base::Ph3::Transformer::setParameters(nomVoltagePrimary, nomVoltageSecondary,
                                        ratedPower, ratioAbs, ratioPhase,
                                        resistance, inductance);
  mParametersSet = true;
}

void DP::Ph3::SSN::Transformer::setMagnetizingBranch(Real noLoadCurrent,
                                                     Real noLoadLoss) {
  if (!std::isfinite(noLoadCurrent) || !std::isfinite(noLoadLoss) ||
      noLoadCurrent <= noLoadLoss || noLoadLoss <= 0.)
    throw std::invalid_argument("Invalid transformer magnetizing branch.");
  mNoLoadCurrent = noLoadCurrent;
  mNoLoadLoss = noLoadLoss;
}

void DP::Ph3::SSN::Transformer::setReferenceWinding(
    WindingReference referenceWinding) {
  if (referenceWinding == WindingReference::Tertiary)
    throw std::invalid_argument("Three-winding transformers are unsupported.");
  mReferenceWinding = referenceWinding;
}

void DP::Ph3::SSN::Transformer::resolveWindingRoles() {
  switch (mReferenceWinding) {
  case WindingReference::Primary:
    mReferenceTerminal = 0;
    break;
  case WindingReference::Secondary:
    mReferenceTerminal = 1;
    break;
  case WindingReference::Tertiary:
    throw std::invalid_argument("Three-winding transformers are unsupported.");
  case WindingReference::Auto:
    mReferenceTerminal =
        (mNominalVoltagePrimary >= mNominalVoltageSecondary) ? 0 : 1;
    break;
  }
  mOrientationSign = (mReferenceTerminal == 0) ? 1. : -1.;
  mRatioFromReference =
      (mReferenceTerminal == 0) ? effectiveRatio() : 1. / effectiveRatio();
}

void DP::Ph3::SSN::Transformer::initializeFromNodesAndTerminals(
    Real frequency) {
  resolveWindingRoles();
  const Real omega = 2. * PI * frequency;
  const Real magnetizingSusceptance =
      std::sqrt(mNoLoadCurrent * mNoLoadCurrent - mNoLoadLoss * mNoLoadLoss) *
      mRatedPower / std::pow(nominalVoltageAt(mReferenceTerminal), 2);
  const Real magnetizingInductance = 1. / (omega * magnetizingSusceptance);
  const Real magnetizingResistance =
      std::pow(nominalVoltageAt(mReferenceTerminal), 2) /
      (mNoLoadLoss * mRatedPower);
  mMagnetizingInductance =
      Math::singlePhaseParameterToThreePhase(magnetizingInductance);
  mMagnetizingResistance =
      Math::singlePhaseParameterToThreePhase(magnetizingResistance);
  mMagnetizingConductance = mMagnetizingResistance.inverse().cast<Complex>();

  mSeriesInductanceHalf = mInductance / 2.;
  mSeriesResistanceHalf.setZero();
  if (mWithResistiveLosses)
    mSeriesResistanceHalf = mResistance / 2.;

  const PhaseVector voltage0 =
      phaseVoltage(RMS3PH_TO_PEAK1PH * initialSingleVoltage(0));
  const PhaseVector voltage1 =
      phaseVoltage(RMS3PH_TO_PEAK1PH * initialSingleVoltage(1));
  const PhaseVector referenceVoltage =
      (mReferenceTerminal == 0) ? voltage0 : voltage1;
  const PhaseVector otherVoltage =
      (mReferenceTerminal == 0) ? voltage1 : voltage0;
  const PhaseVector referredOtherVoltage = mRatioFromReference * otherVoltage;

  const MatrixComp halfImpedance =
      mSeriesResistanceHalf.cast<Complex>() +
      Complex(0., omega) * mSeriesInductanceHalf.cast<Complex>();
  const MatrixComp seriesAdmittance = halfImpedance.inverse();
  const MatrixComp magnetizingAdmittance =
      mMagnetizingConductance.cast<Complex>() +
      Complex(0., -1. / omega) *
          mMagnetizingInductance.inverse().cast<Complex>();
  const MatrixComp midpointInverse =
      (2. * seriesAdmittance + magnetizingAdmittance).inverse();
  const PhaseVector midpointVoltage = midpointInverse * seriesAdmittance *
                                      (referenceVoltage + referredOtherVoltage);
  const PhaseVector current0 =
      seriesAdmittance * (midpointVoltage - referenceVoltage);
  const PhaseVector current1 =
      seriesAdmittance * (referredOtherVoltage - midpointVoltage);
  const PhaseVector currentMagnetizing =
      (magnetizingAdmittance - mMagnetizingConductance.cast<Complex>()) *
      (-midpointVoltage);

  const PhaseVector voltageL0 =
      Complex(0., omega) * mSeriesInductanceHalf.cast<Complex>() * current0;
  const PhaseVector voltageL1 =
      Complex(0., omega) * mSeriesInductanceHalf.cast<Complex>() * current1;
  const PhaseVector voltageLMag = Complex(0., omega) *
                                  mMagnetizingInductance.cast<Complex>() *
                                  currentMagnetizing;
  (**mState).block<3, 1>(kVoltage0, 0) = voltageL0;
  (**mState).block<3, 1>(kCurrent0, 0) = current0;
  (**mState).block<3, 1>(kVoltage1, 0) = voltageL1;
  (**mState).block<3, 1>(kCurrent1, 0) = current1;
  (**mState).block<3, 1>(kVoltageMag, 0) = voltageLMag;
  (**mState).block<3, 1>(kCurrentMag, 0) = currentMagnetizing;
  **mIntfVoltage = voltage1 - voltage0;
  **mIntfCurrent = mOrientationSign * current0;
}

void DP::Ph3::SSN::Transformer::mnaCompInitialize(Real omega, Real timeStep,
                                                  Attribute<Matrix>::Ptr) {
  updateMatrixNodeIndices();
  const Real b = timeStep * omega / 2.;
  mInductorAdmittance =
      ((timeStep / 2.) * mSeriesInductanceHalf.inverse()).cast<Complex>() /
      Complex(1., b);
  mPreviousCurrentFactor =
      Complex((1. - b * b) / (1. + b * b), -2. * b / (1. + b * b));
  const MatrixComp identity = MatrixComp::Identity(3, 3);
  mSeriesSystemInverse =
      (identity + mInductorAdmittance * mSeriesResistanceHalf.cast<Complex>())
          .inverse();
  mSeriesAdmittance = mSeriesSystemInverse * mInductorAdmittance;
  mMagnetizingInductorAdmittance =
      ((timeStep / 2.) * mMagnetizingInductance.inverse()).cast<Complex>() /
      Complex(1., b);
  mMidpointInverse = (2. * mSeriesAdmittance + mMagnetizingInductorAdmittance +
                      mMagnetizingConductance.cast<Complex>())
                         .inverse();

  const MatrixComp diagonal =
      mSeriesAdmittance * mMidpointInverse * mSeriesAdmittance;
  mPortAdmittance00 = mSeriesAdmittance - diagonal;
  mPortAdmittance01 = -diagonal * mRatioFromReference;
  mPortAdmittance10 = -std::conj(mRatioFromReference) * diagonal;
  mPortAdmittance11 = std::norm(mRatioFromReference) * mPortAdmittance00;
}

void DP::Ph3::SSN::Transformer::mnaCompApplySystemMatrixStamp(
    SparseMatrixRow &systemMatrix) {
  const MatrixComp *blocks[2][2] = {{&mPortAdmittance00, &mPortAdmittance01},
                                    {&mPortAdmittance10, &mPortAdmittance11}};
  for (UInt rowPort = 0; rowPort < 2; ++rowPort) {
    const UInt rowTerminal = (mReferenceTerminal == 0) ? rowPort : 1 - rowPort;
    if (!terminalNotGrounded(rowTerminal))
      continue;
    for (UInt colPort = 0; colPort < 2; ++colPort) {
      const UInt colTerminal =
          (mReferenceTerminal == 0) ? colPort : 1 - colPort;
      if (!terminalNotGrounded(colTerminal))
        continue;
      for (UInt rowPhase = 0; rowPhase < 3; ++rowPhase)
        for (UInt colPhase = 0; colPhase < 3; ++colPhase)
          Math::addToMatrixElement(
              systemMatrix, matrixNodeIndex(rowTerminal, rowPhase),
              matrixNodeIndex(colTerminal, colPhase),
              (*blocks[rowPort][colPort])(rowPhase, colPhase));
    }
  }
}

void DP::Ph3::SSN::Transformer::updateHistory() {
  const auto &state = **mState;
  const PhaseVector raw0 =
      mInductorAdmittance * state.block<3, 1>(kVoltage0, 0) +
      mPreviousCurrentFactor * state.block<3, 1>(kCurrent0, 0);
  const PhaseVector raw1 =
      mInductorAdmittance * state.block<3, 1>(kVoltage1, 0) +
      mPreviousCurrentFactor * state.block<3, 1>(kCurrent1, 0);
  mHistory0 = mSeriesSystemInverse * raw0;
  mHistory1 = mSeriesSystemInverse * raw1;
  mHistoryMagnetizing =
      mMagnetizingInductorAdmittance * state.block<3, 1>(kVoltageMag, 0) +
      mPreviousCurrentFactor * state.block<3, 1>(kCurrentMag, 0);
}

void DP::Ph3::SSN::Transformer::mnaCompApplyRightSideVectorStamp(
    Matrix &rightVector) {
  const PhaseVector midpointHistory =
      -mHistory0 + mHistory1 + mHistoryMagnetizing;
  const PhaseVector midpointSource = mMidpointInverse * midpointHistory;
  const PhaseVector source0 = mHistory0 + mSeriesAdmittance * midpointSource;
  const PhaseVector source1 = std::conj(mRatioFromReference) *
                              (mSeriesAdmittance * midpointSource - mHistory1);
  const UInt referenceTerminal = mReferenceTerminal;
  const UInt otherTerminal = nonReferenceTerminal();
  if (terminalNotGrounded(referenceTerminal))
    for (UInt phase = 0; phase < 3; ++phase)
      Math::setVectorElement(rightVector,
                             matrixNodeIndex(referenceTerminal, phase),
                             source0(phase, 0));
  if (terminalNotGrounded(otherTerminal))
    for (UInt phase = 0; phase < 3; ++phase)
      Math::setVectorElement(rightVector, matrixNodeIndex(otherTerminal, phase),
                             source1(phase, 0));
}

void DP::Ph3::SSN::Transformer::mnaCompPreStep(Real, Int) {
  updateHistory();
  mnaCompApplyRightSideVectorStamp(**mRightVector);
}

DP::Ph3::SSN::Transformer::PhaseVector
DP::Ph3::SSN::Transformer::terminalVoltage(const Matrix &leftVector,
                                           UInt terminal) {
  PhaseVector voltage = PhaseVector::Zero();
  if (terminalNotGrounded(terminal))
    for (UInt phase = 0; phase < 3; ++phase)
      voltage(phase) = Math::complexFromVectorElement(
          leftVector, matrixNodeIndex(terminal, phase));
  return voltage;
}

void DP::Ph3::SSN::Transformer::mnaCompPostStep(
    Real, Int, Attribute<Matrix>::Ptr &leftVector) {
  const PhaseVector voltage0 = terminalVoltage(**leftVector, 0);
  const PhaseVector voltage1 = terminalVoltage(**leftVector, 1);
  const PhaseVector referenceVoltage =
      (mReferenceTerminal == 0) ? voltage0 : voltage1;
  const PhaseVector otherVoltage =
      (mReferenceTerminal == 0) ? voltage1 : voltage0;
  const PhaseVector referredOtherVoltage = mRatioFromReference * otherVoltage;
  const PhaseVector midpointHistory =
      -mHistory0 + mHistory1 + mHistoryMagnetizing;
  const PhaseVector midpointVoltage =
      mMidpointInverse *
      (mSeriesAdmittance * referenceVoltage +
       mSeriesAdmittance * referredOtherVoltage + midpointHistory);

  const PhaseVector totalVoltage0 = midpointVoltage - referenceVoltage;
  const PhaseVector current0 = mSeriesAdmittance * totalVoltage0 + mHistory0;
  const PhaseVector voltageL0 =
      totalVoltage0 - mSeriesResistanceHalf.cast<Complex>() * current0;
  const PhaseVector totalVoltage1 = referredOtherVoltage - midpointVoltage;
  const PhaseVector current1 = mSeriesAdmittance * totalVoltage1 + mHistory1;
  const PhaseVector voltageL1 =
      totalVoltage1 - mSeriesResistanceHalf.cast<Complex>() * current1;
  const PhaseVector voltageLMag = -midpointVoltage;
  const PhaseVector currentMagnetizing =
      mMagnetizingInductorAdmittance * voltageLMag + mHistoryMagnetizing;

  (**mState).block<3, 1>(kVoltage0, 0) = voltageL0;
  (**mState).block<3, 1>(kCurrent0, 0) = current0;
  (**mState).block<3, 1>(kVoltage1, 0) = voltageL1;
  (**mState).block<3, 1>(kCurrent1, 0) = current1;
  (**mState).block<3, 1>(kVoltageMag, 0) = voltageLMag;
  (**mState).block<3, 1>(kCurrentMag, 0) = currentMagnetizing;
  **mIntfVoltage = voltage1 - voltage0;
  **mIntfCurrent = mOrientationSign * current0;
}

void DP::Ph3::SSN::Transformer::mnaCompAddPreStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes) {
  prevStepDependencies.push_back(mState);
  modifiedAttributes.push_back(mRightVector);
}

void DP::Ph3::SSN::Transformer::mnaCompAddPostStepDependencies(
    AttributeBase::List &, AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes,
    Attribute<Matrix>::Ptr &leftVector) {
  attributeDependencies.push_back(leftVector);
  modifiedAttributes.push_back(mIntfVoltage);
  modifiedAttributes.push_back(mIntfCurrent);
  modifiedAttributes.push_back(mState);
}
