/* SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
 * SPDX-License-Identifier: MPL-2.0
 */

#include <dpsim-models/EMT/EMT_Ph3_SSN_Transformer.h>

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

using PhaseVector = Eigen::Matrix<Real, 3, 1>;
using PhaseVectorComp = Eigen::Matrix<Complex, 3, 1>;

PhaseVectorComp phaseVoltage(Complex phaseA) {
  PhaseVectorComp voltage;
  voltage(0) = phaseA;
  voltage(1) = phaseA * SHIFT_TO_PHASE_B;
  voltage(2) = phaseA * SHIFT_TO_PHASE_C;
  return voltage;
}
} // namespace

EMT::Ph3::SSN::Transformer::Transformer(String uid, String name,
                                        Logger::Level logLevel,
                                        Bool withResistiveLosses)
    : MNASimPowerComp<Real>(uid, name, true, true, logLevel),
      Base::Ph3::Transformer(mAttributes),
      mState(mAttributes->create<Matrix>("state")), mStateValue(&mState->get()),
      mSeriesResistanceHalf(Matrix::Zero(3, 3)),
      mSeriesInductanceHalf(Matrix::Zero(3, 3)),
      mInductorAdmittance(Matrix::Zero(3, 3)),
      mSeriesSystemInverse(Matrix::Identity(3, 3)),
      mSeriesAdmittance(Matrix::Zero(3, 3)),
      mMagnetizingInductorAdmittance(Matrix::Zero(3, 3)),
      mMagnetizingConductance(Matrix::Zero(3, 3)),
      mMidpointInverse(Matrix::Zero(3, 3)),
      mPortAdmittance00(Matrix::Zero(3, 3)),
      mPortAdmittance01(Matrix::Zero(3, 3)),
      mPortAdmittance10(Matrix::Zero(3, 3)),
      mPortAdmittance11(Matrix::Zero(3, 3)), mHistory0(Matrix::Zero(3, 1)),
      mHistory1(Matrix::Zero(3, 1)), mHistoryMagnetizing(Matrix::Zero(3, 1)),
      mMidpointHistory(Matrix::Zero(3, 1)), mMidpointSource(Matrix::Zero(3, 1)),
      mSource0(Matrix::Zero(3, 1)), mSource1(Matrix::Zero(3, 1)),
      mMagnetizingResistance(Matrix::Zero(3, 3)),
      mMagnetizingInductance(Matrix::Zero(3, 3)),
      mWithResistiveLosses(withResistiveLosses) {
  mPhaseType = PhaseType::ABC;
  setTerminalNumber(2);
  *mIntfVoltageValue = Matrix::Zero(3, 1);
  *mIntfCurrentValue = Matrix::Zero(3, 1);
  *mStateValue = Matrix::Zero(18, 1);
}

SimPowerComp<Real>::Ptr EMT::Ph3::SSN::Transformer::clone(String copySuffix) {
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

void EMT::Ph3::SSN::Transformer::setParameters(
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

void EMT::Ph3::SSN::Transformer::setMagnetizingBranch(Real noLoadCurrent,
                                                      Real noLoadLoss) {
  if (!std::isfinite(noLoadCurrent) || !std::isfinite(noLoadLoss) ||
      noLoadCurrent <= noLoadLoss || noLoadLoss <= 0.)
    throw std::invalid_argument("Invalid transformer magnetizing branch.");
  mNoLoadCurrent = noLoadCurrent;
  mNoLoadLoss = noLoadLoss;
}

void EMT::Ph3::SSN::Transformer::setReferenceWinding(
    WindingReference referenceWinding) {
  if (referenceWinding == WindingReference::Tertiary)
    throw std::invalid_argument("Three-winding transformers are unsupported.");
  mReferenceWinding = referenceWinding;
}

void EMT::Ph3::SSN::Transformer::resolveWindingRoles() {
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
  if (std::abs(std::arg(mRatioFromReference)) > 1e-9)
    throw std::invalid_argument(
        "EMT SSN transformers support in-phase turns ratios only.");
}

void EMT::Ph3::SSN::Transformer::initializeFromNodesAndTerminals(
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
  mMagnetizingConductance = mMagnetizingResistance.inverse();

  mSeriesInductanceHalf = mInductance / 2.;
  mSeriesResistanceHalf.setZero();
  if (mWithResistiveLosses)
    mSeriesResistanceHalf = mResistance / 2.;

  const PhaseVectorComp voltage0 =
      phaseVoltage(RMS3PH_TO_PEAK1PH * initialSingleVoltage(0));
  const PhaseVectorComp voltage1 =
      phaseVoltage(RMS3PH_TO_PEAK1PH * initialSingleVoltage(1));
  const PhaseVectorComp referenceVoltage =
      (mReferenceTerminal == 0) ? voltage0 : voltage1;
  const PhaseVectorComp otherVoltage =
      (mReferenceTerminal == 0) ? voltage1 : voltage0;
  const PhaseVectorComp referredOtherVoltage =
      mRatioFromReference.real() * otherVoltage;

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
  const PhaseVectorComp midpointVoltage =
      midpointInverse * seriesAdmittance *
      (referenceVoltage + referredOtherVoltage);
  const PhaseVectorComp current0 =
      seriesAdmittance * (midpointVoltage - referenceVoltage);
  const PhaseVectorComp current1 =
      seriesAdmittance * (referredOtherVoltage - midpointVoltage);
  const PhaseVectorComp currentMagnetizing =
      (magnetizingAdmittance - mMagnetizingConductance.cast<Complex>()) *
      (-midpointVoltage);

  const PhaseVectorComp voltageL0 =
      Complex(0., omega) * mSeriesInductanceHalf.cast<Complex>() * current0;
  const PhaseVectorComp voltageL1 =
      Complex(0., omega) * mSeriesInductanceHalf.cast<Complex>() * current1;
  const PhaseVectorComp voltageLMag = Complex(0., omega) *
                                      mMagnetizingInductance.cast<Complex>() *
                                      currentMagnetizing;
  mStateValue->block<3, 1>(kVoltage0, 0) = voltageL0.real();
  mStateValue->block<3, 1>(kCurrent0, 0) = current0.real();
  mStateValue->block<3, 1>(kVoltage1, 0) = voltageL1.real();
  mStateValue->block<3, 1>(kCurrent1, 0) = current1.real();
  mStateValue->block<3, 1>(kVoltageMag, 0) = voltageLMag.real();
  mStateValue->block<3, 1>(kCurrentMag, 0) = currentMagnetizing.real();
  *mIntfVoltageValue = (voltage1 - voltage0).real();
  *mIntfCurrentValue = mOrientationSign * current0.real();
}

void EMT::Ph3::SSN::Transformer::exportFusedModel(LinearSSNModel &model) {
  model = LinearSSNModel{};
  model.kind = LinearSSNModel::Kind::Transformer;
  model.terminalCount = 2;
  for (UInt terminal = 0; terminal < 2; ++terminal) {
    model.grounded[terminal] = !terminalNotGrounded(terminal);
    for (UInt phase = 0; phase < 3; ++phase)
      model.nodeIndices[terminal * 3 + phase] =
          matrixNodeIndex(terminal, phase);
    for (UInt phase = 0; phase < 3; ++phase)
      model.nodeNames[terminal * 3 + phase] = node(terminal)->name();
  }
  model.state = *mStateValue;
  model.seriesResistanceHalf = mSeriesResistanceHalf;
  model.seriesSystemInverse = mSeriesSystemInverse;
  model.inductorAdmittance = mInductorAdmittance;
  model.seriesAdmittance = mSeriesAdmittance;
  model.magnetizingInductorAdmittance = mMagnetizingInductorAdmittance;
  model.magnetizingConductance = mMagnetizingConductance;
  model.midpointInverse = mMidpointInverse;
  model.referenceTerminal = mReferenceTerminal;
  model.ratioFromReference = mRatioFromReference.real();
  model.orientationSign = mOrientationSign;
  model.portAdmittance = Matrix::Zero(6, 6);
  if (mReferenceTerminal == 0) {
    model.portAdmittance.block<3, 3>(0, 0) = mPortAdmittance00;
    model.portAdmittance.block<3, 3>(0, 3) = mPortAdmittance01;
    model.portAdmittance.block<3, 3>(3, 0) = mPortAdmittance10;
    model.portAdmittance.block<3, 3>(3, 3) = mPortAdmittance11;
  } else {
    model.portAdmittance.block<3, 3>(0, 0) = mPortAdmittance11;
    model.portAdmittance.block<3, 3>(0, 3) = mPortAdmittance10;
    model.portAdmittance.block<3, 3>(3, 0) = mPortAdmittance01;
    model.portAdmittance.block<3, 3>(3, 3) = mPortAdmittance00;
  }
}

void EMT::Ph3::SSN::Transformer::mnaCompInitialize(Real, Real timeStep,
                                                   Attribute<Matrix>::Ptr) {
  updateMatrixNodeIndices();
  mInductorAdmittance = (timeStep / 2.) * mSeriesInductanceHalf.inverse();
  mSeriesSystemInverse =
      (Matrix::Identity(3, 3) + mInductorAdmittance * mSeriesResistanceHalf)
          .inverse();
  mSeriesAdmittance = mSeriesSystemInverse * mInductorAdmittance;
  mMagnetizingInductorAdmittance =
      (timeStep / 2.) * mMagnetizingInductance.inverse();
  mMidpointInverse = (2. * mSeriesAdmittance + mMagnetizingInductorAdmittance +
                      mMagnetizingConductance)
                         .inverse();

  const Matrix diagonal =
      mSeriesAdmittance * mMidpointInverse * mSeriesAdmittance;
  mPortAdmittance00 = mSeriesAdmittance - diagonal;
  mPortAdmittance01 = -diagonal * mRatioFromReference.real();
  mPortAdmittance10 = mPortAdmittance01;
  mPortAdmittance11 = std::norm(mRatioFromReference) * mPortAdmittance00;
}

void EMT::Ph3::SSN::Transformer::mnaCompApplySystemMatrixStamp(
    SparseMatrixRow &systemMatrix) {
  const Matrix *blocks[2][2] = {{&mPortAdmittance00, &mPortAdmittance01},
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

void EMT::Ph3::SSN::Transformer::updateHistory() {
  const auto &state = *mStateValue;
  mHistory0 = mSeriesSystemInverse *
              (mInductorAdmittance * state.block<3, 1>(kVoltage0, 0) +
               state.block<3, 1>(kCurrent0, 0));
  mHistory1 = mSeriesSystemInverse *
              (mInductorAdmittance * state.block<3, 1>(kVoltage1, 0) +
               state.block<3, 1>(kCurrent1, 0));
  mHistoryMagnetizing =
      mMagnetizingInductorAdmittance * state.block<3, 1>(kVoltageMag, 0) +
      state.block<3, 1>(kCurrentMag, 0);
}

void EMT::Ph3::SSN::Transformer::mnaCompApplyRightSideVectorStamp(
    Matrix &rightVector) {
  mMidpointHistory = -mHistory0 + mHistory1 + mHistoryMagnetizing;
  mMidpointSource = mMidpointInverse * mMidpointHistory;
  mSource0 = mHistory0 + mSeriesAdmittance * mMidpointSource;
  mSource1 = mRatioFromReference.real() *
             (mSeriesAdmittance * mMidpointSource - mHistory1);
  const UInt referenceTerminal = mReferenceTerminal;
  const UInt otherTerminal = nonReferenceTerminal();
  if (terminalNotGrounded(referenceTerminal))
    for (UInt phase = 0; phase < 3; ++phase)
      Math::setVectorElement(rightVector,
                             matrixNodeIndex(referenceTerminal, phase),
                             mSource0(phase, 0));
  if (terminalNotGrounded(otherTerminal))
    for (UInt phase = 0; phase < 3; ++phase)
      Math::setVectorElement(rightVector, matrixNodeIndex(otherTerminal, phase),
                             mSource1(phase, 0));
}

void EMT::Ph3::SSN::Transformer::mnaAddCompactRightSideVectorStamp(
    Matrix &rightVector) {
  const UInt referenceTerminal = mReferenceTerminal;
  const UInt otherTerminal = nonReferenceTerminal();
  if (terminalNotGrounded(referenceTerminal))
    for (UInt phase = 0; phase < 3; ++phase)
      Math::addToVectorElement(rightVector,
                               matrixNodeIndex(referenceTerminal, phase),
                               mSource0(phase, 0));
  if (terminalNotGrounded(otherTerminal))
    for (UInt phase = 0; phase < 3; ++phase)
      Math::addToVectorElement(rightVector,
                               matrixNodeIndex(otherTerminal, phase),
                               mSource1(phase, 0));
}

void EMT::Ph3::SSN::Transformer::mnaCompPreStep(Real, Int) {
  updateHistory();
  mnaCompApplyRightSideVectorStamp(*mRightVectorValue);
}

EMT::Ph3::SSN::Transformer::PhaseVector
EMT::Ph3::SSN::Transformer::terminalVoltage(const Matrix &leftVector,
                                            UInt terminal) {
  PhaseVector voltage = PhaseVector::Zero();
  if (terminalNotGrounded(terminal))
    for (UInt phase = 0; phase < 3; ++phase)
      voltage(phase) = Math::realFromVectorElement(
          leftVector, matrixNodeIndex(terminal, phase));
  return voltage;
}

void EMT::Ph3::SSN::Transformer::mnaCompPostStep(
    Real, Int, Attribute<Matrix>::Ptr &leftVector) {
  const PhaseVector voltage0 = terminalVoltage(**leftVector, 0);
  const PhaseVector voltage1 = terminalVoltage(**leftVector, 1);
  const PhaseVector referenceVoltage =
      (mReferenceTerminal == 0) ? voltage0 : voltage1;
  const PhaseVector otherVoltage =
      (mReferenceTerminal == 0) ? voltage1 : voltage0;
  const PhaseVector referredOtherVoltage =
      mRatioFromReference.real() * otherVoltage;
  const PhaseVector midpointHistory =
      -mHistory0 + mHistory1 + mHistoryMagnetizing;
  const PhaseVector midpointVoltage =
      mMidpointInverse *
      (mSeriesAdmittance * referenceVoltage +
       mSeriesAdmittance * referredOtherVoltage + midpointHistory);

  const PhaseVector totalVoltage0 = midpointVoltage - referenceVoltage;
  const PhaseVector current0 = mSeriesAdmittance * totalVoltage0 + mHistory0;
  const PhaseVector voltageL0 =
      totalVoltage0 - mSeriesResistanceHalf * current0;
  const PhaseVector totalVoltage1 = referredOtherVoltage - midpointVoltage;
  const PhaseVector current1 = mSeriesAdmittance * totalVoltage1 + mHistory1;
  const PhaseVector voltageL1 =
      totalVoltage1 - mSeriesResistanceHalf * current1;
  const PhaseVector voltageLMag = -midpointVoltage;
  const PhaseVector currentMagnetizing =
      mMagnetizingInductorAdmittance * voltageLMag + mHistoryMagnetizing;

  mStateValue->block<3, 1>(kVoltage0, 0) = voltageL0;
  mStateValue->block<3, 1>(kCurrent0, 0) = current0;
  mStateValue->block<3, 1>(kVoltage1, 0) = voltageL1;
  mStateValue->block<3, 1>(kCurrent1, 0) = current1;
  mStateValue->block<3, 1>(kVoltageMag, 0) = voltageLMag;
  mStateValue->block<3, 1>(kCurrentMag, 0) = currentMagnetizing;
  *mIntfVoltageValue = voltage1 - voltage0;
  *mIntfCurrentValue = mOrientationSign * current0;
}

void EMT::Ph3::SSN::Transformer::mnaCompAddPreStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes) {
  prevStepDependencies.push_back(mState);
  modifiedAttributes.push_back(mRightVector);
}

void EMT::Ph3::SSN::Transformer::mnaCompAddPostStepDependencies(
    AttributeBase::List &, AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes,
    Attribute<Matrix>::Ptr &leftVector) {
  attributeDependencies.push_back(leftVector);
  modifiedAttributes.push_back(mIntfVoltage);
  modifiedAttributes.push_back(mIntfCurrent);
  modifiedAttributes.push_back(mState);
}
