/* SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
 * SPDX-License-Identifier: MPL-2.0
 */

#include <dpsim-models/DP/DP_Ph3_SSN_Transformer.h>

#include <cmath>
#include <stdexcept>

using namespace CPS;

DP::Ph3::SSN::Transformer::Transformer(String uid, String name,
                                       Logger::Level logLevel,
                                       Bool withResistiveLosses)
    : MultiPortVTypeSSNComp(uid, name, logLevel),
      Base::Ph3::Transformer(mAttributes),
      mSeriesResistanceHalf(Matrix::Zero(3, 3)),
      mSeriesInductanceHalf(Matrix::Zero(3, 3)),
      mMagnetizingResistance(Matrix::Zero(3, 3)),
      mMagnetizingInductance(Matrix::Zero(3, 3)),
      mWithResistiveLosses(withResistiveLosses) {
  const Matrix zero = Matrix::Zero(6, 6);
  SSNComp::setParameters(zero, zero, zero, zero);
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
  const Matrix zero = Matrix::Zero(6, 6);
  SSNComp::setParameters(zero, zero, zero, zero);
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
  mRatioFromReference =
      (mReferenceTerminal == 0) ? effectiveRatio() : 1. / effectiveRatio();
}

void DP::Ph3::SSN::Transformer::initializeFromNodesAndTerminals(
    Real frequency) {
  if (frequency <= 0.)
    throw std::invalid_argument("Frequency must be positive.");
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

  mSeriesInductanceHalf = mInductance / 2.;
  mSeriesResistanceHalf.setZero();
  if (mWithResistiveLosses)
    mSeriesResistanceHalf = mResistance / 2.;

  // State x = [i_ref; i_other], with the magnetizing current eliminated by
  // midpoint KCL: i_mag = i_ref + i_other.
  Matrix mass = Matrix::Zero(6, 6);
  Matrix resistance = Matrix::Zero(6, 6);
  mass.block<3, 3>(0, 0) = mSeriesInductanceHalf + mMagnetizingInductance;
  mass.block<3, 3>(0, 3) = mMagnetizingInductance;
  mass.block<3, 3>(3, 0) = mMagnetizingInductance;
  mass.block<3, 3>(3, 3) = mSeriesInductanceHalf + mMagnetizingInductance;
  resistance.block<3, 3>(0, 0) = mSeriesResistanceHalf + mMagnetizingResistance;
  resistance.block<3, 3>(0, 3) = mMagnetizingResistance;
  resistance.block<3, 3>(3, 0) = mMagnetizingResistance;
  resistance.block<3, 3>(3, 3) = mSeriesResistanceHalf + mMagnetizingResistance;

  const Matrix inverseMass = mass.inverse();
  const Matrix identity = Matrix::Identity(3, 3);
  const MatrixComp identityComp = identity.cast<Complex>();
  mInputTransform.setZero();
  mOutputTransform.setZero();
  if (mReferenceTerminal == 0) {
    mInputTransform.block<3, 3>(0, 0) = identityComp;
    mInputTransform.block<3, 3>(3, 3) = mRatioFromReference * identityComp;
    mOutputTransform.block<3, 3>(0, 0) = identityComp;
    mOutputTransform.block<3, 3>(3, 3) =
        std::conj(mRatioFromReference) * identityComp;
  } else {
    mInputTransform.block<3, 3>(0, 3) = identityComp;
    mInputTransform.block<3, 3>(3, 0) = mRatioFromReference * identityComp;
    mOutputTransform.block<3, 3>(0, 3) =
        std::conj(mRatioFromReference) * identityComp;
    mOutputTransform.block<3, 3>(3, 0) = identityComp;
  }

  SSNComp::setParameters(-inverseMass * resistance, inverseMass,
                         Matrix::Identity(6, 6), Matrix::Zero(6, 6));
  MultiPortVTypeSSNComp::initializeFromNodesAndTerminals(frequency);
}

void DP::Ph3::SSN::Transformer::recomputeDiscreteModel(Real omega) {
  SSNComp::recomputeDiscreteModel(omega);
  mDiscreteB *= mInputTransform;
  mW =
      mOutputTransform * (mC.cast<Complex>() * mDiscreteB + mD.cast<Complex>());
}

MatrixComp DP::Ph3::SSN::Transformer::calculateHistoryVector() const {
  MatrixComp stateAndInput =
      mDiscreteA * (**mX) + mDiscreteB * (**inputAttribute());
  return mOutputTransform * mC.cast<Complex>() * stateAndInput;
}

void DP::Ph3::SSN::Transformer::updateHistoryVector(MatrixComp &history) {
  mHistoryState.noalias() = mDiscreteA * (**mX);
  mHistoryInput.noalias() = mDiscreteB * (**inputAttribute());
  mHistoryState += mHistoryInput;
  history.noalias() = mOutputTransform * mComplexC * mHistoryState;
}

MatrixComp DP::Ph3::SSN::Transformer::calculateSteadyStateStateFromInput(
    const MatrixComp &u, Real omega) const {
  return SSNComp::calculateSteadyStateStateFromInput(mInputTransform * u,
                                                     omega);
}

MatrixComp DP::Ph3::SSN::Transformer::calculateSteadyStateOutputFromInput(
    const MatrixComp &x, const MatrixComp &u) const {
  return mOutputTransform *
         SSNComp::calculateSteadyStateOutputFromInput(x, mInputTransform * u);
}
