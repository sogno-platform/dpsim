/* SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
 * SPDX-License-Identifier: MPL-2.0
 */

#include <dpsim-models/EMT/EMT_Ph3_SSN_Transformer.h>

#include <cmath>
#include <stdexcept>

using namespace CPS;

EMT::Ph3::SSN::Transformer::Transformer(String uid, String name,
                                        Logger::Level logLevel,
                                        Bool withResistiveLosses)
    : MultiPortVTypeSSNComp(uid, name, logLevel),
      Base::Ph3::Transformer(mAttributes),
      mSeriesResistanceHalf(Matrix::Zero(3, 3)),
      mSeriesInductanceHalf(Matrix::Zero(3, 3)),
      mMagnetizingResistance(Matrix::Zero(3, 3)),
      mMagnetizingInductance(Matrix::Zero(3, 3)),
      mWithResistiveLosses(withResistiveLosses) {}

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
  const Matrix zero = Matrix::Zero(6, 6);
  SSNComp::setParameters(zero, zero, zero, zero);
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
  mRatioFromReference =
      (mReferenceTerminal == 0) ? effectiveRatio() : 1. / effectiveRatio();
  if (std::abs(std::arg(mRatioFromReference)) > 1e-9)
    throw std::invalid_argument(
        "EMT SSN transformers support in-phase turns ratios only.");
}

void EMT::Ph3::SSN::Transformer::initializeFromNodesAndTerminals(
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
  const Matrix magnetizingI = mMagnetizingInductance;
  const Matrix magnetizingR = mMagnetizingResistance;
  mass.block<3, 3>(0, 0) = mSeriesInductanceHalf + magnetizingI;
  mass.block<3, 3>(0, 3) = magnetizingI;
  mass.block<3, 3>(3, 0) = magnetizingI;
  mass.block<3, 3>(3, 3) = mSeriesInductanceHalf + magnetizingI;
  resistance.block<3, 3>(0, 0) = mSeriesResistanceHalf + magnetizingR;
  resistance.block<3, 3>(0, 3) = magnetizingR;
  resistance.block<3, 3>(3, 0) = magnetizingR;
  resistance.block<3, 3>(3, 3) = mSeriesResistanceHalf + magnetizingR;

  const Matrix inverseMass = mass.inverse();
  Matrix inputMap = Matrix::Zero(6, 6);
  Matrix outputMap = Matrix::Zero(6, 6);
  const Matrix identity = Matrix::Identity(3, 3);
  const Real ratio = mRatioFromReference.real();
  if (mReferenceTerminal == 0) {
    inputMap.block<3, 3>(0, 0) = identity;
    inputMap.block<3, 3>(3, 3) = ratio * identity;
    outputMap.block<3, 3>(0, 0) = identity;
    outputMap.block<3, 3>(3, 3) = ratio * identity;
  } else {
    inputMap.block<3, 3>(0, 3) = identity;
    inputMap.block<3, 3>(3, 0) = ratio * identity;
    outputMap.block<3, 3>(0, 3) = ratio * identity;
    outputMap.block<3, 3>(3, 0) = identity;
  }
  SSNComp::setParameters(-inverseMass * resistance, inverseMass * inputMap,
                         outputMap, Matrix::Zero(6, 6));
  MultiPortVTypeSSNComp::initializeFromNodesAndTerminals(frequency);
}
