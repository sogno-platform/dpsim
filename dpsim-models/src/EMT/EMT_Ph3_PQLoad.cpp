/* Copyright 2017-2026 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/EMT/EMT_Ph3_PQLoad.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>

using namespace CPS;

EMT::Ph3::PQLoad::PQLoad(String uid, String name, Logger::Level logLevel)
    : MNASimPowerComp<Real>(uid, name, true, true, logLevel),
      mActivePower(mAttributes->create<Real>("P", 0.0)),
      mReactivePower(mAttributes->create<Real>("Q", 0.0)),
      mNomVoltage(mAttributes->create<Real>("V_nom", 0.0)),
      mMinimumVoltagePerUnit(mAttributes->create<Real>("V_min_pu", 0.1)),
      mMeasuredActivePower(mAttributes->create<Real>("p_inst", 0.0)),
      mMeasuredReactivePower(mAttributes->create<Real>("q_inst", 0.0)) {
  mPhaseType = PhaseType::ABC;
  setTerminalNumber(1);

  **mIntfVoltage = Matrix::Zero(3, 1);
  **mIntfCurrent = Matrix::Zero(3, 1);
  mNumIter = mAttributes->create<Int>("NIterations", 0);

  SPDLOG_LOGGER_INFO(mSLog, "Create {} {}", this->type(), name);
}

EMT::Ph3::PQLoad::PQLoad(String name, Logger::Level logLevel)
    : PQLoad(name, name, logLevel) {}

EMT::Ph3::PQLoad::PQLoad(String name, Real activePower, Real reactivePower,
                         Real nominalVoltage, Logger::Level logLevel)
    : PQLoad(name, name, activePower, reactivePower, nominalVoltage, logLevel) {
}

EMT::Ph3::PQLoad::PQLoad(String uid, String name, Real activePower,
                         Real reactivePower, Real nominalVoltage,
                         Logger::Level logLevel)
    : PQLoad(uid, name, logLevel) {
  setParameters(activePower, reactivePower, nominalVoltage);
}

void EMT::Ph3::PQLoad::setParameters(Real activePower, Real reactivePower,
                                     Real nominalVoltage,
                                     Real minimumVoltagePerUnit) {
  if (!std::isfinite(activePower) || !std::isfinite(reactivePower))
    throw std::invalid_argument("PQLoad power set points must be finite.");
  if (!std::isfinite(nominalVoltage) || nominalVoltage <= 0.0)
    throw std::invalid_argument("PQLoad nominal voltage must be positive.");
  if (!std::isfinite(minimumVoltagePerUnit) || minimumVoltagePerUnit <= 0.0)
    throw std::invalid_argument(
        "PQLoad minimum per-unit voltage must be positive.");

  **mActivePower = activePower;
  **mReactivePower = reactivePower;
  **mNomVoltage = nominalVoltage;
  **mMinimumVoltagePerUnit = minimumVoltagePerUnit;
  mParametersSet = true;

  SPDLOG_LOGGER_INFO(mSLog,
                     "\nActive power={} [W]"
                     "\nReactive power={} [var]"
                     "\nNominal line-to-line voltage={} [V]"
                     "\nMinimum voltage={} [pu]",
                     **mActivePower, **mReactivePower, **mNomVoltage,
                     **mMinimumVoltagePerUnit);
}

SimPowerComp<Real>::Ptr EMT::Ph3::PQLoad::clone(String name) {
  auto copy = PQLoad::make(name, mLogLevel);
  if (mParametersSet) {
    copy->setParameters(**mActivePower, **mReactivePower, **mNomVoltage,
                        **mMinimumVoltagePerUnit);
  } else {
    **copy->mMinimumVoltagePerUnit = **mMinimumVoltagePerUnit;
  }
  copy->setMaxIterations(mMaxIter);
  copy->setTolerance(mTolerance);
  return copy;
}

Matrix EMT::Ph3::PQLoad::quadratureVoltage(const Matrix &voltage) {
  Matrix voltageQuadrature = Matrix::Zero(3, 1);
  voltageQuadrature << (voltage(1, 0) - voltage(2, 0)) / std::sqrt(3.0),
      (voltage(2, 0) - voltage(0, 0)) / std::sqrt(3.0),
      (voltage(0, 0) - voltage(1, 0)) / std::sqrt(3.0);
  return voltageQuadrature;
}

Matrix EMT::Ph3::PQLoad::calculateCurrent(const Matrix &voltage) const {
  Matrix voltageNoZeroSequence = voltage;
  voltageNoZeroSequence.array() -= voltageNoZeroSequence.mean();

  const Matrix voltageQuadrature = quadratureVoltage(voltageNoZeroSequence);
  const Real minimumVoltage = **mMinimumVoltagePerUnit * **mNomVoltage;
  const Real denominator = std::max(voltageNoZeroSequence.squaredNorm(),
                                    minimumVoltage * minimumVoltage);

  return (**mActivePower * voltageNoZeroSequence +
          **mReactivePower * voltageQuadrature) /
         denominator;
}

void EMT::Ph3::PQLoad::updateMeasuredPower() {
  Matrix voltageNoZeroSequence = **mIntfVoltage;
  voltageNoZeroSequence.array() -= voltageNoZeroSequence.mean();
  const Matrix voltageQuadrature = quadratureVoltage(voltageNoZeroSequence);

  **mMeasuredActivePower =
      voltageNoZeroSequence.cwiseProduct(**mIntfCurrent).sum();
  **mMeasuredReactivePower =
      voltageQuadrature.cwiseProduct(**mIntfCurrent).sum();
}

void EMT::Ph3::PQLoad::initializeFromNodesAndTerminals(Real frequency) {
  if (!mParametersSet) {
    **mActivePower = mTerminals[0]->singleActivePower();
    **mReactivePower = mTerminals[0]->singleReactivePower();
    **mNomVoltage = std::abs(mTerminals[0]->initialSingleVoltage());

    if (**mNomVoltage <= 0.0) {
      throw std::invalid_argument(
          "PQLoad requires a positive nominal voltage when initialized from "
          "terminal data.");
    }
  }

  MatrixComp initialVoltageABC = MatrixComp::Zero(3, 1);
  initialVoltageABC(0, 0) =
      RMS3PH_TO_PEAK1PH * mTerminals[0]->initialSingleVoltage();
  initialVoltageABC(1, 0) = initialVoltageABC(0, 0) * SHIFT_TO_PHASE_B;
  initialVoltageABC(2, 0) = initialVoltageABC(0, 0) * SHIFT_TO_PHASE_C;

  **mIntfVoltage = initialVoltageABC.real();
  **mIntfCurrent = calculateCurrent(**mIntfVoltage);
  updateMeasuredPower();

  SPDLOG_LOGGER_INFO(mSLog,
                     "\n--- Initialization ---"
                     "\nVoltage: {:s}"
                     "\nCurrent: {:s}"
                     "\nActive-power set point: {} [W]"
                     "\nReactive-power set point: {} [var]"
                     "\n--- Initialization finished ---",
                     Logger::matrixToString(**mIntfVoltage),
                     Logger::matrixToString(**mIntfCurrent), **mActivePower,
                     **mReactivePower);
}

void EMT::Ph3::PQLoad::mnaCompInitialize(Real omega, Real timeStep,
                                         Attribute<Matrix>::Ptr leftVector) {
  updateMatrixNodeIndices();
}

void EMT::Ph3::PQLoad::mnaCompApplySystemMatrixStamp(
    SparseMatrixRow &systemMatrix) {
  // The PQ load is represented by an ideal current sink and therefore has no
  // system-matrix stamp.
}

void EMT::Ph3::PQLoad::mnaCompApplyRightSideVectorStamp(Matrix &rightVector) {
  if (!terminalNotGrounded(0))
    return;

  for (UInt phase = 0; phase < 3; ++phase) {
    // Positive interface current is consumed by the load and therefore is a
    // negative injection into the network node.
    Math::setVectorElement(rightVector, matrixNodeIndex(0, phase),
                           -(**mIntfCurrent)(phase, 0));
  }
}

void EMT::Ph3::PQLoad::mnaCompUpdateVoltage(const Matrix &leftVector) {
  **mIntfVoltage = Matrix::Zero(3, 1);
  if (!terminalNotGrounded(0))
    return;

  for (UInt phase = 0; phase < 3; ++phase) {
    (**mIntfVoltage)(phase, 0) =
        Math::realFromVectorElement(leftVector, matrixNodeIndex(0, phase));
  }
}

void EMT::Ph3::PQLoad::mnaCompPreStep(Real time, Int timeStepCount) {
  **mNumIter = 0;
  **mIntfCurrent = calculateCurrent(**mIntfVoltage);
  (**mRightVector).setZero();
  mnaCompApplyRightSideVectorStamp(**mRightVector);
}

void EMT::Ph3::PQLoad::mnaCompPostStep(Real time, Int timeStepCount,
                                       Attribute<Matrix>::Ptr &leftVector) {
  mnaCompUpdateVoltage(**leftVector);
  updateMeasuredPower();
}

void EMT::Ph3::PQLoad::mnaCompAddPreStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes) {
  prevStepDependencies.push_back(mIntfVoltage);
  attributeDependencies.push_back(mActivePower);
  attributeDependencies.push_back(mReactivePower);
  attributeDependencies.push_back(mNomVoltage);
  attributeDependencies.push_back(mMinimumVoltagePerUnit);
  modifiedAttributes.push_back(mIntfCurrent);
  modifiedAttributes.push_back(mNumIter);
  modifiedAttributes.push_back(mRightVector);
}

void EMT::Ph3::PQLoad::mnaCompAddPostStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes,
    Attribute<Matrix>::Ptr &leftVector) {
  attributeDependencies.push_back(leftVector);
  attributeDependencies.push_back(mIntfCurrent);
  modifiedAttributes.push_back(mIntfVoltage);
  modifiedAttributes.push_back(mMeasuredActivePower);
  modifiedAttributes.push_back(mMeasuredReactivePower);
}

void EMT::Ph3::PQLoad::correctorStep() {
  **mNumIter = **mNumIter + 1;
  **mIntfCurrent = calculateCurrent(**mIntfVoltage);
  (**mRightVector).setZero();
  mnaCompApplyRightSideVectorStamp(**mRightVector);
}

void EMT::Ph3::PQLoad::updateVoltage(const Matrix &leftVector) {
  mnaCompUpdateVoltage(leftVector);
}

bool EMT::Ph3::PQLoad::requiresIteration() {
  const Matrix targetCurrent = calculateCurrent(**mIntfVoltage);
  const Real currentBase = std::hypot(**mActivePower, **mReactivePower) /
                           std::max(**mNomVoltage, 1.0);
  const Real currentScale =
      std::max({targetCurrent.norm(), currentBase, 1e-12});
  const Real relativeError =
      (targetCurrent - **mIntfCurrent).norm() / currentScale;

  if (relativeError <= mTolerance)
    return false;

  if (**mNumIter >= mMaxIter) {
    SPDLOG_LOGGER_WARN(mSLog,
                       "PQLoad corrector reached its iteration limit ({}): "
                       "relative current residual={} (tolerance={}).",
                       mMaxIter, relativeError, mTolerance);
    return false;
  }

  return true;
}
