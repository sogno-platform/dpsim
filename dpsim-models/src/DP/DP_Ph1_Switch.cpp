/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <dpsim-models/DP/DP_Ph1_Switch.h>

using namespace CPS;

DP::Ph1::Switch::Switch(String uid, String name, Logger::Level logLevel)
    : MNASimPowerComp<Complex>(uid, name, false, true, logLevel),
      Base::Ph1::Switch(mAttributes),
      mOpeningRequested(mAttributes->create<Bool>("opening_requested")),
      mPoleClosed(mAttributes->create<Bool>("pole_closed")),
      mInstantCurrent(mAttributes->create<Real>("i_instantaneous")),
      mZeroCrossingTime(mAttributes->create<Real>("zero_crossing_time")),
      mExponentialTransitionActive(
          mAttributes->create<Bool>("exponential_transition_active")),
      mExponentialProgress(mAttributes->create<Real>("exponential_progress")),
      mExponentialTransitionStartTime(
          mAttributes->create<Real>("exponential_transition_start_time")),
      mExponentialTransitionEndTime(
          mAttributes->create<Real>("exponential_transition_end_time")),
      mEffectiveResistance(mAttributes->create<Real>("effective_resistance")) {
  setTerminalNumber(2);
  **mIntfVoltage = MatrixComp::Zero(1, 1);
  **mIntfCurrent = MatrixComp::Zero(1, 1);

  **mOpeningRequested = false;
  **mPoleClosed = false;

  **mInstantCurrent = 0.0;

  resetZeroCrossingTime();

  **mExponentialTransitionActive = false;
  **mExponentialProgress = 0.0;
  **mExponentialTransitionStartTime = -1.0;
  **mExponentialTransitionEndTime = -1.0;
  **mEffectiveResistance = 0.0;
}

SimPowerComp<Complex>::Ptr DP::Ph1::Switch::clone(String name) {
  auto copy = Switch::make(name, mLogLevel);
  copy->setParameters(**mOpenResistance, **mClosedResistance, **mIsClosed);
  copy->setSwitchingMode(mSwitchingMode);
  copy->setZeroCrossingTolerance(mZeroCrossingTolerance);
  copy->setExponentialSwitchingTime(mExponentialSwitchingTime);
  return copy;
}

void DP::Ph1::Switch::setSwitchingMode(SwitchingMode mode) {
  mSwitchingMode = mode;

  **mPoleClosed = **mIsClosed;
  **mOpeningRequested = false;
  resetZeroCrossingTime();

  mPreviousCurrentValid = false;
  mResetZeroCrossingHistory = false;

  resetExponentialTransition();
  synchronizeEffectiveResistance(**mIsClosed);
}

void DP::Ph1::Switch::setZeroCrossingTolerance(Real tolerance) {
  if (tolerance < 0.0) {
    throw std::invalid_argument(
        "DP::Ph1::Switch zero-crossing tolerance must be non-negative.");
  }

  mZeroCrossingTolerance = tolerance;
}

void DP::Ph1::Switch::setExponentialSwitchingTime(Real switchingTime) {
  if (switchingTime <= 0.0) {
    throw std::invalid_argument(
        "DP::Ph1::Switch exponential switching time must be positive.");
  }

  mExponentialSwitchingTime = switchingTime;
}

void DP::Ph1::Switch::openSwitch() {
  Base::Ph1::Switch::openSwitch();

  if (mSwitchingMode == SwitchingMode::Ideal) {
    **mOpeningRequested = false;
    **mPoleClosed = false;
    resetZeroCrossingTime();
    resetExponentialTransition();
    synchronizeEffectiveResistance(false);
    return;
  }

  if (!**mPoleClosed) {
    **mOpeningRequested = false;
    return;
  }

  if (mSwitchingMode == SwitchingMode::CurrentZero) {
    **mOpeningRequested = true;
    resetZeroCrossingTime();
    resetExponentialTransition();
    mResetZeroCrossingHistory = true;

    SPDLOG_LOGGER_INFO(mSLog,
                       "DP current-zero opening command received. "
                       "Waiting for the reconstructed physical current zero.");
    return;
  }

  validateExponentialResistanceParameters();
  **mOpeningRequested = true;
  resetZeroCrossingTime();
  resetExponentialTransition();
  **mExponentialTransitionActive = true;
  synchronizeEffectiveResistance(true);
  **mPoleClosed = true;

  SPDLOG_LOGGER_INFO(mSLog,
                     "DP exponential ZCS-emulation opening command received. "
                     "Switching duration={:.6e}s.",
                     mExponentialSwitchingTime);
}

void DP::Ph1::Switch::closeSwitch() {
  Base::Ph1::Switch::closeSwitch();

  **mOpeningRequested = false;
  **mPoleClosed = true;
  resetZeroCrossingTime();
  mResetZeroCrossingHistory = false;

  resetExponentialTransition();
  synchronizeEffectiveResistance(true);

  SPDLOG_LOGGER_INFO(mSLog, "DP switch closing command: pole closed.");
}

void DP::Ph1::Switch::initializeFromNodesAndTerminals(Real frequency) {

  Real impedance = (**mIsClosed) ? **mClosedResistance : **mOpenResistance;
  (**mIntfVoltage)(0, 0) = initialSingleVoltage(1) - initialSingleVoltage(0);
  (**mIntfCurrent)(0, 0) = (**mIntfVoltage)(0, 0) / impedance;

  // Power-flow initialization defines the initial physical breaker state.
  **mPoleClosed = **mIsClosed;
  **mOpeningRequested = false;
  resetZeroCrossingTime();
  resetExponentialTransition();
  synchronizeEffectiveResistance(**mIsClosed);

  SPDLOG_LOGGER_INFO(mSLog,
                     "\n--- Initialization from powerflow ---"
                     "\nVoltage across: {:s}"
                     "\nCurrent: {:s}"
                     "\nTerminal 0 voltage: {:s}"
                     "\nTerminal 1 voltage: {:s}"
                     "\nSwitching mode: {:s}"
                     "\nInitial breaker state: {:s}"
                     "\n--- Initialization from powerflow finished ---",
                     Logger::phasorToString((**mIntfVoltage)(0, 0)),
                     Logger::phasorToString((**mIntfCurrent)(0, 0)),
                     Logger::phasorToString(initialSingleVoltage(0)),
                     Logger::phasorToString(initialSingleVoltage(1)),
                     mSwitchingMode == SwitchingMode::Ideal
                         ? "Ideal"
                         : (mSwitchingMode == SwitchingMode::CurrentZero
                                ? "CurrentZero"
                                : "ExponentialZCSEmulation"),
                     **mIsClosed ? "closed" : "open");
}

void DP::Ph1::Switch::mnaCompInitialize(Real omega, Real timeStep,
                                        Attribute<Matrix>::Ptr leftVector) {
  updateMatrixNodeIndices();
  mTimeStep = timeStep;
  **mRightVector = Matrix::Zero(0, 0);

  // omega is the DP reference-frame angular frequency supplied by the solver.
  mShiftOmega = omega;

  // Final safeguard in case setSwitchingMode() was called before setParameters().
  **mPoleClosed = **mIsClosed;
  synchronizeEffectiveResistance(**mIsClosed);
  resetExponentialTransition();

  mPoleClosedPrev = **mPoleClosed;
  mIsClosedPrev = **mIsClosed;

  **mInstantCurrent = reconstructInstantaneousCurrent(0.0);

  mPreviousInstantaneousCurrent = **mInstantCurrent;
  mEffectiveResistancePrev = **mEffectiveResistance;

  mPreviousCurrentTime = 0.0;
  mPreviousCurrentValid = true;
  mResetZeroCrossingHistory = false;
}

void DP::Ph1::Switch::mnaCompApplySystemMatrixStamp(
    SparseMatrixRow &systemMatrix) {
  MNAStampUtils::stampAdmittance(currentConductance(), systemMatrix,
                                 matrixNodeIndex(0), matrixNodeIndex(1),
                                 terminalNotGrounded(0), terminalNotGrounded(1),
                                 mSLog);
}

void DP::Ph1::Switch::mnaCompApplySwitchSystemMatrixStamp(
    Bool closed, SparseMatrixRow &systemMatrix, Int freqIdx) {
  // Conventional precomputed two-state path. The CurrentZero and exponential
  // modes bypass this by returning supportsPrecomputedSystemMatrices() == false.
  Complex conductance = (closed) ? Complex(1. / **mClosedResistance, 0)
                                 : Complex(1. / **mOpenResistance, 0);

  MNAStampUtils::stampAdmittance(conductance, systemMatrix, matrixNodeIndex(0),
                                 matrixNodeIndex(1), terminalNotGrounded(0),
                                 terminalNotGrounded(1), mSLog);
}

void DP::Ph1::Switch::mnaCompApplyRightSideVectorStamp(Matrix &rightVector) {}

Bool DP::Ph1::Switch::mnaIsClosed() {
  if (mSwitchingMode == SwitchingMode::Ideal)
    return **mIsClosed;

  return **mPoleClosed;
}

Bool DP::Ph1::Switch::supportsPrecomputedSystemMatrices() const {
  return mSwitchingMode == SwitchingMode::Ideal;
}

void DP::Ph1::Switch::mnaCompUpdateVoltage(const Matrix &leftVector) {
  // Voltage across component is defined as V1 - V0
  (**mIntfVoltage)(0, 0) = 0;
  if (terminalNotGrounded(1))
    (**mIntfVoltage)(0, 0) =
        Math::complexFromVectorElement(leftVector, matrixNodeIndex(1, 0));
  if (terminalNotGrounded(0))
    (**mIntfVoltage)(0, 0) =
        (**mIntfVoltage)(0, 0) -
        Math::complexFromVectorElement(leftVector, matrixNodeIndex(0));
}

void DP::Ph1::Switch::mnaCompUpdateCurrent(const Matrix &leftVector) {
  (**mIntfCurrent)(0, 0) = currentConductance() * (**mIntfVoltage)(0, 0);
}

void DP::Ph1::Switch::mnaCompAddPostStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes,
    Attribute<Matrix>::Ptr &leftVector) {

  attributeDependencies.push_back(leftVector);
  modifiedAttributes.push_back(mIntfVoltage);
  modifiedAttributes.push_back(mIntfCurrent);

  modifiedAttributes.push_back(mOpeningRequested);
  modifiedAttributes.push_back(mPoleClosed);

  modifiedAttributes.push_back(mInstantCurrent);
  modifiedAttributes.push_back(mZeroCrossingTime);

  modifiedAttributes.push_back(mExponentialTransitionActive);
  modifiedAttributes.push_back(mExponentialProgress);
  modifiedAttributes.push_back(mExponentialTransitionStartTime);
  modifiedAttributes.push_back(mExponentialTransitionEndTime);
  modifiedAttributes.push_back(mEffectiveResistance);
}

void DP::Ph1::Switch::mnaCompPostStep(Real time, Int timeStepCount,
                                      Attribute<Matrix>::Ptr &leftVector) {
  mnaCompUpdateVoltage(**leftVector);
  mnaCompUpdateCurrent(**leftVector);

  **mInstantCurrent = reconstructInstantaneousCurrent(time);

  if (mSwitchingMode == SwitchingMode::CurrentZero) {
    updateZeroCrossingState(time);
  } else if (mSwitchingMode == SwitchingMode::ExponentialZCSEmulation) {
    updateExponentialTransition(time);
  }
}

Bool DP::Ph1::Switch::hasParameterChanged() {
  if (mSwitchingMode == SwitchingMode::Ideal) {
    // Check if state of switch changed
    if (mIsClosedPrev != **mIsClosed) {
      mIsClosedPrev = **mIsClosed;
      return true; //recompute system matrix
    }

    return false; // do not recompute system matrix
  }

  if (mSwitchingMode == SwitchingMode::ExponentialZCSEmulation) {
    if (**mEffectiveResistance != mEffectiveResistancePrev) {
      mEffectiveResistancePrev = **mEffectiveResistance;
      return true;
    }

    return false;
  }

  if (**mPoleClosed != mPoleClosedPrev) {
    mPoleClosedPrev = **mPoleClosed;
    return true;
  }

  return false;
}

void DP::Ph1::Switch::setPoleClosed(Bool closed) {
  **mPoleClosed = closed;

  // Keep the resistance diagnostics synchronized with the discrete
  // CurrentZero pole state. Exponential mode manages R(t) independently.
  if (mSwitchingMode == SwitchingMode::CurrentZero)
    synchronizeEffectiveResistance(closed);
}

void DP::Ph1::Switch::resetZeroCrossingTime() { **mZeroCrossingTime = -1.0; }

void DP::Ph1::Switch::synchronizeEffectiveResistance(Bool closed) {
  **mEffectiveResistance = closed ? **mClosedResistance : **mOpenResistance;
}

void DP::Ph1::Switch::resetExponentialTransition() {
  **mExponentialTransitionActive = false;
  **mExponentialProgress = 0.0;
  **mExponentialTransitionStartTime = -1.0;
  **mExponentialTransitionEndTime = -1.0;
  mExponentialTransitionStarted = false;
}

void DP::Ph1::Switch::validateExponentialResistanceParameters() const {
  if (mExponentialSwitchingTime <= 0.0) {
    throw std::invalid_argument(
        "DP::Ph1::Switch exponential switching time must be positive.");
  }

  if (**mClosedResistance <= 0.0 || **mOpenResistance <= 0.0) {
    throw std::invalid_argument("DP::Ph1::Switch exponential mode requires "
                                "positive open and closed resistances.");
  }
}

Real DP::Ph1::Switch::exponentialResistance(Real alpha) const {
  const Real rClosed = **mClosedResistance;
  const Real rOpen = **mOpenResistance;
  const Real boundedAlpha = std::max(0.0, std::min(1.0, alpha));

  return std::exp(std::log(rClosed) +
                  boundedAlpha * (std::log(rOpen) - std::log(rClosed)));
}

void DP::Ph1::Switch::updateExponentialTransition(Real time) {
  if (!**mExponentialTransitionActive)
    return;

  if (!mExponentialTransitionStarted) {
    mExponentialTransitionStarted = true;
    **mExponentialTransitionStartTime = time;
    **mExponentialTransitionEndTime = time + mExponentialSwitchingTime;
  }

  const Real targetTime = time + mTimeStep;
  const Real alpha = (targetTime - **mExponentialTransitionStartTime) /
                     mExponentialSwitchingTime;
  const Real boundedAlpha = std::max(0.0, std::min(1.0, alpha));

  **mExponentialProgress = boundedAlpha;
  **mEffectiveResistance = exponentialResistance(boundedAlpha);

  if (boundedAlpha >= 1.0) {
    synchronizeEffectiveResistance(false);
    **mPoleClosed = false;
    **mOpeningRequested = false;
    **mExponentialTransitionActive = false;

    SPDLOG_LOGGER_INFO(
        mSLog,
        "DP exponential ZCS-emulation opening completed: start={:.9f}s, "
        "end={:.9f}s, duration={:.6e}s.",
        **mExponentialTransitionStartTime, **mExponentialTransitionEndTime,
        mExponentialSwitchingTime);
  }
}

Complex DP::Ph1::Switch::currentConductance() const {
  if (mSwitchingMode == SwitchingMode::Ideal) {
    return (**mIsClosed) ? Complex(1. / **mClosedResistance, 0)
                         : Complex(1. / **mOpenResistance, 0);
  }

  if (mSwitchingMode == SwitchingMode::CurrentZero) {
    return (**mPoleClosed) ? Complex(1. / **mClosedResistance, 0)
                           : Complex(1. / **mOpenResistance, 0);
  }

  return Complex(1. / **mEffectiveResistance, 0);
}

Real DP::Ph1::Switch::reconstructInstantaneousCurrent(Real time) const {
  // Restore the reference-frequency carrier removed by the DP formulation.
  // DP::Ph1 envelopes keep the power-flow amplitude scaling, so no RMS-to-peak
  // conversion is applied here; the zero crossing instant does not depend on it.
  const Complex carrier = std::polar<Real>(1.0, mShiftOmega * time);

  return std::real((**mIntfCurrent)(0, 0) * carrier);
}

Bool DP::Ph1::Switch::currentCrossedZero(Real previousCurrent,
                                         Real current) const {
  if (std::abs(current) <= mZeroCrossingTolerance)
    return true;

  return (previousCurrent > mZeroCrossingTolerance &&
          current < -mZeroCrossingTolerance) ||
         (previousCurrent < -mZeroCrossingTolerance &&
          current > mZeroCrossingTolerance);
}

Real DP::Ph1::Switch::interpolateZeroCrossingTime(Real previousCurrent,
                                                  Real current,
                                                  Real previousTime,
                                                  Real currentTime) const {
  const Real denominator = std::abs(previousCurrent) + std::abs(current);

  if (denominator <= mZeroCrossingTolerance)
    return currentTime;

  const Real fraction = std::abs(previousCurrent) / denominator;

  return previousTime + fraction * (currentTime - previousTime);
}

void DP::Ph1::Switch::updateZeroCrossingState(Real time) {
  const Real current = **mInstantCurrent;

  // Keep a current history while no opening command is active.
  if (!**mOpeningRequested) {
    mPreviousInstantaneousCurrent = current;
    mPreviousCurrentTime = time;
    mPreviousCurrentValid = true;
    return;
  }

  // First post-command sample: do not compare against the pre-command sample.
  if (mResetZeroCrossingHistory || !mPreviousCurrentValid) {
    if (**mPoleClosed && std::abs(current) <= mZeroCrossingTolerance) {
      setPoleClosed(false);
      **mZeroCrossingTime = time;

      SPDLOG_LOGGER_INFO(mSLog,
                         "DP pole opened at sampled physical current zero: "
                         "t={:.9f}s, i={:.6e}A",
                         time, current);
    }

    mPreviousInstantaneousCurrent = current;
    mPreviousCurrentTime = time;
    mPreviousCurrentValid = true;
    mResetZeroCrossingHistory = false;

    if (!**mPoleClosed)
      **mOpeningRequested = false;

    return;
  }

  if (**mPoleClosed &&
      currentCrossedZero(mPreviousInstantaneousCurrent, current)) {
    const Real zeroTime = interpolateZeroCrossingTime(
        mPreviousInstantaneousCurrent, current, mPreviousCurrentTime, time);

    setPoleClosed(false);
    **mZeroCrossingTime = zeroTime;

    SPDLOG_LOGGER_INFO(mSLog,
                       "DP physical current zero detected: "
                       "t_z={:.9f}s, i_prev={:.6e}A, i={:.6e}A. Pole opened.",
                       zeroTime, mPreviousInstantaneousCurrent, current);
  }

  mPreviousInstantaneousCurrent = current;
  mPreviousCurrentTime = time;
  mPreviousCurrentValid = true;

  if (!**mPoleClosed) {
    **mOpeningRequested = false;

    SPDLOG_LOGGER_INFO(mSLog, "DP current-zero interruption completed: "
                              "the breaker pole is open.");
  }
}
