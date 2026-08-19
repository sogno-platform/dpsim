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

#include <dpsim-models/EMT/EMT_Ph3_Switch.h>

using namespace CPS;

EMT::Ph3::Switch::Switch(String uid, String name, Logger::Level logLevel)
    : MNASimPowerComp<Real>(uid, name, false, true, logLevel),
      Base::Ph3::Switch(mAttributes),
      mOpeningRequested(mAttributes->create<Bool>("opening_requested")),
      mPoleClosedA(mAttributes->create<Bool>("pole_closed_a")),
      mPoleClosedB(mAttributes->create<Bool>("pole_closed_b")),
      mPoleClosedC(mAttributes->create<Bool>("pole_closed_c")),
      mZeroCrossingTimeA(mAttributes->create<Real>("zero_crossing_time_a")),
      mZeroCrossingTimeB(mAttributes->create<Real>("zero_crossing_time_b")),
      mZeroCrossingTimeC(mAttributes->create<Real>("zero_crossing_time_c")),
      mExponentialTransitionActive(
          mAttributes->create<Bool>("exponential_transition_active")),
      mExponentialTransitionClosing(
          mAttributes->create<Bool>("exponential_transition_closing")),
      mExponentialProgress(mAttributes->create<Real>("exponential_progress")),
      mExponentialTransitionStartTime(
          mAttributes->create<Real>("exponential_transition_start_time")),
      mExponentialTransitionEndTime(
          mAttributes->create<Real>("exponential_transition_end_time")),
      mEffectiveResistanceA(
          mAttributes->create<Real>("effective_resistance_a")),
      mEffectiveResistanceB(
          mAttributes->create<Real>("effective_resistance_b")),
      mEffectiveResistanceC(
          mAttributes->create<Real>("effective_resistance_c")) {
  setTerminalNumber(2);

  // IMPORTANT:
  // Keep the three-phase interface dimensions fixed from construction.
  // DataLogger inspects matrix dimensions when attributes are registered,
  // before the simulation starts.
  **mIntfVoltage = Matrix::Zero(3, 1);
  **mIntfCurrent = Matrix::Zero(3, 1);

  **mOpeningRequested = false;
  setAllPoles(false);
  resetZeroCrossingTimes();

  **mExponentialTransitionActive = false;
  **mExponentialTransitionClosing = false;
  **mExponentialProgress = 0.0;
  **mExponentialTransitionStartTime = -1.0;
  **mExponentialTransitionEndTime = -1.0;
  **mEffectiveResistanceA = 0.0;
  **mEffectiveResistanceB = 0.0;
  **mEffectiveResistanceC = 0.0;
}

SimPowerComp<Real>::Ptr EMT::Ph3::Switch::clone(String name) {
  auto copy = Switch::make(name, mLogLevel);
  copy->setParameters(**mOpenResistance, **mClosedResistance, **mIsClosed);
  copy->setSwitchingMode(mSwitchingMode);
  copy->setZeroCrossingTolerance(mZeroCrossingTolerance);
  copy->setExponentialSwitchingTime(mExponentialSwitchingTime);
  return copy;
}

void EMT::Ph3::Switch::setSwitchingMode(SwitchingMode mode) {
  mSwitchingMode = mode;

  // Synchronize physical poles with the configured initial command state.
  setAllPoles(**mIsClosed);
  **mOpeningRequested = false;
  resetZeroCrossingTimes();

  mPreviousCurrentValid = false;
  mResetZeroCrossingHistory = false;

  resetExponentialTransition();
  synchronizeEffectiveResistance(**mIsClosed);
}

void EMT::Ph3::Switch::setZeroCrossingTolerance(Real tolerance) {
  if (tolerance < 0.0) {
    throw std::invalid_argument(
        "EMT::Ph3::Switch zero-crossing tolerance must be non-negative.");
  }

  mZeroCrossingTolerance = tolerance;
}

void EMT::Ph3::Switch::setExponentialSwitchingTime(Real switchingTime) {
  if (switchingTime <= 0.0) {
    throw std::invalid_argument(
        "EMT::Ph3::Switch exponential switching time must be positive.");
  }

  mExponentialSwitchingTime = switchingTime;
}

void EMT::Ph3::Switch::openSwitch() {
  Base::Ph3::Switch::openSwitch();

  // An opening command during a closing ramp abandons that ramp. The poles are
  // still open at that point, so the switch simply stays open.
  if (**mExponentialTransitionActive && **mExponentialTransitionClosing) {
    resetExponentialTransition();
    setAllPoles(false);
    synchronizeEffectiveResistance(false);
    **mOpeningRequested = false;

    SPDLOG_LOGGER_INFO(mSLog, "Opening command received during an exponential "
                              "closing transition. Transition abandoned.");
    return;
  }

  if (mSwitchingMode == SwitchingMode::Ideal) {
    **mOpeningRequested = false;
    setAllPoles(false);
    resetZeroCrossingTimes();
    resetExponentialTransition();
    synchronizeEffectiveResistance(false);
    return;
  }

  if (allPolesOpen()) {
    **mOpeningRequested = false;
    return;
  }

  if (mSwitchingMode == SwitchingMode::CurrentZero) {
    **mOpeningRequested = true;
    resetZeroCrossingTimes();
    resetExponentialTransition();
    mResetZeroCrossingHistory = true;

    SPDLOG_LOGGER_INFO(mSLog, "Current-zero opening command received. Waiting "
                              "for phase-current zeros.");
    return;
  }

  validateExponentialResistanceParameters();
  **mOpeningRequested = true;
  resetZeroCrossingTimes();
  resetExponentialTransition();
  **mExponentialTransitionActive = true;
  synchronizeEffectiveResistance(true);
  setAllPoles(true);

  SPDLOG_LOGGER_INFO(mSLog,
                     "Exponential ZCS-emulation opening command received. "
                     "Switching duration={:.6e}s.",
                     mExponentialSwitchingTime);
}

void EMT::Ph3::Switch::closeSwitch() {
  Base::Ph3::Switch::closeSwitch();

  **mOpeningRequested = false;
  resetZeroCrossingTimes();
  mResetZeroCrossingHistory = false;

  if (mSwitchingMode != SwitchingMode::ExponentialZCSEmulation) {
    setAllPoles(true);
    resetExponentialTransition();
    synchronizeEffectiveResistance(true);

    SPDLOG_LOGGER_INFO(mSLog,
                       "Switch closing command: all three poles closed.");
    return;
  }

  if (allPolesClosed() && !**mExponentialTransitionActive) {
    resetExponentialTransition();
    synchronizeEffectiveResistance(true);
    return;
  }

  // Energising a branch is the mirror image of interrupting it: the same
  // resistance path is traversed from R_open towards R_closed. The poles stay
  // open until the ramp completes.
  validateExponentialResistanceParameters();
  resetExponentialTransition();
  **mExponentialTransitionActive = true;
  **mExponentialTransitionClosing = true;
  **mExponentialProgress = 1.0;
  synchronizeEffectiveResistance(false);
  setAllPoles(false);

  SPDLOG_LOGGER_INFO(mSLog,
                     "Exponential ZCS-emulation closing command received. "
                     "Switching duration={:.6e}s.",
                     mExponentialSwitchingTime);
}

void EMT::Ph3::Switch::initializeFromNodesAndTerminals(Real frequency) {
  Matrix impedance = (**mIsClosed) ? **mClosedResistance : **mOpenResistance;

  MatrixComp vInitABC = MatrixComp::Zero(3, 1);
  vInitABC(0, 0) = initialSingleVoltage(1) - initialSingleVoltage(0);
  vInitABC(1, 0) = vInitABC(0, 0) * SHIFT_TO_PHASE_B;
  vInitABC(2, 0) = vInitABC(0, 0) * SHIFT_TO_PHASE_C;

  **mIntfVoltage = vInitABC.real();
  **mIntfCurrent = (impedance.inverse() * vInitABC).real();

  // Power-flow initialization defines the initial physical breaker state.
  setAllPoles(**mIsClosed);
  **mOpeningRequested = false;
  resetZeroCrossingTimes();
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
                     Logger::matrixToString(**mIntfVoltage),
                     Logger::matrixToString(**mIntfCurrent),
                     Logger::phasorToString(initialSingleVoltage(0)),
                     Logger::phasorToString(initialSingleVoltage(1)),
                     mSwitchingMode == SwitchingMode::Ideal
                         ? "Ideal"
                         : (mSwitchingMode == SwitchingMode::CurrentZero
                                ? "CurrentZero"
                                : "ExponentialZCSEmulation"),
                     **mIsClosed ? "closed" : "open");
}

void EMT::Ph3::Switch::mnaCompInitialize(Real omega, Real timeStep,
                                         Attribute<Matrix>::Ptr leftVector) {
  updateMatrixNodeIndices();
  mTimeStep = timeStep;
  **mRightVector = Matrix::Zero(0, 0);

  // setParameters() belongs to the shared base class. Synchronize here as a
  // final safeguard in case setSwitchingMode() was called before setParameters().
  setAllPoles(**mIsClosed);
  synchronizeEffectiveResistance(**mIsClosed);
  resetExponentialTransition();

  mPoleClosedPrev = {
      **mPoleClosedA,
      **mPoleClosedB,
      **mPoleClosedC,
  };

  if ((**mIntfCurrent).rows() == 3) {
    mPreviousCurrent = **mIntfCurrent;
    mPreviousCurrentValid = true;
  } else {
    mPreviousCurrent = Matrix::Zero(3, 1);
    mPreviousCurrentValid = false;
  }

  mEffectiveResistancePrev = {{
      **mEffectiveResistanceA,
      **mEffectiveResistanceB,
      **mEffectiveResistanceC,
  }};

  mPreviousCurrentTime = 0.0;
  mResetZeroCrossingHistory = false;
}

Bool EMT::Ph3::Switch::mnaIsClosed() {
  if (mSwitchingMode == SwitchingMode::Ideal)
    return **mIsClosed;

  return allPolesClosed();
}

Bool EMT::Ph3::Switch::supportsPrecomputedSystemMatrices() const {
  return mSwitchingMode == SwitchingMode::Ideal;
}

void EMT::Ph3::Switch::mnaCompApplySystemMatrixStamp(
    SparseMatrixRow &systemMatrix) {
  const MatrixFixedSize<3, 3> conductance = currentConductanceMatrix();

  MNAStampUtils::stampConductanceMatrix(
      conductance, systemMatrix, matrixNodeIndex(0), matrixNodeIndex(1),
      terminalNotGrounded(0), terminalNotGrounded(1), mSLog);

  SPDLOG_LOGGER_TRACE(mSLog, "\nConductance matrix: {:s}",
                      Logger::matrixToString(conductance));
}

void EMT::Ph3::Switch::mnaCompApplySwitchSystemMatrixStamp(
    Bool closed, SparseMatrixRow &systemMatrix, Int freqIdx) {
  // This method is used by the conventional precomputed two-state switch path.
  // CurrentZero mode advertises supportsPrecomputedSystemMatrices() == false
  // and therefore uses mnaCompApplySystemMatrixStamp() through the variable
  // matrix recomputation path instead.
  MatrixFixedSize<3, 3> conductance =
      closed ? (**mClosedResistance).inverse() : (**mOpenResistance).inverse();

  MNAStampUtils::stampConductanceMatrix(
      conductance, systemMatrix, matrixNodeIndex(0), matrixNodeIndex(1),
      terminalNotGrounded(0), terminalNotGrounded(1), mSLog);

  SPDLOG_LOGGER_TRACE(mSLog, "\nConductance matrix: {:s}",
                      Logger::matrixToString(conductance));
}

void EMT::Ph3::Switch::mnaCompApplyRightSideVectorStamp(Matrix &rightVector) {}

void EMT::Ph3::Switch::mnaCompAddPostStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes,
    Attribute<Matrix>::Ptr &leftVector) {
  attributeDependencies.push_back(leftVector);

  modifiedAttributes.push_back(mIntfVoltage);
  modifiedAttributes.push_back(mIntfCurrent);

  modifiedAttributes.push_back(mOpeningRequested);
  modifiedAttributes.push_back(mPoleClosedA);
  modifiedAttributes.push_back(mPoleClosedB);
  modifiedAttributes.push_back(mPoleClosedC);
  modifiedAttributes.push_back(mZeroCrossingTimeA);
  modifiedAttributes.push_back(mZeroCrossingTimeB);
  modifiedAttributes.push_back(mZeroCrossingTimeC);

  modifiedAttributes.push_back(mExponentialTransitionActive);
  modifiedAttributes.push_back(mExponentialTransitionClosing);
  modifiedAttributes.push_back(mExponentialProgress);
  modifiedAttributes.push_back(mExponentialTransitionStartTime);
  modifiedAttributes.push_back(mExponentialTransitionEndTime);
  modifiedAttributes.push_back(mEffectiveResistanceA);
  modifiedAttributes.push_back(mEffectiveResistanceB);
  modifiedAttributes.push_back(mEffectiveResistanceC);
}

void EMT::Ph3::Switch::mnaCompPostStep(Real time, Int timeStepCount,
                                       Attribute<Matrix>::Ptr &leftVector) {
  mnaCompUpdateVoltage(**leftVector);
  mnaCompUpdateCurrent(**leftVector);

  if (mSwitchingMode == SwitchingMode::CurrentZero) {
    updateZeroCrossingState(time);
  } else if (mSwitchingMode == SwitchingMode::ExponentialZCSEmulation) {
    updateExponentialTransition(time);
  }
}

void EMT::Ph3::Switch::mnaCompUpdateVoltage(const Matrix &leftVector) {
  // Voltage across component is defined as V1 - V0.
  **mIntfVoltage = Matrix::Zero(3, 1);

  if (terminalNotGrounded(1)) {
    (**mIntfVoltage)(0, 0) =
        Math::realFromVectorElement(leftVector, matrixNodeIndex(1, 0));
    (**mIntfVoltage)(1, 0) =
        Math::realFromVectorElement(leftVector, matrixNodeIndex(1, 1));
    (**mIntfVoltage)(2, 0) =
        Math::realFromVectorElement(leftVector, matrixNodeIndex(1, 2));
  }

  if (terminalNotGrounded(0)) {
    (**mIntfVoltage)(0, 0) -=
        Math::realFromVectorElement(leftVector, matrixNodeIndex(0, 0));
    (**mIntfVoltage)(1, 0) -=
        Math::realFromVectorElement(leftVector, matrixNodeIndex(0, 1));
    (**mIntfVoltage)(2, 0) -=
        Math::realFromVectorElement(leftVector, matrixNodeIndex(0, 2));
  }
}

void EMT::Ph3::Switch::mnaCompUpdateCurrent(const Matrix &leftVector) {
  **mIntfCurrent = currentConductanceMatrix() * **mIntfVoltage;
}

Bool EMT::Ph3::Switch::hasParameterChanged() {
  if (mSwitchingMode == SwitchingMode::Ideal) {
    // Preserve the previous DPsim switch behavior.
    if (mIsClosedPrev != mnaIsClosed()) {
      mIsClosedPrev = mnaIsClosed();
      return true;
    }

    return false;
  }

  if (mSwitchingMode == SwitchingMode::ExponentialZCSEmulation) {
    const std::array<Real, 3> resistanceNow{{
        **mEffectiveResistanceA,
        **mEffectiveResistanceB,
        **mEffectiveResistanceC,
    }};

    Bool changed = false;
    for (UInt phase = 0; phase < 3; ++phase) {
      if (resistanceNow[phase] != mEffectiveResistancePrev[phase]) {
        mEffectiveResistancePrev[phase] = resistanceNow[phase];
        changed = true;
      }
    }
    return changed;
  }

  const std::array<Bool, 3> poleClosedNow{{
      **mPoleClosedA,
      **mPoleClosedB,
      **mPoleClosedC,
  }};

  Bool changed = false;

  for (UInt phase = 0; phase < 3; ++phase) {
    if (poleClosedNow[phase] != mPoleClosedPrev[phase]) {
      changed = true;
      mPoleClosedPrev[phase] = poleClosedNow[phase];
    }
  }

  return changed;
}

Bool EMT::Ph3::Switch::poleClosed(UInt phase) const {
  switch (phase) {
  case 0:
    return **mPoleClosedA;
  case 1:
    return **mPoleClosedB;
  case 2:
    return **mPoleClosedC;
  default:
    throw std::out_of_range("EMT::Ph3::Switch phase index must be 0, 1, or 2.");
  }
}

void EMT::Ph3::Switch::setPoleClosed(UInt phase, Bool closed) {
  switch (phase) {
  case 0:
    **mPoleClosedA = closed;
    break;
  case 1:
    **mPoleClosedB = closed;
    break;
  case 2:
    **mPoleClosedC = closed;
    break;
  default:
    throw std::out_of_range("EMT::Ph3::Switch phase index must be 0, 1, or 2.");
  }

  // Keep the resistance diagnostics synchronized with the discrete
  // CurrentZero pole state. Exponential mode manages R(t) independently.
  if (mSwitchingMode == SwitchingMode::CurrentZero &&
      (**mClosedResistance).rows() >= 3 && (**mOpenResistance).rows() >= 3) {
    setEffectiveResistance(phase, closed ? (**mClosedResistance)(phase, phase)
                                         : (**mOpenResistance)(phase, phase));
  }
}

void EMT::Ph3::Switch::setAllPoles(Bool closed) {
  **mPoleClosedA = closed;
  **mPoleClosedB = closed;
  **mPoleClosedC = closed;
}

Bool EMT::Ph3::Switch::allPolesClosed() const {
  return **mPoleClosedA && **mPoleClosedB && **mPoleClosedC;
}

Bool EMT::Ph3::Switch::allPolesOpen() const {
  return !**mPoleClosedA && !**mPoleClosedB && !**mPoleClosedC;
}

void EMT::Ph3::Switch::resetZeroCrossingTimes() {
  **mZeroCrossingTimeA = -1.0;
  **mZeroCrossingTimeB = -1.0;
  **mZeroCrossingTimeC = -1.0;
}

Real EMT::Ph3::Switch::effectiveResistance(UInt phase) const {
  switch (phase) {
  case 0:
    return **mEffectiveResistanceA;
  case 1:
    return **mEffectiveResistanceB;
  case 2:
    return **mEffectiveResistanceC;
  default:
    throw std::out_of_range("EMT::Ph3::Switch phase index must be 0, 1, or 2.");
  }
}

void EMT::Ph3::Switch::setEffectiveResistance(UInt phase, Real resistance) {
  switch (phase) {
  case 0:
    **mEffectiveResistanceA = resistance;
    return;
  case 1:
    **mEffectiveResistanceB = resistance;
    return;
  case 2:
    **mEffectiveResistanceC = resistance;
    return;
  default:
    throw std::out_of_range("EMT::Ph3::Switch phase index must be 0, 1, or 2.");
  }
}

void EMT::Ph3::Switch::synchronizeEffectiveResistance(Bool closed) {
  if ((**mClosedResistance).rows() < 3 || (**mOpenResistance).rows() < 3)
    return;

  for (UInt phase = 0; phase < 3; ++phase) {
    setEffectiveResistance(phase, closed ? (**mClosedResistance)(phase, phase)
                                         : (**mOpenResistance)(phase, phase));
  }
}

void EMT::Ph3::Switch::resetExponentialTransition() {
  **mExponentialTransitionActive = false;
  **mExponentialTransitionClosing = false;
  **mExponentialProgress = 0.0;
  **mExponentialTransitionStartTime = -1.0;
  **mExponentialTransitionEndTime = -1.0;
  mExponentialTransitionStarted = false;
}

void EMT::Ph3::Switch::validateExponentialResistanceParameters() const {
  if ((**mClosedResistance).rows() < 3 || (**mOpenResistance).rows() < 3) {
    throw std::invalid_argument(
        "EMT::Ph3::Switch exponential mode requires 3x3 resistance matrices.");
  }

  if (mExponentialSwitchingTime <= 0.0) {
    throw std::invalid_argument(
        "EMT::Ph3::Switch exponential switching time must be positive.");
  }

  for (UInt phase = 0; phase < 3; ++phase) {
    const Real rClosed = (**mClosedResistance)(phase, phase);
    const Real rOpen = (**mOpenResistance)(phase, phase);

    if (rClosed <= 0.0 || rOpen <= 0.0) {
      throw std::invalid_argument(
          "EMT::Ph3::Switch exponential mode requires positive diagonal "
          "open and closed resistances.");
    }
  }
}

Real EMT::Ph3::Switch::exponentialResistance(UInt phase, Real alpha) const {
  const Real rClosed = (**mClosedResistance)(phase, phase);
  const Real rOpen = (**mOpenResistance)(phase, phase);
  const Real boundedAlpha = std::max(0.0, std::min(1.0, alpha));

  return std::exp(std::log(rClosed) +
                  boundedAlpha * (std::log(rOpen) - std::log(rClosed)));
}

void EMT::Ph3::Switch::updateExponentialTransition(Real time) {
  if (!**mExponentialTransitionActive)
    return;

  if (!mExponentialTransitionStarted) {
    mExponentialTransitionStarted = true;
    **mExponentialTransitionStartTime = time;
    **mExponentialTransitionEndTime = time + mExponentialSwitchingTime;
  }

  // Prepare R(t+dt) in post-step so that the next MNA solve at t+dt stamps
  // the intended resistance trajectory without requiring a time-aware event API.
  const Real targetTime = time + mTimeStep;
  const Real elapsed = (targetTime - **mExponentialTransitionStartTime) /
                       mExponentialSwitchingTime;
  const Real boundedElapsed = std::max(0.0, std::min(1.0, elapsed));

  const Bool closing = **mExponentialTransitionClosing;

  // alpha is the position on the resistance path: 0 at R_closed, 1 at R_open.
  const Real alpha = closing ? 1.0 - boundedElapsed : boundedElapsed;

  **mExponentialProgress = alpha;

  for (UInt phase = 0; phase < 3; ++phase)
    setEffectiveResistance(phase, exponentialResistance(phase, alpha));

  if (boundedElapsed >= 1.0) {
    synchronizeEffectiveResistance(closing);
    setAllPoles(closing);
    **mOpeningRequested = false;
    **mExponentialTransitionActive = false;
    **mExponentialTransitionClosing = false;

    SPDLOG_LOGGER_INFO(
        mSLog,
        "Exponential ZCS-emulation {:s} completed: start={:.9f}s, "
        "end={:.9f}s, duration={:.6e}s.",
        closing ? "closing" : "opening", **mExponentialTransitionStartTime,
        **mExponentialTransitionEndTime, mExponentialSwitchingTime);
  }
}

MatrixFixedSize<3, 3> EMT::Ph3::Switch::currentConductanceMatrix() const {
  if (mSwitchingMode == SwitchingMode::Ideal) {
    MatrixFixedSize<3, 3> conductance = (**mIsClosed)
                                            ? (**mClosedResistance).inverse()
                                            : (**mOpenResistance).inverse();
    return conductance;
  }

  MatrixFixedSize<3, 3> conductance = MatrixFixedSize<3, 3>::Zero();

  for (UInt phase = 0; phase < 3; ++phase) {
    Real resistance = 0.0;

    if (mSwitchingMode == SwitchingMode::CurrentZero) {
      resistance = poleClosed(phase) ? (**mClosedResistance)(phase, phase)
                                     : (**mOpenResistance)(phase, phase);
    } else {
      resistance = effectiveResistance(phase);
    }

    conductance(phase, phase) = 1.0 / resistance;
  }

  return conductance;
}

Bool EMT::Ph3::Switch::currentCrossedZero(Real previousCurrent,
                                          Real current) const {
  if (std::abs(current) <= mZeroCrossingTolerance)
    return true;

  return (previousCurrent > mZeroCrossingTolerance &&
          current < -mZeroCrossingTolerance) ||
         (previousCurrent < -mZeroCrossingTolerance &&
          current > mZeroCrossingTolerance);
}

Real EMT::Ph3::Switch::interpolateZeroCrossingTime(Real previousCurrent,
                                                   Real current,
                                                   Real previousTime,
                                                   Real currentTime) const {
  const Real denominator = std::abs(previousCurrent) + std::abs(current);

  if (denominator <= mZeroCrossingTolerance)
    return currentTime;

  const Real fraction = std::abs(previousCurrent) / denominator;

  return previousTime + fraction * (currentTime - previousTime);
}

void EMT::Ph3::Switch::setZeroCrossingTime(UInt phase, Real time) {
  switch (phase) {
  case 0:
    **mZeroCrossingTimeA = time;
    return;
  case 1:
    **mZeroCrossingTimeB = time;
    return;
  case 2:
    **mZeroCrossingTimeC = time;
    return;
  default:
    throw std::out_of_range("EMT::Ph3::Switch phase index must be 0, 1, or 2.");
  }
}

void EMT::Ph3::Switch::updateZeroCrossingState(Real time) {
  const Matrix current = **mIntfCurrent;

  if (current.rows() != 3)
    return;

  // While no opening request is active, continuously maintain a clean
  // current history for diagnostics and for a later opening command.
  if (!**mOpeningRequested) {
    mPreviousCurrent = current;
    mPreviousCurrentTime = time;
    mPreviousCurrentValid = true;
    return;
  }

  // The first sample after the opening command must not use a sample from
  // before the command for a sign-change test. It may still open a pole if
  // that first post-command sample itself is already at zero.
  if (mResetZeroCrossingHistory || !mPreviousCurrentValid) {
    for (UInt phase = 0; phase < 3; ++phase) {
      if (poleClosed(phase) &&
          std::abs(current(phase, 0)) <= mZeroCrossingTolerance) {
        setPoleClosed(phase, false);
        setZeroCrossingTime(phase, time);

        SPDLOG_LOGGER_INFO(mSLog,
                           "Phase {} opened at sampled current zero: "
                           "t={:.9f}s, i={:.6e}A",
                           phase, time, current(phase, 0));
      }
    }

    mPreviousCurrent = current;
    mPreviousCurrentTime = time;
    mPreviousCurrentValid = true;
    mResetZeroCrossingHistory = false;

    if (allPolesOpen())
      **mOpeningRequested = false;

    return;
  }

  for (UInt phase = 0; phase < 3; ++phase) {
    if (!poleClosed(phase))
      continue;

    const Real previous = mPreviousCurrent(phase, 0);
    const Real present = current(phase, 0);

    if (!currentCrossedZero(previous, present))
      continue;

    const Real zeroTime = interpolateZeroCrossingTime(
        previous, present, mPreviousCurrentTime, time);

    setPoleClosed(phase, false);
    setZeroCrossingTime(phase, zeroTime);

    SPDLOG_LOGGER_INFO(mSLog,
                       "Phase {} current zero detected: "
                       "t_z={:.9f}s, i_prev={:.6e}A, i={:.6e}A. "
                       "Pole opened.",
                       phase, zeroTime, previous, present);
  }

  mPreviousCurrent = current;
  mPreviousCurrentTime = time;
  mPreviousCurrentValid = true;

  if (allPolesOpen()) {
    **mOpeningRequested = false;

    SPDLOG_LOGGER_INFO(mSLog, "Current-zero interruption completed: "
                              "all three breaker poles are open.");
  }
}
