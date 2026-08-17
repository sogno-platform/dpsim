// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <dpsim-models/DP/DP_Ph3_Switch.h>

using namespace CPS;

DP::Ph3::Switch::Switch(String uid, String name, Logger::Level logLevel)
    : MNASimPowerComp<Complex>(uid, name, false, true, logLevel),
      Base::Ph3::Switch(mAttributes),
      mOpeningRequested(mAttributes->create<Bool>("opening_requested")),
      mPoleClosedA(mAttributes->create<Bool>("pole_closed_a")),
      mPoleClosedB(mAttributes->create<Bool>("pole_closed_b")),
      mPoleClosedC(mAttributes->create<Bool>("pole_closed_c")),
      mInstantCurrentA(mAttributes->create<Real>("i_instantaneous_a")),
      mInstantCurrentB(mAttributes->create<Real>("i_instantaneous_b")),
      mInstantCurrentC(mAttributes->create<Real>("i_instantaneous_c")),
      mZeroCrossingTimeA(mAttributes->create<Real>("zero_crossing_time_a")),
      mZeroCrossingTimeB(mAttributes->create<Real>("zero_crossing_time_b")),
      mZeroCrossingTimeC(mAttributes->create<Real>("zero_crossing_time_c")),
      mExponentialTransitionActive(
          mAttributes->create<Bool>("exponential_transition_active")),
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
  mPhaseType = PhaseType::ABC;
  setTerminalNumber(2);

  **mIntfVoltage = MatrixComp::Zero(3, 1);
  **mIntfCurrent = MatrixComp::Zero(3, 1);

  **mOpeningRequested = false;
  setAllPoles(false);

  **mInstantCurrentA = 0.0;
  **mInstantCurrentB = 0.0;
  **mInstantCurrentC = 0.0;

  resetZeroCrossingTimes();

  **mExponentialTransitionActive = false;
  **mExponentialProgress = 0.0;
  **mExponentialTransitionStartTime = -1.0;
  **mExponentialTransitionEndTime = -1.0;
  **mEffectiveResistanceA = 0.0;
  **mEffectiveResistanceB = 0.0;
  **mEffectiveResistanceC = 0.0;
}

SimPowerComp<Complex>::Ptr DP::Ph3::Switch::clone(String name) {
  auto copy = Switch::make(name, mLogLevel);
  copy->setParameters(**mOpenResistance, **mClosedResistance, **mIsClosed);
  copy->setSwitchingMode(mSwitchingMode);
  copy->setZeroCrossingTolerance(mZeroCrossingTolerance);
  copy->setExponentialSwitchingTime(mExponentialSwitchingTime);
  return copy;
}

void DP::Ph3::Switch::setSwitchingMode(SwitchingMode mode) {
  mSwitchingMode = mode;

  setAllPoles(**mIsClosed);
  **mOpeningRequested = false;
  resetZeroCrossingTimes();

  mPreviousCurrentValid = false;
  mResetZeroCrossingHistory = false;

  resetExponentialTransition();
  synchronizeEffectiveResistance(**mIsClosed);
}

void DP::Ph3::Switch::setZeroCrossingTolerance(Real tolerance) {
  if (tolerance < 0.0) {
    throw std::invalid_argument(
        "DP::Ph3::Switch zero-crossing tolerance must be non-negative.");
  }

  mZeroCrossingTolerance = tolerance;
}

void DP::Ph3::Switch::setExponentialSwitchingTime(Real switchingTime) {
  if (switchingTime <= 0.0) {
    throw std::invalid_argument(
        "DP::Ph3::Switch exponential switching time must be positive.");
  }

  mExponentialSwitchingTime = switchingTime;
}

void DP::Ph3::Switch::openSwitch() {
  Base::Ph3::Switch::openSwitch();

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

    SPDLOG_LOGGER_INFO(
        mSLog, "DP current-zero opening command received. "
               "Waiting for reconstructed physical phase-current zeros.");
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
                     "DP exponential ZCS-emulation opening command received. "
                     "Switching duration={:.6e}s.",
                     mExponentialSwitchingTime);
}

void DP::Ph3::Switch::closeSwitch() {
  Base::Ph3::Switch::closeSwitch();

  **mOpeningRequested = false;
  setAllPoles(true);
  resetZeroCrossingTimes();
  mResetZeroCrossingHistory = false;

  resetExponentialTransition();
  synchronizeEffectiveResistance(true);

  SPDLOG_LOGGER_INFO(
      mSLog, "DP switch closing command: all three physical poles closed.");
}

void DP::Ph3::Switch::initializeFromNodesAndTerminals(Real frequency) {
  mTerminals[0]->setPhaseType(PhaseType::ABC);
  mTerminals[1]->setPhaseType(PhaseType::ABC);

  Matrix impedance = (**mIsClosed) ? **mClosedResistance : **mOpenResistance;

  // DP::Ph3 MNA quantities are phase-peak complex envelopes. Power-flow
  // node voltages are line-line RMS, so convert the A-phase voltage to
  // phase peak and construct the balanced B/C envelopes explicitly.
  MatrixComp vInitABC = MatrixComp::Zero(3, 1);
  vInitABC(0, 0) =
      RMS3PH_TO_PEAK1PH * (initialSingleVoltage(1) - initialSingleVoltage(0));
  vInitABC(1, 0) = vInitABC(0, 0) * SHIFT_TO_PHASE_B;
  vInitABC(2, 0) = vInitABC(0, 0) * SHIFT_TO_PHASE_C;

  **mIntfVoltage = vInitABC;
  **mIntfCurrent = impedance.inverse() * vInitABC;

  setAllPoles(**mIsClosed);
  **mOpeningRequested = false;
  resetZeroCrossingTimes();
  resetExponentialTransition();
  synchronizeEffectiveResistance(**mIsClosed);

  SPDLOG_LOGGER_INFO(mSLog,
                     "\n--- Initialization from powerflow ---"
                     "\nVoltage envelope:"
                     "\n{:s}"
                     "\nCurrent envelope:"
                     "\n{:s}"
                     "\nTerminal 0 voltage envelope:"
                     "\n{:s}"
                     "\nTerminal 1 voltage envelope:"
                     "\n{:s}"
                     "\nSwitching mode: {:s}"
                     "\nInitial breaker state: {:s}"
                     "\n--- Initialization from powerflow finished ---",
                     Logger::phasorMatrixToString(**mIntfVoltage),
                     Logger::phasorMatrixToString(**mIntfCurrent),
                     Logger::phasorMatrixToString(initialVoltage(0)),
                     Logger::phasorMatrixToString(initialVoltage(1)),
                     mSwitchingMode == SwitchingMode::Ideal
                         ? "Ideal"
                         : (mSwitchingMode == SwitchingMode::CurrentZero
                                ? "CurrentZero"
                                : "ExponentialZCSEmulation"),
                     **mIsClosed ? "closed" : "open");
}

void DP::Ph3::Switch::mnaCompInitialize(Real omega, Real timeStep,
                                        Attribute<Matrix>::Ptr leftVector) {
  updateMatrixNodeIndices();
  mTimeStep = timeStep;
  **mRightVector = Matrix::Zero(0, 0);

  // omega is the DP reference-frame angular frequency supplied by the solver.
  mShiftOmega = omega;

  // Final safeguard in case setSwitchingMode() was called before setParameters().
  setAllPoles(**mIsClosed);
  synchronizeEffectiveResistance(**mIsClosed);
  resetExponentialTransition();

  mPoleClosedPrev = {{
      **mPoleClosedA,
      **mPoleClosedB,
      **mPoleClosedC,
  }};

  mIsClosedPrev = **mIsClosed;

  const Matrix instantaneous = reconstructInstantaneousCurrent(0.0);

  setInstantaneousCurrentAttributes(instantaneous);

  mPreviousInstantaneousCurrent = instantaneous;
  mEffectiveResistancePrev = {{
      **mEffectiveResistanceA,
      **mEffectiveResistanceB,
      **mEffectiveResistanceC,
  }};

  mPreviousCurrentTime = 0.0;
  mPreviousCurrentValid = true;
  mResetZeroCrossingHistory = false;
}

Bool DP::Ph3::Switch::mnaIsClosed() {
  if (mSwitchingMode == SwitchingMode::Ideal)
    return **mIsClosed;

  return allPolesClosed();
}

Bool DP::Ph3::Switch::supportsPrecomputedSystemMatrices() const {
  return mSwitchingMode == SwitchingMode::Ideal;
}

void DP::Ph3::Switch::mnaCompApplySystemMatrixStamp(
    SparseMatrixRow &systemMatrix) {
  const MatrixFixedSizeComp<3, 3> conductance = currentConductanceMatrix();

  MNAStampUtils::stampAdmittanceMatrix(
      conductance, systemMatrix, matrixNodeIndex(0), matrixNodeIndex(1),
      terminalNotGrounded(0), terminalNotGrounded(1), mSLog);
}

void DP::Ph3::Switch::mnaCompApplySwitchSystemMatrixStamp(
    Bool closed, SparseMatrixRow &systemMatrix, Int freqIdx) {
  // Conventional precomputed two-state path. CurrentZero mode bypasses this
  // by returning supportsPrecomputedSystemMatrices() == false.
  MatrixFixedSizeComp<3, 3> conductance = MatrixFixedSizeComp<3, 3>::Zero();

  conductance.real() =
      closed ? (**mClosedResistance).inverse() : (**mOpenResistance).inverse();

  MNAStampUtils::stampAdmittanceMatrix(
      conductance, systemMatrix, matrixNodeIndex(0), matrixNodeIndex(1),
      terminalNotGrounded(0), terminalNotGrounded(1), mSLog);
}

void DP::Ph3::Switch::mnaCompAddPostStepDependencies(
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

  modifiedAttributes.push_back(mInstantCurrentA);
  modifiedAttributes.push_back(mInstantCurrentB);
  modifiedAttributes.push_back(mInstantCurrentC);

  modifiedAttributes.push_back(mZeroCrossingTimeA);
  modifiedAttributes.push_back(mZeroCrossingTimeB);
  modifiedAttributes.push_back(mZeroCrossingTimeC);

  modifiedAttributes.push_back(mExponentialTransitionActive);
  modifiedAttributes.push_back(mExponentialProgress);
  modifiedAttributes.push_back(mExponentialTransitionStartTime);
  modifiedAttributes.push_back(mExponentialTransitionEndTime);
  modifiedAttributes.push_back(mEffectiveResistanceA);
  modifiedAttributes.push_back(mEffectiveResistanceB);
  modifiedAttributes.push_back(mEffectiveResistanceC);
}

void DP::Ph3::Switch::mnaCompPostStep(Real time, Int timeStepCount,
                                      Attribute<Matrix>::Ptr &leftVector) {
  mnaCompUpdateVoltage(**leftVector);
  mnaCompUpdateCurrent(**leftVector);

  const Matrix instantaneous = reconstructInstantaneousCurrent(time);

  setInstantaneousCurrentAttributes(instantaneous);

  if (mSwitchingMode == SwitchingMode::CurrentZero) {
    updateZeroCrossingState(time);
  } else if (mSwitchingMode == SwitchingMode::ExponentialZCSEmulation) {
    updateExponentialTransition(time);
  }
}

void DP::Ph3::Switch::mnaCompUpdateVoltage(const Matrix &leftVector) {
  **mIntfVoltage = MatrixComp::Zero(3, 1);

  if (terminalNotGrounded(1)) {
    (**mIntfVoltage)(0, 0) =
        Math::complexFromVectorElement(leftVector, matrixNodeIndex(1, 0));
    (**mIntfVoltage)(1, 0) =
        Math::complexFromVectorElement(leftVector, matrixNodeIndex(1, 1));
    (**mIntfVoltage)(2, 0) =
        Math::complexFromVectorElement(leftVector, matrixNodeIndex(1, 2));
  }

  if (terminalNotGrounded(0)) {
    (**mIntfVoltage)(0, 0) -=
        Math::complexFromVectorElement(leftVector, matrixNodeIndex(0, 0));
    (**mIntfVoltage)(1, 0) -=
        Math::complexFromVectorElement(leftVector, matrixNodeIndex(0, 1));
    (**mIntfVoltage)(2, 0) -=
        Math::complexFromVectorElement(leftVector, matrixNodeIndex(0, 2));
  }
}

void DP::Ph3::Switch::mnaCompUpdateCurrent(const Matrix &leftVector) {
  **mIntfCurrent = currentConductanceMatrix() * **mIntfVoltage;
}

Bool DP::Ph3::Switch::hasParameterChanged() {
  if (mSwitchingMode == SwitchingMode::Ideal) {
    if (mIsClosedPrev != **mIsClosed) {
      mIsClosedPrev = **mIsClosed;
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
      mPoleClosedPrev[phase] = poleClosedNow[phase];
      changed = true;
    }
  }

  return changed;
}

Bool DP::Ph3::Switch::poleClosed(UInt phase) const {
  switch (phase) {
  case 0:
    return **mPoleClosedA;
  case 1:
    return **mPoleClosedB;
  case 2:
    return **mPoleClosedC;
  default:
    throw std::out_of_range("DP::Ph3::Switch phase index must be 0, 1, or 2.");
  }
}

void DP::Ph3::Switch::setPoleClosed(UInt phase, Bool closed) {
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
    throw std::out_of_range("DP::Ph3::Switch phase index must be 0, 1, or 2.");
  }

  // Keep the resistance diagnostics synchronized with the discrete
  // CurrentZero pole state. Exponential mode manages R(t) independently.
  if (mSwitchingMode == SwitchingMode::CurrentZero &&
      (**mClosedResistance).rows() >= 3 && (**mOpenResistance).rows() >= 3) {
    setEffectiveResistance(phase, closed ? (**mClosedResistance)(phase, phase)
                                         : (**mOpenResistance)(phase, phase));
  }
}

void DP::Ph3::Switch::setAllPoles(Bool closed) {
  **mPoleClosedA = closed;
  **mPoleClosedB = closed;
  **mPoleClosedC = closed;
}

Bool DP::Ph3::Switch::allPolesClosed() const {
  return **mPoleClosedA && **mPoleClosedB && **mPoleClosedC;
}

Bool DP::Ph3::Switch::allPolesOpen() const {
  return !**mPoleClosedA && !**mPoleClosedB && !**mPoleClosedC;
}

void DP::Ph3::Switch::resetZeroCrossingTimes() {
  **mZeroCrossingTimeA = -1.0;
  **mZeroCrossingTimeB = -1.0;
  **mZeroCrossingTimeC = -1.0;
}

Real DP::Ph3::Switch::effectiveResistance(UInt phase) const {
  switch (phase) {
  case 0:
    return **mEffectiveResistanceA;
  case 1:
    return **mEffectiveResistanceB;
  case 2:
    return **mEffectiveResistanceC;
  default:
    throw std::out_of_range("DP::Ph3::Switch phase index must be 0, 1, or 2.");
  }
}

void DP::Ph3::Switch::setEffectiveResistance(UInt phase, Real resistance) {
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
    throw std::out_of_range("DP::Ph3::Switch phase index must be 0, 1, or 2.");
  }
}

void DP::Ph3::Switch::synchronizeEffectiveResistance(Bool closed) {
  if ((**mClosedResistance).rows() < 3 || (**mOpenResistance).rows() < 3)
    return;

  for (UInt phase = 0; phase < 3; ++phase) {
    setEffectiveResistance(phase, closed ? (**mClosedResistance)(phase, phase)
                                         : (**mOpenResistance)(phase, phase));
  }
}

void DP::Ph3::Switch::resetExponentialTransition() {
  **mExponentialTransitionActive = false;
  **mExponentialProgress = 0.0;
  **mExponentialTransitionStartTime = -1.0;
  **mExponentialTransitionEndTime = -1.0;
  mExponentialTransitionStarted = false;
}

void DP::Ph3::Switch::validateExponentialResistanceParameters() const {
  if ((**mClosedResistance).rows() < 3 || (**mOpenResistance).rows() < 3) {
    throw std::invalid_argument(
        "DP::Ph3::Switch exponential mode requires 3x3 resistance matrices.");
  }

  if (mExponentialSwitchingTime <= 0.0) {
    throw std::invalid_argument(
        "DP::Ph3::Switch exponential switching time must be positive.");
  }

  for (UInt phase = 0; phase < 3; ++phase) {
    const Real rClosed = (**mClosedResistance)(phase, phase);
    const Real rOpen = (**mOpenResistance)(phase, phase);

    if (rClosed <= 0.0 || rOpen <= 0.0) {
      throw std::invalid_argument(
          "DP::Ph3::Switch exponential mode requires positive diagonal "
          "open and closed resistances.");
    }
  }
}

Real DP::Ph3::Switch::exponentialResistance(UInt phase, Real alpha) const {
  const Real rClosed = (**mClosedResistance)(phase, phase);
  const Real rOpen = (**mOpenResistance)(phase, phase);
  const Real boundedAlpha = std::max(0.0, std::min(1.0, alpha));

  return std::exp(std::log(rClosed) +
                  boundedAlpha * (std::log(rOpen) - std::log(rClosed)));
}

void DP::Ph3::Switch::updateExponentialTransition(Real time) {
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

  for (UInt phase = 0; phase < 3; ++phase)
    setEffectiveResistance(phase, exponentialResistance(phase, boundedAlpha));

  if (boundedAlpha >= 1.0) {
    synchronizeEffectiveResistance(false);
    setAllPoles(false);
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

MatrixFixedSizeComp<3, 3> DP::Ph3::Switch::currentConductanceMatrix() const {
  MatrixFixedSizeComp<3, 3> conductance = MatrixFixedSizeComp<3, 3>::Zero();

  if (mSwitchingMode == SwitchingMode::Ideal) {
    conductance.real() = (**mIsClosed) ? (**mClosedResistance).inverse()
                                       : (**mOpenResistance).inverse();
    return conductance;
  }

  for (UInt phase = 0; phase < 3; ++phase) {
    Real resistance = 0.0;

    if (mSwitchingMode == SwitchingMode::CurrentZero) {
      resistance = poleClosed(phase) ? (**mClosedResistance)(phase, phase)
                                     : (**mOpenResistance)(phase, phase);
    } else {
      resistance = effectiveResistance(phase);
    }

    conductance(phase, phase) = Complex(1.0 / resistance, 0.0);
  }

  return conductance;
}

Matrix DP::Ph3::Switch::reconstructInstantaneousCurrent(Real time) const {
  Matrix instantaneous = Matrix::Zero(3, 1);

  // DP::Ph3 quantities are phase-peak complex envelopes. Restore the removed
  // reference-frequency carrier; no additional RMS-to-peak scaling is needed.
  const Complex carrier = std::polar<Real>(1.0, mShiftOmega * time);

  for (UInt phase = 0; phase < 3; ++phase) {
    instantaneous(phase, 0) = std::real((**mIntfCurrent)(phase, 0) * carrier);
  }

  return instantaneous;
}

void DP::Ph3::Switch::setInstantaneousCurrentAttributes(const Matrix &current) {
  **mInstantCurrentA = current(0, 0);
  **mInstantCurrentB = current(1, 0);
  **mInstantCurrentC = current(2, 0);
}

Bool DP::Ph3::Switch::currentCrossedZero(Real previousCurrent,
                                         Real current) const {
  if (std::abs(current) <= mZeroCrossingTolerance)
    return true;

  return (previousCurrent > mZeroCrossingTolerance &&
          current < -mZeroCrossingTolerance) ||
         (previousCurrent < -mZeroCrossingTolerance &&
          current > mZeroCrossingTolerance);
}

Real DP::Ph3::Switch::interpolateZeroCrossingTime(Real previousCurrent,
                                                  Real current,
                                                  Real previousTime,
                                                  Real currentTime) const {
  const Real denominator = std::abs(previousCurrent) + std::abs(current);

  if (denominator <= mZeroCrossingTolerance)
    return currentTime;

  const Real fraction = std::abs(previousCurrent) / denominator;

  return previousTime + fraction * (currentTime - previousTime);
}

void DP::Ph3::Switch::setZeroCrossingTime(UInt phase, Real time) {
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
    throw std::out_of_range("DP::Ph3::Switch phase index must be 0, 1, or 2.");
  }
}

void DP::Ph3::Switch::updateZeroCrossingState(Real time) {
  const Matrix current = reconstructInstantaneousCurrent(time);

  // Keep a current history while no opening command is active.
  if (!**mOpeningRequested) {
    mPreviousInstantaneousCurrent = current;
    mPreviousCurrentTime = time;
    mPreviousCurrentValid = true;
    return;
  }

  // First post-command sample: do not compare against the pre-command sample.
  if (mResetZeroCrossingHistory || !mPreviousCurrentValid) {
    for (UInt phase = 0; phase < 3; ++phase) {
      if (poleClosed(phase) &&
          std::abs(current(phase, 0)) <= mZeroCrossingTolerance) {
        setPoleClosed(phase, false);
        setZeroCrossingTime(phase, time);

        SPDLOG_LOGGER_INFO(
            mSLog,
            "DP phase {} opened at sampled physical current zero: "
            "t={:.9f}s, i={:.6e}A",
            phase, time, current(phase, 0));
      }
    }

    mPreviousInstantaneousCurrent = current;
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

    const Real previous = mPreviousInstantaneousCurrent(phase, 0);
    const Real present = current(phase, 0);

    if (!currentCrossedZero(previous, present))
      continue;

    const Real zeroTime = interpolateZeroCrossingTime(
        previous, present, mPreviousCurrentTime, time);

    setPoleClosed(phase, false);
    setZeroCrossingTime(phase, zeroTime);

    SPDLOG_LOGGER_INFO(mSLog,
                       "DP phase {} physical current zero detected: "
                       "t_z={:.9f}s, i_prev={:.6e}A, i={:.6e}A. "
                       "Pole opened.",
                       phase, zeroTime, previous, present);
  }

  mPreviousInstantaneousCurrent = current;
  mPreviousCurrentTime = time;
  mPreviousCurrentValid = true;

  if (allPolesOpen()) {
    **mOpeningRequested = false;

    SPDLOG_LOGGER_INFO(mSLog, "DP current-zero interruption completed: "
                              "all three physical breaker poles are open.");
  }
}
