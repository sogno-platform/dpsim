/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/EMT/EMT_Ph3_RXLoad.h>
#include <dpsim-models/MNAStampUtils.h>

using namespace CPS;

EMT::Ph3::RXLoad::RXLoad(String uid, String name, Logger::Level logLevel)
    : RXLoad(uid, name, true, logLevel) {}

EMT::Ph3::RXLoad::RXLoad(String uid, String name, Bool useSubcomponents,
                         Logger::Level logLevel)
    : CompositePowerComp<Real>(uid, name, true, true, logLevel),
      mActivePower(mAttributes->create<Matrix>("P")),
      mReactivePower(mAttributes->create<Matrix>("Q")),
      mNomVoltage(mAttributes->create<Real>("V_nom")),
      mReactanceInSeries(false) {
  mUseSubcomponents = useSubcomponents;
  mDirectState = mAttributes->create<Matrix>("direct_state");
  **mDirectState = Matrix::Zero(12, 1);
  mPhaseType = PhaseType::ABC;
  setTerminalNumber(1);

  SPDLOG_LOGGER_INFO(mSLog, "Create {} {}", this->type(), name);
  **mIntfVoltage = Matrix::Zero(3, 1);
  **mIntfCurrent = Matrix::Zero(3, 1);
  mSLog->flush();
}

EMT::Ph3::RXLoad::RXLoad(String name, Logger::Level logLevel)
    : RXLoad(name, name, logLevel) {}

EMT::Ph3::RXLoad::RXLoad(String name, Matrix activePower, Matrix reactivePower,
                         Real volt, Logger::Level logLevel)
    : RXLoad(name, logLevel) {
  **mActivePower = activePower;
  **mReactivePower = reactivePower;
  mPower = MatrixComp::Zero(3, 3);
  mPower << Complex((**mActivePower)(0, 0), (**mReactivePower)(0, 0)),
      Complex((**mActivePower)(0, 1), (**mReactivePower)(0, 1)),
      Complex((**mActivePower)(0, 2), (**mReactivePower)(0, 2)),
      Complex((**mActivePower)(1, 0), (**mReactivePower)(1, 0)),
      Complex((**mActivePower)(1, 1), (**mReactivePower)(1, 1)),
      Complex((**mActivePower)(1, 2), (**mReactivePower)(1, 2)),
      Complex((**mActivePower)(2, 0), (**mReactivePower)(2, 0)),
      Complex((**mActivePower)(2, 1), (**mReactivePower)(2, 1)),
      Complex((**mActivePower)(2, 2), (**mReactivePower)(2, 2));

  **mNomVoltage = volt;
  initPowerFromTerminal = false;
}

SimPowerComp<Real>::Ptr EMT::Ph3::RXLoad::clone(String name) {
  auto copy = RXLoad::make(name, mUseSubcomponents, mLogLevel);
  if (!initPowerFromTerminal)
    copy->setParameters(**mActivePower, **mReactivePower, **mNomVoltage,
                        mReactanceInSeries);
  return copy;
}

void EMT::Ph3::RXLoad::setParameters(Matrix activePower, Matrix reactivePower,
                                     Real volt, bool reactanceInSeries) {
  if (!mUseSubcomponents && reactanceInSeries)
    throw std::invalid_argument(
        "Direct RXLoad mode currently supports parallel R-X loads only.");
  **mActivePower = activePower;
  **mReactivePower = reactivePower;
  mReactanceInSeries = reactanceInSeries;

  if (mReactanceInSeries) {
    setVirtualNodeNumber(1);
  }
  // complex power
  mPower = MatrixComp::Zero(3, 3);
  mPower(0, 0) = {(**mActivePower)(0, 0), (**mReactivePower)(0, 0)};
  mPower(1, 1) = {(**mActivePower)(1, 1), (**mReactivePower)(1, 1)};
  mPower(2, 2) = {(**mActivePower)(2, 2), (**mReactivePower)(2, 2)};

  **mNomVoltage = volt;

  SPDLOG_LOGGER_INFO(mSLog,
                     "\nActive Power [W]: {}"
                     "\nReactive Power [VAr]: {}",
                     Logger::matrixToString(**mActivePower),
                     Logger::matrixToString(**mReactivePower));
  SPDLOG_LOGGER_INFO(mSLog, "Nominal Voltage={} [V]", **mNomVoltage);

  initPowerFromTerminal = false;
}

void EMT::Ph3::RXLoad::createSubComponents() {
  if (mSubCompCreated)
    return;
  mSubCompCreated = true;
  if (!mUseSubcomponents)
    return;

  // Intentionally empty: which of R/L/C exist depends on the load power sign, known only in
  // initializeParentFromNodesAndTerminals(), where the sub-components are created. Safe: any
  // series-mode virtual node is already declared in setParameters().
}

void EMT::Ph3::RXLoad::initializeParentFromNodesAndTerminals(Real frequency) {
  Real omega = 2. * PI * frequency;

  if (initPowerFromTerminal) {
    **mActivePower = Matrix::Zero(3, 3);
    (**mActivePower)(0, 0) = mTerminals[0]->singleActivePower() / 3.;
    (**mActivePower)(1, 1) = mTerminals[0]->singleActivePower() / 3.;
    (**mActivePower)(2, 2) = mTerminals[0]->singleActivePower() / 3.;

    **mReactivePower = Matrix::Zero(3, 3);
    (**mReactivePower)(0, 0) = mTerminals[0]->singleReactivePower() / 3.;
    (**mReactivePower)(1, 1) = mTerminals[0]->singleReactivePower() / 3.;
    (**mReactivePower)(2, 2) = mTerminals[0]->singleReactivePower() / 3.;

    // complex power
    mPower = MatrixComp::Zero(3, 3);
    mPower(0, 0) = {(**mActivePower)(0, 0), (**mReactivePower)(0, 0)};
    mPower(1, 1) = {(**mActivePower)(1, 1), (**mReactivePower)(1, 1)};
    mPower(2, 2) = {(**mActivePower)(2, 2), (**mReactivePower)(2, 2)};

    **mNomVoltage = std::abs(mTerminals[0]->initialSingleVoltage());

    SPDLOG_LOGGER_INFO(mSLog,
                       "\nActive Power [W]: {}"
                       "\nReactive Power [VAr]: {}",
                       Logger::matrixToString(**mActivePower),
                       Logger::matrixToString(**mReactivePower));
    SPDLOG_LOGGER_INFO(mSLog, "Nominal Voltage={} [V]", **mNomVoltage);
  }

  // Compute derived impedance values and create+parametrize sub-components
  // now that power and voltage are guaranteed to be known.
  if ((**mActivePower)(0, 0) != 0)
    mResistance =
        std::pow(**mNomVoltage / sqrt(3), 2) * (**mActivePower).inverse();
  else
    mResistance = Matrix::Zero(3, 3);

  if ((**mReactivePower)(0, 0) != 0)
    mReactance =
        std::pow(**mNomVoltage / sqrt(3), 2) * (**mReactivePower).inverse();
  else
    mReactance = Matrix::Zero(3, 3);

  if (!mUseSubcomponents) {
    const Real omega = 2. * PI * frequency;
    if (mReactance(0, 0) > 0.)
      mInductance = mReactance / omega;
    else if (mReactance(0, 0) < 0.)
      mCapacitance = -mReactance.inverse() / omega;
    mDirectTotalConductance =
        mResistance.norm() > 0. ? mResistance.inverse()(0, 0) : 0.;
    mDirectHasInductor = mReactance(0, 0) > 0.;
    mDirectHasCapacitor = mReactance(0, 0) < 0.;
    MatrixComp voltage = MatrixComp::Zero(3, 1);
    voltage(0) = RMS3PH_TO_PEAK1PH * mTerminals[0]->initialSingleVoltage();
    voltage(1) = voltage(0) * SHIFT_TO_PHASE_B;
    voltage(2) = voltage(0) * SHIFT_TO_PHASE_C;
    MatrixComp loadG = MatrixComp::Zero(3, 3);
    if (mResistance.norm() > 0.)
      loadG = mResistance.inverse().cast<Complex>();
    **mIntfVoltage = voltage.real();
    (**mDirectState).setZero();
    if (mDirectHasInductor) {
      const MatrixComp yL =
          (Complex(0., omega) * mInductance.cast<Complex>()).inverse();
      (**mDirectState).block<3, 1>(0, 0) = voltage.real();
      (**mDirectState).block<3, 1>(3, 0) = (yL * voltage).real();
    }
    if (mDirectHasCapacitor) {
      const MatrixComp yC = Complex(0., omega) * mCapacitance.cast<Complex>();
      (**mDirectState).block<3, 1>(6, 0) = voltage.real();
      (**mDirectState).block<3, 1>(9, 0) = (yC * voltage).real();
    }
    **mIntfCurrent = loadG.real() * voltage.real();
    if (mDirectHasInductor)
      **mIntfCurrent += (**mDirectState).block<3, 1>(3, 0);
    if (mDirectHasCapacitor)
      **mIntfCurrent += (**mDirectState).block<3, 1>(9, 0);
    return;
  }

  if ((**mActivePower)(0, 0) != 0) {
    mSubResistor =
        std::make_shared<EMT::Ph3::Resistor>(**mName + "_res", mLogLevel);
    mSubResistor->setParameters(mResistance);
    if (mReactanceInSeries) {
      mSubResistor->connect({mTerminals[0]->node(), mVirtualNodes[0]});
    } else {
      mSubResistor->connect({SimNode::GND, mTerminals[0]->node()});
    }
    addMNASubComponent(mSubResistor, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                       MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);
  }

  if (mReactance(0, 0) > 0) {
    mInductance = mReactance / omega;

    mSubInductor =
        std::make_shared<EMT::Ph3::Inductor>(**mName + "_ind", mLogLevel);
    mSubInductor->setParameters(mInductance);
    if (mReactanceInSeries) {
      mSubInductor->connect({SimNode::GND, mVirtualNodes[0]});
    } else {
      mSubInductor->connect({SimNode::GND, mTerminals[0]->node()});
    }
    addMNASubComponent(mSubInductor, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                       MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);
  } else if (mReactance(0, 0) < 0) {
    mCapacitance = -1. / omega * mReactance.inverse();

    mSubCapacitor =
        std::make_shared<EMT::Ph3::Capacitor>(**mName + "_cap", mLogLevel);
    mSubCapacitor->setParameters(mCapacitance);
    if (mReactanceInSeries) {
      mSubCapacitor->connect({SimNode::GND, mVirtualNodes[0]});
    } else {
      mSubCapacitor->connect({SimNode::GND, mTerminals[0]->node()});
    }
    addMNASubComponent(mSubCapacitor,
                       MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                       MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);
  }

  MatrixComp vInitABC = MatrixComp::Zero(3, 1);
  vInitABC(0, 0) = RMS3PH_TO_PEAK1PH * mTerminals[0]->initialSingleVoltage();
  vInitABC(1, 0) = vInitABC(0, 0) * SHIFT_TO_PHASE_B;
  vInitABC(2, 0) = vInitABC(0, 0) * SHIFT_TO_PHASE_C;
  **mIntfVoltage = vInitABC.real();

  if (mReactanceInSeries) {
    MatrixComp impedance = MatrixComp::Zero(3, 3);
    impedance << Complex(mResistance(0, 0), mReactance(0, 0)),
        Complex(mResistance(0, 1), mReactance(0, 1)),
        Complex(mResistance(0, 2), mReactance(0, 2)),
        Complex(mResistance(1, 0), mReactance(1, 0)),
        Complex(mResistance(1, 1), mReactance(1, 1)),
        Complex(mResistance(1, 2), mReactance(1, 2)),
        Complex(mResistance(2, 0), mReactance(2, 0)),
        Complex(mResistance(2, 1), mReactance(2, 1)),
        Complex(mResistance(2, 2), mReactance(2, 2));
    MatrixComp iInit = impedance.inverse() * vInitABC;
    **mIntfCurrent = iInit.real();

    // Initialization of virtual node
    // Initial voltage of phase B,C is set after A
    MatrixComp vInitTerm0 = MatrixComp::Zero(3, 1);
    vInitTerm0(0, 0) = RMS3PH_TO_PEAK1PH * initialSingleVoltage(0);
    vInitTerm0(1, 0) = vInitTerm0(0, 0) * SHIFT_TO_PHASE_B;
    vInitTerm0(2, 0) = vInitTerm0(0, 0) * SHIFT_TO_PHASE_C;
    mVirtualNodes[0]->setInitialVoltage(PEAK1PH_TO_RMS3PH *
                                        (vInitTerm0 - mResistance * iInit));
  }

  if ((**mActivePower)(0, 0) != 0) {
    mSubResistor->initializeFromNodesAndTerminals(frequency);
    if (!mReactanceInSeries) {
      **mIntfCurrent += mSubResistor->intfCurrent();
    }
  }

  if (mReactance(0, 0) > 0) {
    mSubInductor->initializeFromNodesAndTerminals(frequency);
    if (!mReactanceInSeries) {
      **mIntfCurrent += mSubInductor->intfCurrent();
    }
  } else if (mReactance(0, 0) < 0) {
    mSubCapacitor->initializeFromNodesAndTerminals(frequency);
    if (!mReactanceInSeries) {
      **mIntfCurrent += mSubCapacitor->intfCurrent();
    }
  }

  SPDLOG_LOGGER_INFO(
      mSLog,
      "\n--- Initialization from powerflow ---"
      "\nVoltage across: {:s}"
      "\nCurrent: {:s}"
      "\nTerminal 0 voltage: {:s}"
      "\nActive Power: {:s}"
      "\nReactive Power: {:s}"
      "\nResistance: {:s}"
      "\nReactance: {:s}"
      "\n--- Initialization from powerflow finished ---",
      Logger::matrixToString(**mIntfVoltage),
      Logger::matrixToString(**mIntfCurrent),
      Logger::phasorToString(RMS3PH_TO_PEAK1PH * initialSingleVoltage(0)),
      Logger::matrixToString(**mActivePower),
      Logger::matrixToString(**mReactivePower),
      Logger::matrixToString(mResistance), Logger::matrixToString(mReactance));
  mSLog->flush();
}

void EMT::Ph3::RXLoad::mnaParentAddPreStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes) {
  if (!mUseSubcomponents)
    prevStepDependencies.push_back(mDirectState);
  modifiedAttributes.push_back(mRightVector);
};

void EMT::Ph3::RXLoad::mnaParentAddPostStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes,
    Attribute<Matrix>::Ptr &leftVector) {
  attributeDependencies.push_back(leftVector);
  modifiedAttributes.push_back(mIntfCurrent);
  modifiedAttributes.push_back(mIntfVoltage);
  if (!mUseSubcomponents)
    modifiedAttributes.push_back(mDirectState);
};

void EMT::Ph3::RXLoad::mnaParentPreStep(Real time, Int timeStepCount) {
  if (!mUseSubcomponents) {
    const Matrix &state = **mDirectState;
    for (Int phase = 0; phase < 3; ++phase) {
      if (mDirectHasInductor)
        mDirectHistoryLoad(phase, 0) =
            mDirectInductorAdmittance * state(phase, 0) + state(phase + 3, 0);
      if (mDirectHasCapacitor)
        mDirectHistoryShunt(phase, 0) =
            -state(phase + 9, 0) -
            mDirectCapacitorAdmittance * state(phase + 6, 0);
    }
  }
  mnaCompApplyRightSideVectorStamp(**mRightVector);
}

void EMT::Ph3::RXLoad::mnaParentPostStep(Real time, Int timeStepCount,
                                         Attribute<Matrix>::Ptr &leftVector) {
  if (!mUseSubcomponents) {
    mnaCompUpdateVoltage(**leftVector);
    const Matrix &voltage = **mIntfVoltage;
    Matrix &state = **mDirectState;
    Matrix &current = **mIntfCurrent;
    for (Int phase = 0; phase < 3; ++phase) {
      Real inductorCurrent = 0.;
      Real capacitorCurrent = 0.;
      if (mDirectHasInductor) {
        inductorCurrent = mDirectInductorAdmittance * voltage(phase, 0) +
                          mDirectHistoryLoad(phase, 0);
        state(phase, 0) = voltage(phase, 0);
        state(phase + 3, 0) = inductorCurrent;
      }
      if (mDirectHasCapacitor) {
        capacitorCurrent = mDirectCapacitorAdmittance * voltage(phase, 0) +
                           mDirectHistoryShunt(phase, 0);
        state(phase + 6, 0) = voltage(phase, 0);
        state(phase + 9, 0) = capacitorCurrent;
      }
      current(phase, 0) = mDirectTotalConductance * voltage(phase, 0) +
                          inductorCurrent + capacitorCurrent;
    }
    return;
  }
  mnaCompUpdateVoltage(**leftVector);
  mnaCompUpdateCurrent(**leftVector);
}

void EMT::Ph3::RXLoad::mnaCompUpdateVoltage(const Matrix &leftVector) {
  (**mIntfVoltage).setZero();
  (**mIntfVoltage)(0, 0) =
      Math::realFromVectorElement(leftVector, matrixNodeIndex(0, 0));
  (**mIntfVoltage)(1, 0) =
      Math::realFromVectorElement(leftVector, matrixNodeIndex(0, 1));
  (**mIntfVoltage)(2, 0) =
      Math::realFromVectorElement(leftVector, matrixNodeIndex(0, 2));
}

void EMT::Ph3::RXLoad::mnaCompUpdateCurrent(const Matrix &leftVector) {
  if (!mUseSubcomponents) {
    const Matrix &voltage = **mIntfVoltage;
    Matrix &current = **mIntfCurrent;
    for (Int phase = 0; phase < 3; ++phase) {
      Real inductorCurrent = 0.;
      Real capacitorCurrent = 0.;
      if (mDirectHasInductor)
        inductorCurrent = mDirectInductorAdmittance * voltage(phase, 0) +
                          mDirectHistoryLoad(phase, 0);
      if (mDirectHasCapacitor)
        capacitorCurrent = mDirectCapacitorAdmittance * voltage(phase, 0) +
                           mDirectHistoryShunt(phase, 0);
      current(phase, 0) = mDirectTotalConductance * voltage(phase, 0) +
                          inductorCurrent + capacitorCurrent;
    }
    return;
  }
  if (mReactanceInSeries) {
    **mIntfCurrent = mSubInductor->intfCurrent();
  } else {
    (**mIntfCurrent).setZero();
    for (auto &subc : mSubComponents) {
      **mIntfCurrent += subc->intfCurrent();
    }
  }
}

void EMT::Ph3::RXLoad::mnaParentInitialize(Real, Real timeStep,
                                           Attribute<Matrix>::Ptr) {
  if (mUseSubcomponents)
    return;
  mDirectInductorAdmittance =
      mDirectHasInductor ? timeStep / (2. * mInductance(0, 0)) : 0.;
  mDirectCapacitorAdmittance =
      mDirectHasCapacitor ? 2. * mCapacitance(0, 0) / timeStep : 0.;
  mDirectNodalAdmittance =
      (mDirectTotalConductance + mDirectInductorAdmittance +
       mDirectCapacitorAdmittance) *
      Matrix::Identity(3, 3);
}

void EMT::Ph3::RXLoad::mnaParentApplySystemMatrixStamp(
    SparseMatrixRow &systemMatrix) {
  if (!mUseSubcomponents) {
    updateMatrixNodeIndices();
    MNAStampUtils::stampConductanceMatrix(mDirectNodalAdmittance, systemMatrix,
                                          matrixNodeIndex(0), 0,
                                          terminalNotGrounded(0), false, mSLog);
  }
}

void EMT::Ph3::RXLoad::mnaParentApplyRightSideVectorStamp(Matrix &rightVector) {
  if (!mUseSubcomponents && terminalNotGrounded(0))
    for (UInt phase = 0; phase < 3; ++phase)
      Math::setVectorElement(
          rightVector, matrixNodeIndex(0, phase),
          -(mDirectHistoryLoad(phase) + mDirectHistoryShunt(phase)));
}

Bool EMT::Ph3::RXLoad::mnaHasCompactRightSideVectorStamp() const {
  return !mUseSubcomponents ||
         CompositePowerComp<Real>::mnaHasCompactRightSideVectorStamp();
}

void EMT::Ph3::RXLoad::mnaAddCompactRightSideVectorStamp(Matrix &rightVector) {
  if (mUseSubcomponents) {
    CompositePowerComp<Real>::mnaAddCompactRightSideVectorStamp(rightVector);
    return;
  }
  if (!terminalNotGrounded(0))
    return;
  for (UInt phase = 0; phase < 3; ++phase) {
    const UInt idx = matrixNodeIndex(0, phase);
    Math::addToVectorElement(rightVector, idx,
                             Math::realFromVectorElement(**mRightVector, idx));
  }
}
