/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/DP/DP_Ph1_RXLoad.h>
#include <dpsim-models/MNAStampUtils.h>

using namespace CPS;

DP::Ph1::RXLoad::RXLoad(String uid, String name, Logger::Level logLevel)
    : RXLoad(uid, name, true, logLevel) {}

DP::Ph1::RXLoad::RXLoad(String uid, String name, Bool useSubcomponents,
                        Logger::Level logLevel)
    : CompositePowerComp<Complex>(uid, name, true, true, logLevel),
      mUseSubcomponents(useSubcomponents),
      mActivePower(mAttributes->create<Real>("P")),
      mReactivePower(mAttributes->create<Real>("Q")),
      mNomVoltage(mAttributes->create<Real>("V_nom")) {
  setTerminalNumber(1);

  SPDLOG_LOGGER_INFO(mSLog, "Create {} {}", this->type(), name);
  **mIntfVoltage = MatrixComp::Zero(1, 1);
  **mIntfCurrent = MatrixComp::Zero(1, 1);
}

DP::Ph1::RXLoad::RXLoad(String name, Logger::Level logLevel)
    : RXLoad(name, name, logLevel) {}

SimPowerComp<Complex>::Ptr DP::Ph1::RXLoad::clone(String name) {
  auto copy = RXLoad::make(name, mLogLevel);
  if (mParametersSet)
    copy->setParameters(**mActivePower, **mReactivePower, **mNomVoltage);
  return copy;
}

void DP::Ph1::RXLoad::createSubComponents() {
  if (!mUseSubcomponents)
    return;
  if (mSubCompCreated)
    return;
  mSubCompCreated = true;

  // Intentionally empty: which of R/L/C exist depends on the load power sign, known only in
  // initializeParentFromNodesAndTerminals(), where the sub-components are created. Safe: no new virtual nodes.
}

void DP::Ph1::RXLoad::initializeParentFromNodesAndTerminals(Real frequency) {
  if (!mParametersSet) {
    setParameters(mTerminals[0]->singleActivePower(),
                  mTerminals[0]->singleReactivePower(),
                  std::abs(mTerminals[0]->initialSingleVoltage()));
  }

  Real omega = 2. * PI * frequency;

  if (**mActivePower != 0) {
    mResistance = std::pow(**mNomVoltage, 2) / **mActivePower;
    mDirectConductance = 1. / mResistance;
    if (mUseSubcomponents) {
      mSubResistor =
          std::make_shared<DP::Ph1::Resistor>(**mName + "_res", mLogLevel);
      mSubResistor->setParameters(mResistance);
      mSubResistor->connect({SimNode::GND, mTerminals[0]->node()});
      addMNASubComponent(mSubResistor,
                         MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                         MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, false);
    }
  } else {
    mResistance = 0;
    mDirectConductance = 0.;
  }

  mReactance = (**mReactivePower != 0)
                   ? std::pow(**mNomVoltage, 2) / **mReactivePower
                   : 0;

  if (mReactance > 0) {
    mInductance = mReactance / omega;
    mDirectUseInductor = true;
    mDirectUseCapacitor = false;
    if (mUseSubcomponents) {
      mSubInductor =
          std::make_shared<DP::Ph1::Inductor>(**mName + "_ind", mLogLevel);
      mSubInductor->setParameters(mInductance);
      mSubInductor->connect({SimNode::GND, mTerminals[0]->node()});
      addMNASubComponent(mSubInductor,
                         MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                         MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);
    }
  } else if (mReactance < 0) {
    mCapacitance = -1. / omega / mReactance;
    mDirectUseInductor = false;
    mDirectUseCapacitor = true;
    mDirectCapacitance = mCapacitance;
    if (mUseSubcomponents) {
      mSubCapacitor =
          std::make_shared<DP::Ph1::Capacitor>(**mName + "_cap", mLogLevel);
      mSubCapacitor->setParameters(mCapacitance);
      mSubCapacitor->connect({SimNode::GND, mTerminals[0]->node()});
      addMNASubComponent(mSubCapacitor,
                         MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                         MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);
    }
  } else {
    mDirectUseInductor = false;
    mDirectUseCapacitor = false;
  }

  (**mIntfVoltage)(0, 0) = mTerminals[0]->initialSingleVoltage();
  (**mIntfCurrent)(0, 0) =
      (std::abs((**mIntfVoltage)(0, 0)) > 0)
          ? std::conj(Complex(**mActivePower, **mReactivePower) /
                      (**mIntfVoltage)(0, 0))
          : Complex(0, 0);
  if (!mUseSubcomponents && std::abs((**mIntfVoltage)(0, 0)) > 0.) {
    if (mDirectUseInductor)
      mDirectBranchCurrent = MatrixComp::Zero(mNumFreqs, 1);
    else if (mDirectUseCapacitor)
      mDirectBranchCurrent = MatrixComp::Zero(mNumFreqs, 1);
  }

  SPDLOG_LOGGER_INFO(mSLog,
                     "\n--- Initialization from powerflow ---"
                     "\nVoltage across: {:s}"
                     "\nCurrent: {:s}"
                     "\nTerminal 0 voltage: {:s}"
                     "\nResistance: {:f}"
                     "\nReactance: {:f}"
                     "\n--- Initialization from powerflow finished ---",
                     Logger::phasorToString((**mIntfVoltage)(0, 0)),
                     Logger::phasorToString((**mIntfCurrent)(0, 0)),
                     Logger::phasorToString(initialSingleVoltage(0)),
                     mResistance, mReactance);
}

void DP::Ph1::RXLoad::mnaParentInitialize(Real omega, Real timeStep,
                                          Attribute<Matrix>::Ptr leftVector) {
  if (mUseSubcomponents)
    return;
  mDirectEquivCond = MatrixComp::Zero(mNumFreqs, 1);
  mDirectHistory = MatrixComp::Zero(mNumFreqs, 1);
  mDirectPrevCurrentFactor = MatrixComp::Zero(mNumFreqs, 1);
  mDirectPrevVoltageFactor = MatrixComp::Zero(mNumFreqs, 1);
  mDirectBranchCurrent = MatrixComp::Zero(mNumFreqs, 1);
  if (!mDirectUseInductor && !mDirectUseCapacitor)
    return;

  for (UInt freq = 0; freq < mNumFreqs; ++freq) {
    const Real harmonicOmega = 2. * PI * mFrequencies(freq, 0);
    if (mDirectUseInductor) {
      const Real b = timeStep * harmonicOmega / 2.;
      const Real a = timeStep / (2. * mInductance);
      mDirectEquivCond(freq, 0) =
          Complex(a / (1. + b * b), -a * b / (1. + b * b));
      mDirectPrevCurrentFactor(freq, 0) =
          Complex((1. - b * b) / (1. + b * b), -2. * b / (1. + b * b));
      if (freq == 0 && harmonicOmega != 0.)
        mDirectBranchCurrent(freq, 0) =
            (**mIntfVoltage)(0, 0) / Complex(0., harmonicOmega * mInductance);
    } else {
      const Real capConductance = 2. * mDirectCapacitance / timeStep;
      const Real capSusceptance = harmonicOmega * mDirectCapacitance;
      mDirectEquivCond(freq, 0) = Complex(capConductance, capSusceptance);
      mDirectPrevVoltageFactor(freq, 0) =
          Complex(capConductance, -capSusceptance);
      if (freq == 0)
        mDirectBranchCurrent(freq, 0) =
            Complex(0., harmonicOmega * mDirectCapacitance) *
            (**mIntfVoltage)(0, 0);
    }
  }
}

void DP::Ph1::RXLoad::mnaParentApplySystemMatrixStamp(
    SparseMatrixRow &systemMatrix) {
  if (mUseSubcomponents)
    return;
  for (UInt freq = 0; freq < mNumFreqs; ++freq)
    MNAStampUtils::stampAdmittance(
        Complex(mDirectConductance, 0.) + mDirectEquivCond(freq, 0),
        systemMatrix, matrixNodeIndex(0), 0, terminalNotGrounded(0), false,
        mSLog, mNumFreqs, freq);
}

void DP::Ph1::RXLoad::mnaParentApplyRightSideVectorStamp(Matrix &rightVector) {
  if (mUseSubcomponents || (!mDirectUseInductor && !mDirectUseCapacitor))
    return;
  for (UInt freq = 0; freq < mNumFreqs; ++freq) {
    if (mDirectUseInductor)
      mDirectHistory(freq, 0) =
          mDirectEquivCond(freq, 0) * (**mIntfVoltage)(0, freq) +
          mDirectPrevCurrentFactor(freq, 0) * mDirectBranchCurrent(freq, 0);
    else
      mDirectHistory(freq, 0) =
          -mDirectBranchCurrent(freq, 0) -
          mDirectPrevVoltageFactor(freq, 0) * (**mIntfVoltage)(0, freq);
    if (terminalNotGrounded(0))
      Math::setVectorElement(rightVector, matrixNodeIndex(0),
                             -mDirectHistory(freq, 0), mNumFreqs, freq);
  }
}

void DP::Ph1::RXLoad::setParameters(Real activePower, Real reactivePower,
                                    Real volt) {
  mParametersSet = true;
  **mActivePower = activePower;
  **mReactivePower = reactivePower;
  **mNomVoltage = volt;

  SPDLOG_LOGGER_INFO(mSLog, "Active Power={} [W] Reactive Power={} [VAr]",
                     **mActivePower, **mReactivePower);
  SPDLOG_LOGGER_INFO(mSLog, "Nominal Voltage={} [V]", **mNomVoltage);
}

void DP::Ph1::RXLoad::mnaCompUpdateVoltage(const Matrix &leftVector) {
  for (UInt freq = 0; freq < mNumFreqs; ++freq)
    (**mIntfVoltage)(0, freq) = Math::complexFromVectorElement(
        leftVector, matrixNodeIndex(0), mNumFreqs, freq);
}

void DP::Ph1::RXLoad::mnaCompUpdateCurrent(const Matrix &leftVector) {
  if (!mUseSubcomponents) {
    for (UInt freq = 0; freq < mNumFreqs; ++freq) {
      const Complex voltage = (**mIntfVoltage)(0, freq);
      const Complex branchCurrent =
          mDirectEquivCond(freq, 0) * voltage + mDirectHistory(freq, 0);
      if (mDirectUseInductor || mDirectUseCapacitor)
        mDirectBranchCurrent(freq, 0) = branchCurrent;
      (**mIntfCurrent)(0, freq) =
          Complex(mDirectConductance, 0.) * voltage +
          ((mDirectUseInductor || mDirectUseCapacitor) ? branchCurrent
                                                       : Complex(0., 0.));
    }
    return;
  }
  (**mIntfCurrent)(0, 0) = 0;

  for (auto subComp : mSubComponents) {
    (**mIntfCurrent)(0, 0) += subComp->intfCurrent()(0, 0);
  }
}

void DP::Ph1::RXLoad::mnaParentAddPreStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes) {
  if (!mUseSubcomponents) {
    prevStepDependencies.push_back(mIntfVoltage);
    prevStepDependencies.push_back(mIntfCurrent);
  }
  modifiedAttributes.push_back(mRightVector);
}

void DP::Ph1::RXLoad::mnaParentPreStep(Real time, Int timeStepCount) {
  mnaCompApplyRightSideVectorStamp(**mRightVector);
}

void DP::Ph1::RXLoad::mnaParentAddPostStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes,
    Attribute<Matrix>::Ptr &leftVector) {
  attributeDependencies.push_back(leftVector);
  modifiedAttributes.push_back(mIntfVoltage);
  modifiedAttributes.push_back(mIntfCurrent);
}

void DP::Ph1::RXLoad::mnaParentPostStep(Real time, Int timeStepCount,
                                        Attribute<Matrix>::Ptr &leftVector) {
  mnaCompUpdateVoltage(**leftVector);
  mnaCompUpdateCurrent(**leftVector);
}
