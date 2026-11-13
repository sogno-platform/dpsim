/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/DP/DP_Ph1_RXLoadSwitch.h>
#include <dpsim-models/MNAStampUtils.h>

using namespace CPS;

DP::Ph1::RXLoadSwitch::RXLoadSwitch(String uid, String name,
                                    Logger::Level logLevel)
    : RXLoadSwitch(uid, name, true, logLevel) {}

DP::Ph1::RXLoadSwitch::RXLoadSwitch(String uid, String name,
                                    Bool useSubcomponents,
                                    Logger::Level logLevel)
    : CompositePowerComp<Complex>(uid, name, true, true, logLevel),
      mUseSubcomponents(useSubcomponents) {
  setTerminalNumber(1);
  setVirtualNodeNumber(1);
  **mIntfVoltage = MatrixComp::Zero(1, 1);
  **mIntfCurrent = MatrixComp::Zero(1, 1);

  if (mUseSubcomponents) {
    mSubRXLoad =
        std::make_shared<DP::Ph1::RXLoad>(**mName + "_rxload", mLogLevel);
    mSubSwitch =
        std::make_shared<DP::Ph1::Switch>(**mName + "_switch", mLogLevel);
    addMNASubComponent(mSubRXLoad, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                       MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);
    addMNASubComponent(mSubSwitch, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                       MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, false);
    mSubSwitch->setParameters(mDirectOpenResistance, mDirectClosedResistance,
                              true);
  }
}

DP::Ph1::RXLoadSwitch::RXLoadSwitch(String name, Logger::Level logLevel)
    : RXLoadSwitch(name, name, logLevel) {}

Bool DP::Ph1::RXLoadSwitch::mnaIsClosed() {
  return mUseSubcomponents ? mSubSwitch->isClosed() : mDirectSwitchClosed;
}

void DP::Ph1::RXLoadSwitch::initializeParentFromNodesAndTerminals(
    Real frequency) {
  if (!mUseSubcomponents) {
    if (!mDirectParametersSet) {
      mDirectActivePower = mTerminals[0]->singleActivePower();
      mDirectReactivePower = mTerminals[0]->singleReactivePower();
      mDirectNomVoltage = std::abs(mTerminals[0]->initialSingleVoltage());
    }
    const Real omega = 2. * PI * frequency;
    mDirectConductance =
        mDirectActivePower != 0.
            ? mDirectActivePower / (mDirectNomVoltage * mDirectNomVoltage)
            : 0.;
    const Real reactance =
        mDirectReactivePower != 0.
            ? mDirectNomVoltage * mDirectNomVoltage / mDirectReactivePower
            : 0.;
    mDirectUseInductor = reactance > 0.;
    mDirectUseCapacitor = reactance < 0.;
    if (mDirectUseInductor)
      mDirectInductance = reactance / omega;
    else if (mDirectUseCapacitor)
      mDirectCapacitance = -1. / (omega * reactance);
    const Complex voltage = initialSingleVoltage(0);
    const Complex current =
        std::abs(voltage) > 0.
            ? std::conj(Complex(mDirectActivePower, mDirectReactivePower) /
                        voltage)
            : Complex(0., 0.);
    const Real switchResistance =
        mDirectSwitchClosed ? mDirectClosedResistance : mDirectOpenResistance;
    mVirtualNodes[0]->setInitialVoltage(voltage - switchResistance * current);
    (**mIntfVoltage)(0, 0) = voltage;
    (**mIntfCurrent)(0, 0) = current;
    return;
  }

  if (!mParametersSet) {
    // use powerflow results
    mSubRXLoad->setParameters(mTerminals[0]->singleActivePower(),
                              mTerminals[0]->singleReactivePower(),
                              std::abs(mTerminals[0]->initialSingleVoltage()));
  }

  // The nested RXLoad reads its connected node's initial voltage during its
  // own initialization. Seed the internal node from the terminal phasor and
  // the closed/open switch drop before initializing that child.
  const Complex terminalVoltage = initialSingleVoltage(0);
  const Complex loadPower(**mSubRXLoad->mActivePower,
                          **mSubRXLoad->mReactivePower);
  const Complex initialCurrent = std::abs(terminalVoltage) > 0.
                                     ? std::conj(loadPower / terminalVoltage)
                                     : Complex(0., 0.);
  const Real initialSwitchResistance =
      mSubSwitch->isClosed() ? mDirectClosedResistance : mDirectOpenResistance;
  mVirtualNodes[0]->setInitialVoltage(terminalVoltage -
                                      initialSwitchResistance * initialCurrent);

  mSubRXLoad->connect({virtualNode(0)});
  mSubSwitch->connect({node(0), virtualNode(0)});
  (**mIntfVoltage)(0, 0) = virtualNode(0)->initialSingleVoltage();
  (**mIntfCurrent)(0, 0) = initialCurrent;

  SPDLOG_LOGGER_INFO(mSLog,
                     "\n--- Initialization from powerflow ---"
                     "\nVoltage across: {:s}"
                     "\nCurrent: {:s}"
                     "\nTerminal 0 voltage: {:s}"
                     "\n--- Initialization from powerflow finished ---",
                     Logger::phasorToString((**mIntfVoltage)(0, 0)),
                     Logger::phasorToString((**mIntfCurrent)(0, 0)),
                     Logger::phasorToString(initialSingleVoltage(0)));
}

void DP::Ph1::RXLoadSwitch::setParameters(Real activePower, Real reactivePower,
                                          Real nomVolt, Real openResistance,
                                          Real closedResistance, Bool closed) {
  mParametersSet = true;
  if (!mUseSubcomponents) {
    mDirectSwitchChanged |= (mDirectSwitchClosed != closed) ||
                            (mDirectOpenResistance != openResistance) ||
                            (mDirectClosedResistance != closedResistance);
    mDirectParametersSet = true;
    mDirectActivePower = activePower;
    mDirectReactivePower = reactivePower;
    mDirectNomVoltage = nomVolt;
    mDirectOpenResistance = openResistance;
    mDirectClosedResistance = closedResistance;
    mDirectSwitchClosed = closed;
    return;
  }
  mSubRXLoad->setParameters(activePower, reactivePower, nomVolt);
  mSubSwitch->setParameters(openResistance, closedResistance, closed);
}

void DP::Ph1::RXLoadSwitch::setSwitchParameters(Real openResistance,
                                                Real closedResistance,
                                                Bool closed) {
  if (!mUseSubcomponents) {
    mDirectSwitchChanged |= (mDirectSwitchClosed != closed) ||
                            (mDirectOpenResistance != openResistance) ||
                            (mDirectClosedResistance != closedResistance);
    mDirectOpenResistance = openResistance;
    mDirectClosedResistance = closedResistance;
    mDirectSwitchClosed = closed;
    return;
  }
  mSubSwitch->setParameters(openResistance, closedResistance, closed);
}

void DP::Ph1::RXLoadSwitch::mnaParentInitialize(
    Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
  if (mUseSubcomponents)
    return;
  mDirectEquivalentConductance = MatrixComp::Zero(mNumFreqs, 1);
  mDirectHistory = MatrixComp::Zero(mNumFreqs, 1);
  mDirectPreviousCurrentFactor = MatrixComp::Zero(mNumFreqs, 1);
  mDirectPreviousVoltageFactor = MatrixComp::Zero(mNumFreqs, 1);
  mDirectBranchCurrent = MatrixComp::Zero(mNumFreqs, 1);
  mDirectBranchVoltage = MatrixComp::Zero(mNumFreqs, 1);
  const Real switchResistance =
      mDirectSwitchClosed ? mDirectClosedResistance : mDirectOpenResistance;
  for (UInt freq = 0; freq < mNumFreqs; ++freq) {
    mDirectBranchVoltage(freq, 0) =
        (**mIntfVoltage)(0, freq) -
        switchResistance * (**mIntfCurrent)(0, freq);
    if (mDirectUseInductor || mDirectUseCapacitor)
      mDirectBranchCurrent(freq, 0) =
          (**mIntfCurrent)(0, freq) -
          Complex(mDirectConductance, 0.) * mDirectBranchVoltage(freq, 0);
    const Real harmonicOmega = 2. * PI * mFrequencies(freq, 0);
    if (mDirectUseInductor) {
      const Real b = timeStep * harmonicOmega / 2.;
      const Real a = timeStep / (2. * mDirectInductance);
      mDirectEquivalentConductance(freq, 0) =
          Complex(a / (1. + b * b), -a * b / (1. + b * b));
      mDirectPreviousCurrentFactor(freq, 0) =
          Complex((1. - b * b) / (1. + b * b), -2. * b / (1. + b * b));
    } else if (mDirectUseCapacitor) {
      const Real capG = 2. * mDirectCapacitance / timeStep;
      const Real capB = harmonicOmega * mDirectCapacitance;
      mDirectEquivalentConductance(freq, 0) = Complex(capG, capB);
      mDirectPreviousVoltageFactor(freq, 0) = Complex(capG, -capB);
    }
  }
  mDirectPreviousSwitchResistance = switchResistance;
}

void DP::Ph1::RXLoadSwitch::mnaParentApplyRightSideVectorStamp(
    Matrix &rightVector) {
  if (mUseSubcomponents || (!mDirectUseInductor && !mDirectUseCapacitor))
    return;
  for (UInt freq = 0; freq < mNumFreqs; ++freq) {
    const Complex branchVoltage =
        (**mIntfVoltage)(0, freq) -
        mDirectPreviousSwitchResistance * (**mIntfCurrent)(0, freq);
    if (mDirectUseInductor)
      mDirectHistory(freq, 0) =
          mDirectEquivalentConductance(freq, 0) * branchVoltage +
          mDirectPreviousCurrentFactor(freq, 0) * mDirectBranchCurrent(freq, 0);
    else
      mDirectHistory(freq, 0) =
          -mDirectBranchCurrent(freq, 0) -
          mDirectPreviousVoltageFactor(freq, 0) * branchVoltage;
    Math::setVectorElement(rightVector, mVirtualNodes[0]->matrixNodeIndex(),
                           -mDirectHistory(freq, 0), mNumFreqs, freq);
  }
}

void DP::Ph1::RXLoadSwitch::mnaCompApplySwitchSystemMatrixStamp(
    Bool closed, SparseMatrixRow &systemMatrix, Int freqIdx) {
  if (!mUseSubcomponents) {
    MNAStampUtils::stampAdmittance(Complex(mDirectConductance, 0.) +
                                       mDirectEquivalentConductance(freqIdx, 0),
                                   systemMatrix,
                                   virtualNode(0)->matrixNodeIndex(), 0, true,
                                   false, mSLog, mNumFreqs, freqIdx);
    const Real resistance =
        closed ? mDirectClosedResistance : mDirectOpenResistance;
    MNAStampUtils::stampAdmittance(
        Complex(1. / resistance, 0.), systemMatrix, matrixNodeIndex(0),
        virtualNode(0)->matrixNodeIndex(), terminalNotGrounded(0), true, mSLog,
        mNumFreqs, freqIdx);
    return;
  }
  mSubRXLoad->mnaApplySystemMatrixStamp(systemMatrix);
  mSubSwitch->mnaCompApplySwitchSystemMatrixStamp(closed, systemMatrix,
                                                  freqIdx);
}

void DP::Ph1::RXLoadSwitch::mnaParentAddPreStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes) {
  prevStepDependencies.push_back(mIntfCurrent);
  prevStepDependencies.push_back(mIntfVoltage);
  modifiedAttributes.push_back(mRightVector);
}

void DP::Ph1::RXLoadSwitch::mnaParentPreStep(Real time, Int timeStepCount) {
  updateSwitchState(time);
  mnaCompApplyRightSideVectorStamp(**mRightVector);
}

void DP::Ph1::RXLoadSwitch::mnaParentAddPostStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes,
    Attribute<Matrix>::Ptr &leftVector) {
  attributeDependencies.push_back(leftVector);
  modifiedAttributes.push_back(mIntfVoltage);
  modifiedAttributes.push_back(mIntfCurrent);
}

void DP::Ph1::RXLoadSwitch::mnaParentPostStep(
    Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
  if (!mUseSubcomponents) {
    for (UInt freq = 0; freq < mNumFreqs; ++freq) {
      (**mIntfVoltage)(0, freq) = Math::complexFromVectorElement(
          **leftVector, matrixNodeIndex(0), mNumFreqs, freq);
      const Complex branchVoltage = Math::complexFromVectorElement(
          **leftVector, virtualNode(0)->matrixNodeIndex(), mNumFreqs, freq);
      const Complex branchCurrent =
          mDirectEquivalentConductance(freq, 0) * branchVoltage +
          mDirectHistory(freq, 0);
      if (mDirectUseInductor || mDirectUseCapacitor)
        mDirectBranchCurrent(freq, 0) = branchCurrent;
      (**mIntfCurrent)(0, freq) =
          Complex(mDirectConductance, 0.) * branchVoltage + branchCurrent;
      mDirectBranchVoltage(freq, 0) = branchVoltage;
    }
    mDirectPreviousSwitchResistance =
        mDirectSwitchClosed ? mDirectClosedResistance : mDirectOpenResistance;
    return;
  }
  **mIntfVoltage = **mSubRXLoad->mIntfVoltage;
  **mIntfCurrent = **mSubRXLoad->mIntfCurrent;
}

void DP::Ph1::RXLoadSwitch::updateSwitchState(Real time) {

  if (time > mSwitchTimeOffset && mnaIsClosed()) {
    Real VRef = mUseSubcomponents ? Math::abs(**mSubRXLoad->mNomVoltage)
                                  : mDirectNomVoltage;
    Real V = Math::abs((**mIntfVoltage)(0, 0));

    Real deltaV = Math::abs((V - VRef) / VRef);

    if (deltaV > 0.1) {
      if (mUseSubcomponents)
        mSubSwitch->open();
      else {
        mDirectSwitchClosed = false;
        mDirectSwitchChanged = true;
      }
      SPDLOG_LOGGER_INFO(mSLog, "Opened Switch at {}", (float)time);
    }
  }
}

Bool DP::Ph1::RXLoadSwitch::hasParameterChanged() {
  if (!mUseSubcomponents) {
    const Bool changed = mDirectSwitchChanged;
    mDirectSwitchChanged = false;
    return changed;
  }
  return mSubSwitch->hasParameterChanged();
};
