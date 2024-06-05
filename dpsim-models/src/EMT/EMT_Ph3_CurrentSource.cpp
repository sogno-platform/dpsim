/* Copyright 2017-2020 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/EMT/EMT_Ph3_CurrentSource.h>

using namespace CPS;

EMT::Ph3::CurrentSource::CurrentSource(String uid, String name,
                                       Logger::Level logLevel)
    : MNASimPowerComp<Real>(uid, name, true, true, logLevel),
      mCurrentRef(mAttributes->create<MatrixComp>("I_ref")), // rms-value
      mSrcFreq(mAttributes->createDynamic<Real>("f_src")),
      mSigOut(mAttributes->createDynamic<Complex>("sigOut")) {
  mPhaseType = PhaseType::ABC;
  setVirtualNodeNumber(0);
  setTerminalNumber(2);
  **mIntfVoltage = Matrix::Zero(3, 1);
  **mIntfCurrent = Matrix::Zero(3, 1);
}

void EMT::Ph3::CurrentSource::setParameters(MatrixComp currentRef,
                                            Real srcFreq) {
  auto srcSigSine = Signal::SineWaveGenerator::make(**mName + "_sw");
  // Complex(1,0) is used as initialPhasor, since magnitude and phase of I_ref are taken into account by updateCurrent
  srcSigSine->mFreq->setReference(mSrcFreq);
  srcSigSine->setParameters(Complex(1, 0), srcFreq);
  mSrcSig = srcSigSine;

  **mCurrentRef = currentRef;
  mParametersSet = true;
}

void EMT::Ph3::CurrentSource::initializeFromNodesAndTerminals(Real frequency) {
  SPDLOG_LOGGER_INFO(
      mSLog, "\n--- Initialization from node voltages and terminal ---");

  // IntfVoltage initialization for each phase
  MatrixComp vInitABC = Matrix::Zero(3, 1);
  vInitABC(0, 0) = RMS3PH_TO_PEAK1PH * initialSingleVoltage(1) -
                   RMS3PH_TO_PEAK1PH * initialSingleVoltage(0);
  vInitABC(1, 0) = vInitABC(0, 0) * SHIFT_TO_PHASE_B;
  vInitABC(2, 0) = vInitABC(0, 0) * SHIFT_TO_PHASE_C;
  **mIntfVoltage = vInitABC.real();

  if (!mParametersSet) {
    auto srcSigSine =
        Signal::SineWaveGenerator::make(**mName + "_sw", Logger::Level::off);
    // Complex(1,0) is used as initialPhasor for signal generator as only phase is used
    srcSigSine->setParameters(Complex(1, 0), frequency);
    mSrcSig = srcSigSine;

    Complex v_init = initialSingleVoltage(1) - initialSingleVoltage(0);
    //TODO: set initial power using init_from_powerflow!
    Complex s_init = terminal(1)->singlePower() - terminal(0)->singlePower();

    // Current flowing from T1 to T0 (rms value)
    Complex i_ref = std::conj(s_init / v_init / sqrt(3.));

    **mCurrentRef = CPS::Math::singlePhaseVariableToThreePhase(i_ref);
    mSrcFreq->setReference(mSrcSig->attributeTyped<Real>("freq"));

    SPDLOG_LOGGER_INFO(mSLog,
                       "\nReference current: {:s}"
                       "\nInitial voltage: {:s}"
                       "\nInitial 3ph power: {:s}"
                       "\nTerminal 0 voltage: {:s}"
                       "\nTerminal 1 voltage: {:s}"
                       "\nTerminal 0 power: {:s}"
                       "\nTerminal 1 power: {:s}",
                       Logger::phasorToString(i_ref),
                       Logger::phasorToString(v_init),
                       Logger::complexToString(s_init),
                       Logger::phasorToString(initialSingleVoltage(0)),
                       Logger::phasorToString(initialSingleVoltage(1)),
                       Logger::complexToString(terminal(0)->singlePower()),
                       Logger::complexToString(terminal(1)->singlePower()));
  } else {
    **mIntfCurrent = RMS3PH_TO_PEAK1PH * (**mCurrentRef).real();
    SPDLOG_LOGGER_INFO(
        mSLog,
        "\nInitialization from node voltages and terminal omitted (parameter "
        "already set)."
        "\nInitial current: {:s}"
        "\nReference current: {:s}",
        Logger::matrixCompToString(**mIntfCurrent),
        Logger::matrixCompToString(attributeTyped<MatrixComp>("I_ref")->get()));
  }
  SPDLOG_LOGGER_INFO(
      mSLog, "\n--- Initialization from node voltages and terminal ---");
  mSLog->flush();
}

void EMT::Ph3::CurrentSource::mnaCompInitialize(
    Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
  updateMatrixNodeIndices();
}

void EMT::Ph3::CurrentSource::mnaCompApplyRightSideVectorStamp(
    Matrix &rightVector) {
  // current flows out of terminal 0 and flows into terminal 1
  if (terminalNotGrounded(0)) {
    Math::setVectorElement(rightVector, matrixNodeIndex(0, 0),
                           -(**mIntfCurrent)(0, 0));
    Math::setVectorElement(rightVector, matrixNodeIndex(0, 1),
                           -(**mIntfCurrent)(1, 0));
    Math::setVectorElement(rightVector, matrixNodeIndex(0, 2),
                           -(**mIntfCurrent)(2, 0));
  }
  if (terminalNotGrounded(1)) {
    Math::setVectorElement(rightVector, matrixNodeIndex(1, 0),
                           (**mIntfCurrent)(0, 0));
    Math::setVectorElement(rightVector, matrixNodeIndex(1, 1),
                           (**mIntfCurrent)(1, 0));
    Math::setVectorElement(rightVector, matrixNodeIndex(1, 2),
                           (**mIntfCurrent)(2, 0));
  }
}

void EMT::Ph3::CurrentSource::updateCurrent(Real time) {
  if (mSrcSig != nullptr) {
    mSrcSig->step(time);
    for (int i = 0; i < 3; i++) {
      (**mIntfCurrent)(i, 0) = RMS3PH_TO_PEAK1PH *
                               Math::abs((**mCurrentRef)(i, 0)) *
                               cos(Math::phase(mSrcSig->getSignal()) +
                                   Math::phase((**mCurrentRef)(i, 0)));
    }
  } else {
    **mIntfCurrent = RMS3PH_TO_PEAK1PH * (**mCurrentRef).real();
  }
  SPDLOG_LOGGER_DEBUG(mSLog, "\nUpdate current: {:s}",
                      Logger::matrixToString(**mIntfCurrent));
}

void EMT::Ph3::CurrentSource::mnaCompAddPreStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes) {
  attributeDependencies.push_back(mCurrentRef);
  modifiedAttributes.push_back(mRightVector);
  modifiedAttributes.push_back(mIntfCurrent);
}

void EMT::Ph3::CurrentSource::mnaCompPreStep(Real time, Int timeStepCount) {
  updateCurrent(time);
  mnaCompApplyRightSideVectorStamp(**mRightVector);
}

void EMT::Ph3::CurrentSource::mnaCompAddPostStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes,
    Attribute<Matrix>::Ptr &leftVector) {
  attributeDependencies.push_back(leftVector);
  modifiedAttributes.push_back(mIntfVoltage);
};

void EMT::Ph3::CurrentSource::mnaCompPostStep(
    Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
  mnaCompUpdateVoltage(**leftVector);
}

void EMT::Ph3::CurrentSource::mnaCompUpdateVoltage(const Matrix &leftVector) {
  // v1 - v0
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
    (**mIntfVoltage)(0, 0) =
        (**mIntfVoltage)(0, 0) -
        Math::realFromVectorElement(leftVector, matrixNodeIndex(0, 0));
    (**mIntfVoltage)(1, 0) =
        (**mIntfVoltage)(1, 0) -
        Math::realFromVectorElement(leftVector, matrixNodeIndex(0, 1));
    (**mIntfVoltage)(2, 0) =
        (**mIntfVoltage)(2, 0) -
        Math::realFromVectorElement(leftVector, matrixNodeIndex(0, 2));
  }
}
