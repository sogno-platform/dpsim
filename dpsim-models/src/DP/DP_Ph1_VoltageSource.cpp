/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/DP/DP_Ph1_VoltageSource.h>

using namespace CPS;

DP::Ph1::VoltageSource::VoltageSource(String uid, String name,
                                      Logger::Level logLevel)
    : MNASimPowerComp<Complex>(uid, name, true, true, logLevel),
      mVoltageRef(mAttributes->createDynamic<Complex>("V_ref")),
      mSrcFreq(mAttributes->createDynamic<Real>("f_src")) {
  setVirtualNodeNumber(1);
  setTerminalNumber(2);
  **mIntfVoltage = MatrixComp::Zero(1, 1);
  **mIntfCurrent = MatrixComp::Zero(1, 1);
}

SimPowerComp<Complex>::Ptr DP::Ph1::VoltageSource::clone(String name) {
  auto copy = VoltageSource::make(name, mLogLevel);
  if (mSrcSig == nullptr)
    copy->setParameters(**mVoltageRef);
  else
    copy->setParameters(mSrcSig->getSignal());
  return copy;
}

void DP::Ph1::VoltageSource::setParameters(Complex voltageRef, Real srcFreq) {
  if (srcFreq == 0.0) {
    auto srcDC = Signal::DCGenerator::make(**mName + "_dc");
    srcDC->mVoltageRef->setReference(mVoltageRef);
    srcDC->setParameters(voltageRef.real());

    mSrcSig = srcDC;
  } else {
    auto srcSigSine = Signal::SineWaveGenerator::make(**mName + "_sw");
    srcSigSine->mVoltageRef->setReference(mVoltageRef);
    srcSigSine->mFreq->setReference(mSrcFreq);
    srcSigSine->setParameters(voltageRef, srcFreq);

    mSrcSig = srcSigSine;
  }
  mParametersSet = true;
}

void DP::Ph1::VoltageSource::setParameters(Complex initialPhasor,
                                           Real freqStart, Real rocof,
                                           Real timeStart, Real duration,
                                           bool smoothRamp) {
  auto srcSigFreqRamp = Signal::FrequencyRampGenerator::make(**mName + "_fr");
  srcSigFreqRamp->mFreq->setReference(mSrcFreq);
  srcSigFreqRamp->setParameters(initialPhasor, freqStart, rocof, timeStart,
                                duration, smoothRamp);
  mSrcSig = srcSigFreqRamp;

  mParametersSet = true;
}

void DP::Ph1::VoltageSource::setParameters(Complex initialPhasor,
                                           Real modulationFrequency,
                                           Real modulationAmplitude,
                                           Real baseFrequency /*= 0.0*/,
                                           bool zigzag /*= false*/) {
  auto srcSigFm = Signal::CosineFMGenerator::make(**mName + "_fm");
  srcSigFm->mFreq->setReference(mSrcFreq);
  srcSigFm->setParameters(initialPhasor, modulationFrequency,
                          modulationAmplitude, baseFrequency, zigzag);
  mSrcSig = srcSigFm;

  mParametersSet = true;
}

void DP::Ph1::VoltageSource::initializeFromNodesAndTerminals(Real frequency) {
  ///CHECK: The frequency parameter is unused
  if (**mVoltageRef == Complex(0, 0))
    **mVoltageRef = initialSingleVoltage(1) - initialSingleVoltage(0);

  if (mSrcSig == nullptr) {
    auto srcSigSine = Signal::SineWaveGenerator::make(**mName);
    srcSigSine->mVoltageRef->setReference(mVoltageRef);
    srcSigSine->mFreq->setReference(mSrcFreq);
    srcSigSine->setParameters(**mVoltageRef);
    mSrcSig = srcSigSine;
  }

  SPDLOG_LOGGER_INFO(mSLog,
                     "\n--- Initialization from node voltages ---"
                     "\nVoltage across: {:s}"
                     "\nTerminal 0 voltage: {:s}"
                     "\nTerminal 1 voltage: {:s}"
                     "\n--- Initialization from node voltages ---",
                     Logger::phasorToString(mSrcSig->getSignal()),
                     Logger::phasorToString(initialSingleVoltage(0)),
                     Logger::phasorToString(initialSingleVoltage(1)));
}

// #### MNA functions ####

void DP::Ph1::VoltageSource::mnaCompAddPreStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes) {
  attributeDependencies.push_back(mVoltageRef);
  modifiedAttributes.push_back(mRightVector);
  modifiedAttributes.push_back(mIntfVoltage);
}

void DP::Ph1::VoltageSource::mnaCompAddPostStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes,
    Attribute<Matrix>::Ptr &leftVector) {
  attributeDependencies.push_back(leftVector);
  modifiedAttributes.push_back(mIntfCurrent);
};

void DP::Ph1::VoltageSource::mnaCompInitialize(
    Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
  updateMatrixNodeIndices();

  (**mIntfVoltage)(0, 0) = mSrcSig->getSignal();

  SPDLOG_LOGGER_INFO(mSLog,
                     "\n--- MNA initialization ---"
                     "\nInitial voltage {:s}"
                     "\nInitial current {:s}"
                     "\n--- MNA initialization finished ---",
                     Logger::phasorToString((**mIntfVoltage)(0, 0)),
                     Logger::phasorToString((**mIntfCurrent)(0, 0)));
}

void DP::Ph1::VoltageSource::mnaCompInitializeHarm(
    Real omega, Real timeStep,
    std::vector<Attribute<Matrix>::Ptr> leftVectors) {
  updateMatrixNodeIndices();

  (**mIntfVoltage)(0, 0) = mSrcSig->getSignal();

  mMnaTasks.push_back(std::make_shared<MnaPreStepHarm>(*this));
  mMnaTasks.push_back(std::make_shared<MnaPostStepHarm>(*this, leftVectors));
  **mRightVector = Matrix::Zero(leftVectors[0]->get().rows(), mNumFreqs);
}

void DP::Ph1::VoltageSource::mnaCompApplySystemMatrixStamp(
    SparseMatrixRow &systemMatrix) {
  for (UInt freq = 0; freq < mNumFreqs; freq++) {
    if (terminalNotGrounded(0)) {
      Math::setMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(),
                             matrixNodeIndex(0), Complex(-1, 0), mNumFreqs,
                             freq);
      Math::setMatrixElement(systemMatrix, matrixNodeIndex(0),
                             mVirtualNodes[0]->matrixNodeIndex(),
                             Complex(-1, 0), mNumFreqs, freq);
    }
    if (terminalNotGrounded(1)) {
      Math::setMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(),
                             matrixNodeIndex(1), Complex(1, 0), mNumFreqs,
                             freq);
      Math::setMatrixElement(systemMatrix, matrixNodeIndex(1),
                             mVirtualNodes[0]->matrixNodeIndex(), Complex(1, 0),
                             mNumFreqs, freq);
    }

    SPDLOG_LOGGER_INFO(mSLog, "-- Stamp frequency {:d} ---", freq);
    if (terminalNotGrounded(0)) {
      SPDLOG_LOGGER_INFO(mSLog, "Add {:f} to system at ({:d},{:d})", -1.,
                         matrixNodeIndex(0),
                         mVirtualNodes[0]->matrixNodeIndex());
      SPDLOG_LOGGER_INFO(mSLog, "Add {:f} to system at ({:d},{:d})", -1.,
                         mVirtualNodes[0]->matrixNodeIndex(),
                         matrixNodeIndex(0));
    }
    if (terminalNotGrounded(1)) {
      SPDLOG_LOGGER_INFO(mSLog, "Add {:f} to system at ({:d},{:d})", 1.,
                         mVirtualNodes[0]->matrixNodeIndex(),
                         matrixNodeIndex(1));
      SPDLOG_LOGGER_INFO(mSLog, "Add {:f} to system at ({:d},{:d})", 1.,
                         matrixNodeIndex(1),
                         mVirtualNodes[0]->matrixNodeIndex());
    }
  }
}

void DP::Ph1::VoltageSource::mnaCompApplySystemMatrixStampHarm(
    SparseMatrixRow &systemMatrix, Int freqIdx) {
  if (terminalNotGrounded(0)) {
    Math::setMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(),
                           matrixNodeIndex(0), Complex(-1, 0));
    Math::setMatrixElement(systemMatrix, matrixNodeIndex(0),
                           mVirtualNodes[0]->matrixNodeIndex(), Complex(-1, 0));
  }
  if (terminalNotGrounded(1)) {
    Math::setMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(),
                           matrixNodeIndex(1), Complex(1, 0));
    Math::setMatrixElement(systemMatrix, matrixNodeIndex(1),
                           mVirtualNodes[0]->matrixNodeIndex(), Complex(1, 0));
  }

  SPDLOG_LOGGER_INFO(mSLog, "-- Stamp frequency {:d} ---", freqIdx);
  if (terminalNotGrounded(0)) {
    SPDLOG_LOGGER_INFO(mSLog, "Add {:f} to system at ({:d},{:d})", -1.,
                       matrixNodeIndex(0), mVirtualNodes[0]->matrixNodeIndex());
    SPDLOG_LOGGER_INFO(mSLog, "Add {:f} to system at ({:d},{:d})", -1.,
                       mVirtualNodes[0]->matrixNodeIndex(), matrixNodeIndex(0));
  }
  if (terminalNotGrounded(1)) {
    SPDLOG_LOGGER_INFO(mSLog, "Add {:f} to system at ({:d},{:d})", 1.,
                       mVirtualNodes[0]->matrixNodeIndex(), matrixNodeIndex(1));
    SPDLOG_LOGGER_INFO(mSLog, "Add {:f} to system at ({:d},{:d})", 1.,
                       matrixNodeIndex(1), mVirtualNodes[0]->matrixNodeIndex());
  }
}

void DP::Ph1::VoltageSource::mnaCompApplyRightSideVectorStamp(
    Matrix &rightVector) {
  // TODO: Is this correct with two nodes not gnd?
  Math::setVectorElement(rightVector, mVirtualNodes[0]->matrixNodeIndex(),
                         (**mIntfVoltage)(0, 0), mNumFreqs);
  SPDLOG_LOGGER_DEBUG(mSLog, "Add {:s} to source vector at {:d}",
                      Logger::complexToString((**mIntfVoltage)(0, 0)),
                      mVirtualNodes[0]->matrixNodeIndex());
}

void DP::Ph1::VoltageSource::mnaCompApplyRightSideVectorStampHarm(
    Matrix &rightVector) {
  for (UInt freq = 0; freq < mNumFreqs; freq++) {
    // TODO: Is this correct with two nodes not gnd?
    Math::setVectorElement(rightVector, mVirtualNodes[0]->matrixNodeIndex(),
                           (**mIntfVoltage)(0, freq), 1, 0, freq);
    SPDLOG_LOGGER_DEBUG(mSLog, "Add {:s} to source vector at {:d}",
                        Logger::complexToString((**mIntfVoltage)(0, freq)),
                        mVirtualNodes[0]->matrixNodeIndex());
  }
}

void DP::Ph1::VoltageSource::updateVoltage(Real time) {
  if (mSrcSig != nullptr) {
    mSrcSig->step(time);
    (**mIntfVoltage)(0, 0) = mSrcSig->getSignal();
  } else {
    throw SystemError("VoltageSource::updateVoltage was called but no signal "
                      "generator is configured!");
  }

  SPDLOG_LOGGER_DEBUG(mSLog, "Update Voltage {:s}",
                      Logger::phasorToString((**mIntfVoltage)(0, 0)));
}

void DP::Ph1::VoltageSource::mnaCompPreStep(Real time, Int timeStepCount) {
  updateVoltage(time);
  mnaCompApplyRightSideVectorStamp(**mRightVector);
}

void DP::Ph1::VoltageSource::MnaPreStepHarm::execute(Real time,
                                                     Int timeStepCount) {
  mVoltageSource.updateVoltage(time);
  mVoltageSource.mnaCompApplyRightSideVectorStampHarm(
      **mVoltageSource.mRightVector);
}

void DP::Ph1::VoltageSource::mnaCompPostStep(
    Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
  mnaCompUpdateCurrent(**leftVector);
}

void DP::Ph1::VoltageSource::MnaPostStepHarm::execute(Real time,
                                                      Int timeStepCount) {
  mVoltageSource.mnaCompUpdateCurrent(**mLeftVectors[0]);
}

void DP::Ph1::VoltageSource::mnaCompUpdateCurrent(const Matrix &leftVector) {
  for (UInt freq = 0; freq < mNumFreqs; freq++) {
    (**mIntfCurrent)(0, freq) = Math::complexFromVectorElement(
        leftVector, mVirtualNodes[0]->matrixNodeIndex(), mNumFreqs, freq);
  }
}

void DP::Ph1::VoltageSource::daeResidual(double ttime, const double state[],
                                         const double dstate_dt[],
                                         double resid[],
                                         std::vector<int> &off) {
  /* new state vector definintion:
		state[0]=node0_voltage
		state[1]=node1_voltage
		....
		state[n]=noden_voltage
		state[n+1]=component0_voltage
		state[n+2]=component0_inductance (not yet implemented)
		...
		state[m-1]=componentm_voltage
		state[m]=componentm_inductance
	*/

  int Pos1 = matrixNodeIndex(0);
  int Pos2 = matrixNodeIndex(1);
  int c_offset = off[0] + off[1]; //current offset for component
  int n_offset_1 =
      c_offset + Pos1 + 1; // current offset for first nodal equation
  int n_offset_2 =
      c_offset + Pos2 + 1; // current offset for second nodal equation
  resid[c_offset] = (state[Pos2] - state[Pos1]) -
                    state[c_offset]; // Voltage equation for Resistor
  //resid[++c_offset] = ; //TODO : add inductance equation
  resid[n_offset_1] += (**mIntfCurrent)(0, 0).real();
  resid[n_offset_2] += (**mIntfCurrent)(0, 0).real();
  off[1] += 1;
}

Complex DP::Ph1::VoltageSource::daeInitialize() {
  (**mIntfVoltage)(0, 0) = mSrcSig->getSignal();
  return mSrcSig->getSignal();
}
