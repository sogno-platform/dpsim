/* Voltage Source that reads references from a file and replays them.
 *
 * Author: Niklas Eiling <niklas.eiling@eonerc.rwth-aachen.de>
 * SPDX-FileCopyrightText: 2024 Niklas Eiling <niklas.eiling@eonerc.rwth-aachen.de>
 * SPDX-License-Identifier: Apache-2.0
 */

#include <dpsim-models/DP/DP_Ph1_ProfileVoltageSource.h>

#include <fstream>
#include <iostream>
#include <memory>
#include <villas/format.hpp>
#include <villas/formats/protobuf.hpp>
#include <villas/signal_list.hpp>
#include <villas/signal_type.hpp>

using namespace CPS;

DP::Ph1::ProfileVoltageSource::ProfileVoltageSource(String uid, String name, std::filesystem::path sourceFile, Logger::Level logLevel)
    : MNASimPowerComp<Complex>(uid, name, true, true, logLevel), mVoltage(mAttributes->createDynamic<Complex>("V")), mSourceFile(sourceFile), mSourceIndex(0), mSamples() {
  setVirtualNodeNumber(1);
  setTerminalNumber(2);
  **mIntfVoltage = MatrixComp::Zero(1, 1);
  **mIntfCurrent = MatrixComp::Zero(1, 1);

  readFromFile();
}

void DP::Ph1::ProfileVoltageSource::readFromFile() {
  mSamples.clear();
  std::ifstream file(mSourceFile, std::ios::binary | std::ios::ate);

  if (!file.is_open()) {
    throw SystemError("ProfileVoltageSource::ProfileVoltageSource could not open file " + mSourceFile.string());
  }

  std::streamsize size = file.tellg();
  file.seekg(0, std::ios::beg);

  size_t sample_num = size / (sizeof(struct villas::node::Sample) + SAMPLE_DATA_LENGTH(1));
  std::vector<char> buffer(size);
  if (!file.read(buffer.data(), size)) {
    throw SystemError("ProfileVoltageSource::ProfileVoltageSource could not read file " + mSourceFile.string());
  }

  unsigned i, j;
  Villas__Node__Message *pb_msg;

  pb_msg = villas__node__message__unpack(nullptr, size, (uint8_t *)buffer.data());
  if (!pb_msg)
    throw SystemError("ProfileVoltageSource::ProfileVoltageSource could not unpack Protobuf message");

  for (i = 0; i < pb_msg->n_samples; i++) {
    Villas__Node__Sample *pb_smp = pb_msg->samples[i];

    if (pb_smp->type != VILLAS__NODE__SAMPLE__TYPE__DATA)
      throw SystemError("ProfileVoltageSource::ProfileVoltageSource could not unpack Protobuf message");

    if (pb_smp->n_values != 1) {
      throw SystemError("ProfileVoltageSource::ProfileVoltageSource could not unpack Protobuf message");
    }

    for (j = 0; j < pb_smp->n_values; j++) {
      Villas__Node__Value *pb_val = pb_smp->values[j];

      if (pb_val->value_case != VILLAS__NODE__VALUE__VALUE_F)
        throw SystemError("ProfileVoltageSource::ProfileVoltageSource could not unpack Protobuf message");
      mSamples.push_back(pb_val->f);
    }
  }

  villas__node__message__free_unpacked(pb_msg, nullptr);

  std::cout << "Read " << mSamples.size() << " samples from file " << mSourceFile << std::endl;
  for (double sample : mSamples) {
    std::cout << sample << std::endl;
  }
}

void DP::Ph1::ProfileVoltageSource::setSourceFile(std::filesystem::path file, size_t index) {
  mSourceFile = file;
  mSourceIndex = index;
  readFromFile();
}

SimPowerComp<Complex>::Ptr DP::Ph1::ProfileVoltageSource::clone(String name) {
  auto copy = ProfileVoltageSource::make(name, mSourceFile, mLogLevel);
  copy->setSourceFile(mSourceFile, mSourceIndex);
  return copy;
}

void DP::Ph1::ProfileVoltageSource::initializeFromNodesAndTerminals(Real frequency) {
  /// CHECK: The frequency parameter is unused
  if (**mVoltage == Complex(0, 0))
    **mVoltage = initialSingleVoltage(1) - initialSingleVoltage(0);
}

// #### MNA functions ####

void DP::Ph1::ProfileVoltageSource::mnaCompAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) {
  attributeDependencies.push_back(mVoltage);
  modifiedAttributes.push_back(mRightVector);
  modifiedAttributes.push_back(mIntfVoltage);
}

void DP::Ph1::ProfileVoltageSource::mnaCompAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes,
                                                                   Attribute<Matrix>::Ptr &leftVector) {
  attributeDependencies.push_back(leftVector);
  modifiedAttributes.push_back(mIntfCurrent);
};

void DP::Ph1::ProfileVoltageSource::mnaCompInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
  updateMatrixNodeIndices();

  (**mIntfVoltage)(0, 0) = *mVoltage;

  SPDLOG_LOGGER_INFO(mSLog,
                     "\n--- MNA initialization ---"
                     "\nInitial voltage {:s}"
                     "\nInitial current {:s}"
                     "\n--- MNA initialization finished ---",
                     Logger::phasorToString((**mIntfVoltage)(0, 0)), Logger::phasorToString((**mIntfCurrent)(0, 0)));
}

void DP::Ph1::ProfileVoltageSource::mnaCompInitializeHarm(Real omega, Real timeStep, std::vector<Attribute<Matrix>::Ptr> leftVectors) {
  updateMatrixNodeIndices();

  (**mIntfVoltage)(0, 0) = *mVoltage;

  mMnaTasks.push_back(std::make_shared<MnaPreStepHarm>(*this));
  mMnaTasks.push_back(std::make_shared<MnaPostStepHarm>(*this, leftVectors));
  **mRightVector = Matrix::Zero(leftVectors[0]->get().rows(), mNumFreqs);
}

void DP::Ph1::ProfileVoltageSource::mnaCompApplySystemMatrixStamp(SparseMatrixRow &systemMatrix) {
  for (UInt freq = 0; freq < mNumFreqs; freq++) {
    if (terminalNotGrounded(0)) {
      Math::setMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(), matrixNodeIndex(0), Complex(-1, 0), mNumFreqs, freq);
      Math::setMatrixElement(systemMatrix, matrixNodeIndex(0), mVirtualNodes[0]->matrixNodeIndex(), Complex(-1, 0), mNumFreqs, freq);
    }
    if (terminalNotGrounded(1)) {
      Math::setMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(), matrixNodeIndex(1), Complex(1, 0), mNumFreqs, freq);
      Math::setMatrixElement(systemMatrix, matrixNodeIndex(1), mVirtualNodes[0]->matrixNodeIndex(), Complex(1, 0), mNumFreqs, freq);
    }

    SPDLOG_LOGGER_INFO(mSLog, "-- Stamp frequency {:d} ---", freq);
    if (terminalNotGrounded(0)) {
      SPDLOG_LOGGER_INFO(mSLog, "Add {:f} to system at ({:d},{:d})", -1., matrixNodeIndex(0), mVirtualNodes[0]->matrixNodeIndex());
      SPDLOG_LOGGER_INFO(mSLog, "Add {:f} to system at ({:d},{:d})", -1., mVirtualNodes[0]->matrixNodeIndex(), matrixNodeIndex(0));
    }
    if (terminalNotGrounded(1)) {
      SPDLOG_LOGGER_INFO(mSLog, "Add {:f} to system at ({:d},{:d})", 1., mVirtualNodes[0]->matrixNodeIndex(), matrixNodeIndex(1));
      SPDLOG_LOGGER_INFO(mSLog, "Add {:f} to system at ({:d},{:d})", 1., matrixNodeIndex(1), mVirtualNodes[0]->matrixNodeIndex());
    }
  }
}

void DP::Ph1::ProfileVoltageSource::mnaCompApplySystemMatrixStampHarm(SparseMatrixRow &systemMatrix, Int freqIdx) {
  if (terminalNotGrounded(0)) {
    Math::setMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(), matrixNodeIndex(0), Complex(-1, 0));
    Math::setMatrixElement(systemMatrix, matrixNodeIndex(0), mVirtualNodes[0]->matrixNodeIndex(), Complex(-1, 0));
  }
  if (terminalNotGrounded(1)) {
    Math::setMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(), matrixNodeIndex(1), Complex(1, 0));
    Math::setMatrixElement(systemMatrix, matrixNodeIndex(1), mVirtualNodes[0]->matrixNodeIndex(), Complex(1, 0));
  }

  SPDLOG_LOGGER_INFO(mSLog, "-- Stamp frequency {:d} ---", freqIdx);
  if (terminalNotGrounded(0)) {
    SPDLOG_LOGGER_INFO(mSLog, "Add {:f} to system at ({:d},{:d})", -1., matrixNodeIndex(0), mVirtualNodes[0]->matrixNodeIndex());
    SPDLOG_LOGGER_INFO(mSLog, "Add {:f} to system at ({:d},{:d})", -1., mVirtualNodes[0]->matrixNodeIndex(), matrixNodeIndex(0));
  }
  if (terminalNotGrounded(1)) {
    SPDLOG_LOGGER_INFO(mSLog, "Add {:f} to system at ({:d},{:d})", 1., mVirtualNodes[0]->matrixNodeIndex(), matrixNodeIndex(1));
    SPDLOG_LOGGER_INFO(mSLog, "Add {:f} to system at ({:d},{:d})", 1., matrixNodeIndex(1), mVirtualNodes[0]->matrixNodeIndex());
  }
}

void DP::Ph1::ProfileVoltageSource::mnaCompApplyRightSideVectorStamp(Matrix &rightVector) {
  // TODO: Is this correct with two nodes not gnd?
  Math::setVectorElement(rightVector, mVirtualNodes[0]->matrixNodeIndex(), (**mIntfVoltage)(0, 0), mNumFreqs);
  SPDLOG_LOGGER_DEBUG(mSLog, "Add {:s} to source vector at {:d}", Logger::complexToString((**mIntfVoltage)(0, 0)), mVirtualNodes[0]->matrixNodeIndex());
}

void DP::Ph1::ProfileVoltageSource::mnaCompApplyRightSideVectorStampHarm(Matrix &rightVector) {
  for (UInt freq = 0; freq < mNumFreqs; freq++) {
    // TODO: Is this correct with two nodes not gnd?
    Math::setVectorElement(rightVector, mVirtualNodes[0]->matrixNodeIndex(), (**mIntfVoltage)(0, freq), 1, 0, freq);
    SPDLOG_LOGGER_DEBUG(mSLog, "Add {:s} to source vector at {:d}", Logger::complexToString((**mIntfVoltage)(0, freq)), mVirtualNodes[0]->matrixNodeIndex());
  }
}

void DP::Ph1::ProfileVoltageSource::updateVoltage(Real time) {
  (**mIntfVoltage)(0, 0) = mSamples[mSourceIndex];
  mSourceIndex = (mSourceIndex + 1) % mSamples.size();
  // std::cout << "Update voltage to " << (**mIntfVoltage)(0, 0) << " at " << time << std::endl;
}

void DP::Ph1::ProfileVoltageSource::mnaCompPreStep(Real time, Int timeStepCount) {
  updateVoltage(time);
  mnaCompApplyRightSideVectorStamp(**mRightVector);
}

void DP::Ph1::ProfileVoltageSource::MnaPreStepHarm::execute(Real time, Int timeStepCount) {
  mVoltageSource.updateVoltage(time);
  mVoltageSource.mnaCompApplyRightSideVectorStampHarm(**mVoltageSource.mRightVector);
}

void DP::Ph1::ProfileVoltageSource::mnaCompPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) { mnaCompUpdateCurrent(**leftVector); }

void DP::Ph1::ProfileVoltageSource::MnaPostStepHarm::execute(Real time, Int timeStepCount) { mVoltageSource.mnaCompUpdateCurrent(**mLeftVectors[0]); }

void DP::Ph1::ProfileVoltageSource::mnaCompUpdateCurrent(const Matrix &leftVector) {
  for (UInt freq = 0; freq < mNumFreqs; freq++) {
    (**mIntfCurrent)(0, freq) = Math::complexFromVectorElement(leftVector, mVirtualNodes[0]->matrixNodeIndex(), mNumFreqs, freq);
  }
}

void DP::Ph1::ProfileVoltageSource::daeResidual(double ttime, const double state[], const double dstate_dt[], double resid[], std::vector<int> &off) {
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
  int c_offset = off[0] + off[1];                                  // current offset for component
  int n_offset_1 = c_offset + Pos1 + 1;                            // current offset for first nodal equation
  int n_offset_2 = c_offset + Pos2 + 1;                            // current offset for second nodal equation
  resid[c_offset] = (state[Pos2] - state[Pos1]) - state[c_offset]; // Voltage equation for Resistor
  // resid[++c_offset] = ; //TODO : add inductance equation
  resid[n_offset_1] += (**mIntfCurrent)(0, 0).real();
  resid[n_offset_2] += (**mIntfCurrent)(0, 0).real();
  off[1] += 1;
}

Complex DP::Ph1::ProfileVoltageSource::daeInitialize() {
  (**mIntfVoltage)(0, 0) = *mVoltage;
  return *mVoltage;
}
