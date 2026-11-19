/* SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
 * SPDX-License-Identifier: MPL-2.0
 */

#include <dpsim-models/EMT/EMT_Ph3_SSN_ImpedanceLoad.h>
#include <dpsim-models/MNAStampUtils.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>

using namespace CPS;

EMT::Ph3::SSN::ImpedanceLoad::ImpedanceLoad(String uid, String name,
                                            Logger::Level logLevel)
    : VTypeSSNComp(uid, name, 3, 3, logLevel) {
  setTerminalNumber(1);
  *mIntfVoltageValue = Matrix::Zero(3, 1);
  *mIntfCurrentValue = Matrix::Zero(3, 1);
}

SimPowerComp<Real>::Ptr EMT::Ph3::SSN::ImpedanceLoad::clone(String copySuffix) {
  auto copy = SharedFactory<ImpedanceLoad>::make(copySuffix, mLogLevel);
  copy->setParameters(mActivePower, mReactivePower, mNominalVoltage,
                      mShuntConductance, mShuntSusceptance);
  return copy;
}

void EMT::Ph3::SSN::ImpedanceLoad::setParameters(Real activePower,
                                                 Real reactivePower,
                                                 Real nominalVoltage,
                                                 Real shuntConductance,
                                                 Real shuntSusceptance) {
  if (!std::isfinite(activePower) || !std::isfinite(reactivePower) ||
      !std::isfinite(nominalVoltage) || !std::isfinite(shuntConductance) ||
      !std::isfinite(shuntSusceptance) ||
      ((activePower != 0. || reactivePower != 0.) && nominalVoltage <= 0.))
    throw std::invalid_argument("Invalid impedance-load parameters.");
  mActivePower = activePower;
  mReactivePower = reactivePower;
  mNominalVoltage = nominalVoltage;
  mShuntConductance = shuntConductance;
  mShuntSusceptance = shuntSusceptance;
  mLoadConductance = nominalVoltage > 0.
                         ? activePower / (nominalVoltage * nominalVoltage)
                         : 0.;
  mTotalConductance = mLoadConductance + shuntConductance;
  const Matrix zero = Matrix::Zero(3, 3);
  SSNComp::setParameters(zero, zero, zero,
                         mTotalConductance * Matrix::Identity(3, 3));
}

void EMT::Ph3::SSN::ImpedanceLoad::initializeFromNodesAndTerminals(
    Real frequency) {
  if (frequency <= 0.)
    throw std::invalid_argument("Frequency must be positive.");
  const Real omega = 2. * PI * frequency;
  const Real loadSusceptance =
      mNominalVoltage > 0.
          ? mReactivePower / (mNominalVoltage * mNominalVoltage)
          : 0.;
  const Real inductiveSusceptance =
      std::max(loadSusceptance, 0.) + std::max(-mShuntSusceptance, 0.);
  const Real capacitiveSusceptance =
      std::max(-loadSusceptance, 0.) + std::max(mShuntSusceptance, 0.);
  mLoadInductance =
      inductiveSusceptance > 0. ? 1. / (omega * inductiveSusceptance) : 0.;
  mLoadCapacitance = capacitiveSusceptance / omega;

  const Matrix identity = Matrix::Identity(3, 3);
  Matrix inverseInductance = Matrix::Zero(3, 3);
  Matrix c = Matrix::Zero(3, 3);
  if (mLoadInductance > 0.) {
    inverseInductance = (1. / mLoadInductance) * identity;
    c = identity;
  }
  SSNComp::setParameters(Matrix::Zero(3, 3), inverseInductance, c,
                         mTotalConductance * identity,
                         mLoadCapacitance * identity);
  VTypeSSNComp::initializeFromNodesAndTerminals(frequency);
}

MatrixComp EMT::Ph3::SSN::ImpedanceLoad::buildInitialInputFromNodes(Real) {
  MatrixComp voltage = MatrixComp::Zero(3, 1);
  const Complex phaseA = RMS3PH_TO_PEAK1PH * initialSingleVoltage(0);
  voltage(0, 0) = phaseA;
  voltage(1, 0) = phaseA * SHIFT_TO_PHASE_B;
  voltage(2, 0) = phaseA * SHIFT_TO_PHASE_C;
  return voltage;
}

void EMT::Ph3::SSN::ImpedanceLoad::mnaCompApplySystemMatrixStamp(
    SparseMatrixRow &systemMatrix) {
  MNAStampUtils::stampConductanceMatrix(mW, systemMatrix, matrixNodeIndex(0), 0,
                                        terminalNotGrounded(0), false, mSLog);
}

void EMT::Ph3::SSN::ImpedanceLoad::mnaCompApplyRightSideVectorStamp(
    Matrix &rightVector) {
  if (!terminalNotGrounded(0))
    return;
  for (UInt phase = 0; phase < 3; ++phase)
    Math::setVectorElement(rightVector, matrixNodeIndex(0, phase),
                           -mYHist(phase, 0));
}

void EMT::Ph3::SSN::ImpedanceLoad::mnaCompUpdateVoltage(
    const Matrix &leftVector) {
  mIntfVoltageValue->setZero();
  if (terminalNotGrounded(0))
    for (UInt phase = 0; phase < 3; ++phase)
      (*mIntfVoltageValue)(phase, 0) =
          Math::realFromVectorElement(leftVector, matrixNodeIndex(0, phase));
}
