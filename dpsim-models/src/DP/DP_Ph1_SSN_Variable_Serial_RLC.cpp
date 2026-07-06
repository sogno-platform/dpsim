// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <dpsim-models/DP/DP_Ph1_SSN_Variable_Serial_RLC.h>

using namespace CPS;

DP::Ph1::SSN::Variable_Serial_RLC::Variable_Serial_RLC(String uid, String name,
                                                       Logger::Level logLevel)
    : MixedVTypeVariableSSNComp(uid, name, 0, 2, logLevel) {}

SimPowerComp<Complex>::Ptr
DP::Ph1::SSN::Variable_Serial_RLC::clone(String name) {
  auto copy = Variable_Serial_RLC::make(name, mLogLevel);
  copy->setParameters(mResistance, mInductance, mCapacitance, mOmegaN);
  return copy;
}

void DP::Ph1::SSN::Variable_Serial_RLC::buildStateSpaceModel(Matrix &A,
                                                             Matrix &B,
                                                             Matrix &C,
                                                             Matrix &D) const {
  // Packed real state [Re(vC), Im(vC), Re(iL), Im(iL)]: series RLC coupling
  // applied identically to Re/Im, plus -j*omegaN as a same-state rotation.
  A = Matrix::Zero(4, 4);
  A(0, 1) = mOmegaN;
  A(0, 2) = 1. / mCapacitance;
  A(1, 0) = -mOmegaN;
  A(1, 3) = 1. / mCapacitance;
  A(2, 0) = -1. / mInductance;
  A(2, 2) = -mResistance / mInductance;
  A(2, 3) = mOmegaN;
  A(3, 1) = -1. / mInductance;
  A(3, 2) = -mOmegaN;
  A(3, 3) = -mResistance / mInductance;

  B = Matrix::Zero(4, 2);
  B(2, 0) = 1. / mInductance;
  B(3, 1) = 1. / mInductance;

  C = Matrix::Zero(2, 4);
  C(0, 2) = 1.;
  C(1, 3) = 1.;

  D = Matrix::Zero(2, 2);
}

void DP::Ph1::SSN::Variable_Serial_RLC::setParameters(Real resistance,
                                                      Real inductance,
                                                      Real capacitance,
                                                      Real omegaN) {
  mResistance = resistance;
  mInductance = inductance;
  mCapacitance = capacitance;
  mOmegaN = omegaN;

  Matrix A, B, C, D;
  buildStateSpaceModel(A, B, C, D);

  MixedVTypeVariableSSNComp::setParameters(A, B, C, D);
}

Bool DP::Ph1::SSN::Variable_Serial_RLC::updateComponentParameters() {
  // Change-check intentionally skipped, to exercise recompute-every-step.
  buildStateSpaceModel(mA, mB, mC, mD);
  return true;
}
