// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <dpsim-models/DP/DP_Ph1_SSN_Full_Serial_RLC.h>

using namespace CPS;

DP::Ph1::SSN::Full_Serial_RLC::Full_Serial_RLC(String uid, String name,
                                               Logger::Level logLevel)
    : TwoTerminalVTypeSSNComp(uid, name, logLevel) {}

SimPowerComp<Complex>::Ptr DP::Ph1::SSN::Full_Serial_RLC::clone(String name) {
  auto copy = Full_Serial_RLC::make(name, mLogLevel);
  copy->setParameters(mResistance, mInductance, mCapacitance);
  return copy;
}

void DP::Ph1::SSN::Full_Serial_RLC::setParameters(Real resistance,
                                                  Real inductance,
                                                  Real capacitance) {
  mResistance = resistance;
  mInductance = inductance;
  mCapacitance = capacitance;

  // x = [v_C; i_L], u = port voltage, y = i_L
  Matrix A = Matrix::Zero(2, 2);
  A(0, 1) = 1. / mCapacitance;
  A(1, 0) = -1. / mInductance;
  A(1, 1) = -mResistance / mInductance;
  Matrix B = Matrix::Zero(2, 1);
  B(1, 0) = 1. / mInductance;
  Matrix C = Matrix::Zero(1, 2);
  C(0, 1) = 1.;
  Matrix D = Matrix::Zero(1, 1);

  SSNComp::setParameters(A, B, C, D);
}
