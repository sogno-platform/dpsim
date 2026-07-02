// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <dpsim-models/DP/DP_Ph3_SSN_Full_Serial_RLC.h>

using namespace CPS;

DP::Ph3::SSN::Full_Serial_RLC::Full_Serial_RLC(String uid, String name,
                                               Logger::Level logLevel)
    : TwoTerminalVTypeSSNComp(uid, name, logLevel),
      Base::Ph3::Resistor(mAttributes), Base::Ph3::Inductor(mAttributes),
      Base::Ph3::Capacitor(mAttributes) {}

SimPowerComp<Complex>::Ptr DP::Ph3::SSN::Full_Serial_RLC::clone(String name) {
  auto copy = SharedFactory<Full_Serial_RLC>::make(name, mLogLevel);
  copy->setParameters(**mResistance, **mInductance, **mCapacitance);
  return copy;
}

void DP::Ph3::SSN::Full_Serial_RLC::setParameters(const Matrix &resistance,
                                                  const Matrix &inductance,
                                                  const Matrix &capacitance) {
  if (resistance.rows() != 3 || resistance.cols() != 3)
    throw std::invalid_argument("Resistance matrix must have size 3 x 3.");

  if (inductance.rows() != 3 || inductance.cols() != 3)
    throw std::invalid_argument("Inductance matrix must have size 3 x 3.");

  if (capacitance.rows() != 3 || capacitance.cols() != 3)
    throw std::invalid_argument("Capacitance matrix must have size 3 x 3.");

  **mResistance = resistance;
  **mInductance = inductance;
  **mCapacitance = capacitance;

  Matrix inverseInductance = inductance.inverse();
  Matrix inverseCapacitance = capacitance.inverse();
  Matrix identity3 = Matrix::Identity(3, 3);

  // State choice:
  // x = [uC_abc; i_abc]  -> 6x1
  // u = v_abc            -> 3x1
  // y = i_abc            -> 3x1

  Matrix aMatrix = Matrix::Zero(6, 6);
  aMatrix.block(0, 3, 3, 3) = inverseCapacitance;
  aMatrix.block(3, 0, 3, 3) = -inverseInductance;
  aMatrix.block(3, 3, 3, 3) = -inverseInductance * resistance;

  Matrix bMatrix = Matrix::Zero(6, 3);
  bMatrix.block(3, 0, 3, 3) = inverseInductance;

  Matrix cMatrix = Matrix::Zero(3, 6);
  cMatrix.block(0, 3, 3, 3) = identity3;

  Matrix dMatrix = Matrix::Zero(3, 3);

  SSNComp::setParameters(aMatrix, bMatrix, cMatrix, dMatrix);
}
