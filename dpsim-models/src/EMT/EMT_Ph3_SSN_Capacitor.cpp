// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <dpsim-models/EMT/EMT_Ph3_SSN_Capacitor.h>

using namespace CPS;

EMT::Ph3::SSN::Capacitor::Capacitor(String uid, String name,
                                    Logger::Level logLevel)
    : TwoTerminalITypeSSNComp(uid, name, logLevel),
      Base::Ph3::Capacitor(mAttributes) {
  mPhaseType = PhaseType::ABC;
}

SimPowerComp<Real>::Ptr EMT::Ph3::SSN::Capacitor::clone(String name) {
  auto copy = Capacitor::make(name, mLogLevel);
  copy->setParameters(**mCapacitance);
  return copy;
}

void EMT::Ph3::SSN::Capacitor::setParameters(Matrix capacitance) {
  Base::Ph3::Capacitor::setParameters(capacitance);

  Matrix aMatrix = Matrix::Zero(3, 3);
  Matrix bMatrix = capacitance.inverse();
  Matrix cMatrix = Matrix::Identity(3, 3);
  Matrix dMatrix = Matrix::Zero(3, 3);

  // I-type capacitor:
  //
  // x = vC_abc
  // u = i_abc
  // y = v_abc
  //
  // dvC/dt = C^{-1} i
  // v      = vC
  TwoTerminalITypeSSNComp::setParameters(aMatrix, bMatrix, cMatrix, dMatrix);
}

MatrixComp
EMT::Ph3::SSN::Capacitor::buildInitialInputFromNodes(Real frequency) {
  Real omega = 2.0 * PI * frequency;

  MatrixComp admittance = MatrixComp::Zero(3, 3);
  admittance = Complex(0.0, omega) * (**mCapacitance).cast<Complex>();

  MatrixComp vInitABC = MatrixComp::Zero(3, 1);

  // Interface voltage convention: v = v_terminal1 - v_terminal0
  vInitABC(0, 0) = RMS3PH_TO_PEAK1PH * initialSingleVoltage(1) -
                   RMS3PH_TO_PEAK1PH * initialSingleVoltage(0);
  vInitABC(1, 0) = vInitABC(0, 0) * SHIFT_TO_PHASE_B;
  vInitABC(2, 0) = vInitABC(0, 0) * SHIFT_TO_PHASE_C;

  MatrixComp iInitABC = admittance * vInitABC;

  SPDLOG_LOGGER_INFO(mSLog,
                     "\nCapacitance [F]: {:s}"
                     "\nAdmittance [S]: {:s}",
                     Logger::matrixToString(**mCapacitance),
                     Logger::matrixCompToString(admittance));

  return iInitABC;
}
