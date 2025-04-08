/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/DP/DP_Ph3_Capacitor.h>

using namespace CPS;
using namespace CPS::DP::Ph3;

DP::Ph3::Capacitor::Capacitor(String uid, String name, Logger::Level logLevel)
    : MNASimPowerComp<Complex>(uid, name, true, true, logLevel),
      Base::Ph3::Capacitor(mAttributes) {
  mPhaseType = PhaseType::ABC;
  setTerminalNumber(2);
  mEquivCurrent = MatrixComp::Zero(3, 1);
  **mIntfVoltage = MatrixComp::Zero(3, 1);
  **mIntfCurrent = MatrixComp::Zero(3, 1);
}

SimPowerComp<Complex>::Ptr DP::Ph3::Capacitor::clone(String name) {
  auto copy = Capacitor::make(name, mLogLevel);
  copy->setParameters(**mCapacitance);
  return copy;
}

void DP::Ph3::Capacitor::initializeFromNodesAndTerminals(Real frequency) {

  Real omega = 2 * PI * frequency;
  MatrixComp susceptance = Matrix::Zero(3, 3);

  susceptance << Complex(0, omega * (**mCapacitance)(0, 0)),
      Complex(0, omega * (**mCapacitance)(0, 1)),
      Complex(0, omega * (**mCapacitance)(0, 2)),
      Complex(0, omega * (**mCapacitance)(1, 0)),
      Complex(0, omega * (**mCapacitance)(1, 1)),
      Complex(0, omega * (**mCapacitance)(1, 2)),
      Complex(0, omega * (**mCapacitance)(2, 0)),
      Complex(0, omega * (**mCapacitance)(2, 1)),
      Complex(0, omega * (**mCapacitance)(2, 2));

  // IntfVoltage initialization for each phase
  (**mIntfVoltage)(0, 0) = initialSingleVoltage(1) - initialSingleVoltage(0);
  Real voltMag = Math::abs((**mIntfVoltage)(0, 0));
  Real voltPhase = Math::phase((**mIntfVoltage)(0, 0));
  (**mIntfVoltage)(1, 0) = Complex(voltMag * cos(voltPhase - 2. / 3. * M_PI),
                                   voltMag * sin(voltPhase - 2. / 3. * M_PI));
  (**mIntfVoltage)(2, 0) = Complex(voltMag * cos(voltPhase + 2. / 3. * M_PI),
                                   voltMag * sin(voltPhase + 2. / 3. * M_PI));

  **mIntfCurrent = susceptance * **mIntfVoltage;

  SPDLOG_LOGGER_INFO(mSLog, "\n--- Initialize from power flow ---");
  // << "Impedance: " << impedance << std::endl
  // << "Voltage across: " << std::abs((**mIntfVoltage)(0,0))
  // << "<" << Math::phaseDeg((**mIntfVoltage)(0,0)) << std::endl
  // << "Current: " << std::abs((**mIntfCurrent)(0,0))
  // << "<" << Math::phaseDeg((**mIntfCurrent)(0,0)) << std::endl
  // << "Terminal 0 voltage: " << std::abs(initialSingleVoltage(0))
  // << "<" << Math::phaseDeg(initialSingleVoltage(0)) << std::endl
  // << "Terminal 1 voltage: " << std::abs(initialSingleVoltage(1))
  // << "<" << Math::phaseDeg(initialSingleVoltage(1)) << std::endl
  // << "--- Power flow initialization finished ---" << std::endl;
}

void DP::Ph3::Capacitor::initVars(Real omega, Real timeStep) {
  Matrix a = timeStep / 2 * (**mCapacitance).inverse();
  Real b = timeStep * omega / 2.;

  Matrix equivCondReal = a.inverse();
  Matrix equivCondImag = b * equivCondReal;
  mEquivCond = Matrix::Zero(3, 3);
  mEquivCond << Complex(equivCondReal(0, 0), equivCondImag(0, 0)),
      Complex(equivCondReal(0, 1), equivCondImag(0, 1)),
      Complex(equivCondReal(0, 2), equivCondImag(0, 2)),
      Complex(equivCondReal(1, 0), equivCondImag(1, 0)),
      Complex(equivCondReal(1, 1), equivCondImag(1, 1)),
      Complex(equivCondReal(1, 2), equivCondImag(1, 2)),
      Complex(equivCondReal(2, 0), equivCondImag(2, 0)),
      Complex(equivCondReal(2, 1), equivCondImag(2, 1)),
      Complex(equivCondReal(2, 2), equivCondImag(2, 2));

  // Since equivCondReal == a.inverse() and
#if 0
  Matrix mPrevVoltCoeffReal = a.inverse();
  Matrix mPrevVoltCoeffImag = -b * a.inverse();
#endif
  Matrix mPrevVoltCoeffReal = equivCondReal;
  Matrix mPrevVoltCoeffImag = -b * equivCondReal;

  mPrevVoltCoeff = Matrix::Zero(3, 3);
  mPrevVoltCoeff << Complex(mPrevVoltCoeffReal(0, 0), mPrevVoltCoeffImag(0, 0)),
      Complex(mPrevVoltCoeffReal(0, 1), mPrevVoltCoeffImag(0, 1)),
      Complex(mPrevVoltCoeffReal(0, 2), mPrevVoltCoeffImag(0, 2)),
      Complex(mPrevVoltCoeffReal(1, 0), mPrevVoltCoeffImag(1, 0)),
      Complex(mPrevVoltCoeffReal(1, 1), mPrevVoltCoeffImag(1, 1)),
      Complex(mPrevVoltCoeffReal(1, 2), mPrevVoltCoeffImag(1, 2)),
      Complex(mPrevVoltCoeffReal(2, 0), mPrevVoltCoeffImag(2, 0)),
      Complex(mPrevVoltCoeffReal(2, 1), mPrevVoltCoeffImag(2, 1)),
      Complex(mPrevVoltCoeffReal(2, 2), mPrevVoltCoeffImag(2, 2));

  mEquivCurrent = -mPrevVoltCoeff * **mIntfVoltage - **mIntfCurrent;
}

void DP::Ph3::Capacitor::mnaCompInitialize(Real omega, Real timeStep,
                                           Attribute<Matrix>::Ptr leftVector) {
  updateMatrixNodeIndices();
  initVars(omega, timeStep);
#if 0
  Matrix equivCondReal = 2.0 * mCapacitance / timeStep;
  Matrix equivCondImag = omega * mCapacitance;
  mEquivCond <<
    Complex(equivCondReal(0, 0), equivCondImag(0, 0)),
    Complex(equivCondReal(1, 0), equivCondImag(1, 0)),
    Complex(equivCondReal(2, 0), equivCondImag(2, 0));

  // TODO: Something is wrong here -- from Ph1_Capacitor
  Matrix prevVoltCoeffReal = 2.0 * mCapacitance / timeStep;
  Matrix prevVoltCoeffImag = - omega * mCapacitance;
  mPrevVoltCoeff = Matrix::Zero(3, 1);
  mPrevVoltCoeff <<
    Complex(prevVoltCoeffReal(0, 0), prevVoltCoeffImag(0, 0)),
    Complex(prevVoltCoeffReal(1, 0), prevVoltCoeffImag(1, 0)),
    Complex(prevVoltCoeffReal(2, 0), prevVoltCoeffImag(2, 0));

  mEquivCurrent = -**mIntfCurrent + -mPrevVoltCoeff.cwiseProduct( **mIntfVoltage);*/
  // No need to update current now
  //**mIntfCurrent = mEquivCond.cwiseProduct(**mIntfVoltage) + mEquivCurrent;

  mLog.info() << "\n--- MNA Initialization ---" << std::endl
        << "Initial voltage " << Math::abs((**mIntfVoltage)(0,0))
        << "<" << Math::phaseDeg((**mIntfVoltage)(0,0)) << std::endl
        << "Initial current " << Math::abs((**mIntfCurrent)(0,0))
        << "<" << Math::phaseDeg((**mIntfCurrent)(0,0)) << std::endl
        << "--- MNA initialization finished ---" << std::endl;
#endif
}

void DP::Ph3::Capacitor::mnaCompApplySystemMatrixStamp(
    SparseMatrixRow &systemMatrix) {
  MNAStampUtils::stampAdmittanceMatrix(
      mEquivCond, systemMatrix, matrixNodeIndex(0), matrixNodeIndex(1),
      terminalNotGrounded(0), terminalNotGrounded(1), mSLog);
}

void DP::Ph3::Capacitor::mnaCompApplyRightSideVectorStamp(Matrix &rightVector) {
  //mCureqr = mCurrr + mGcr * mDeltavr + mGci * mDeltavi;
  //mCureqi = mCurri + mGcr * mDeltavi - mGci * mDeltavr;

  mEquivCurrent = -**mIntfCurrent + -mPrevVoltCoeff * **mIntfVoltage;

  if (terminalNotGrounded(0)) {
    Math::setVectorElement(rightVector, matrixNodeIndex(0, 0),
                           mEquivCurrent(0, 0));
    Math::setVectorElement(rightVector, matrixNodeIndex(0, 1),
                           mEquivCurrent(1, 0));
    Math::setVectorElement(rightVector, matrixNodeIndex(0, 2),
                           mEquivCurrent(2, 0));
  }
  if (terminalNotGrounded(1)) {
    Math::setVectorElement(rightVector, matrixNodeIndex(1, 0),
                           -mEquivCurrent(0, 0));
    Math::setVectorElement(rightVector, matrixNodeIndex(1, 1),
                           -mEquivCurrent(1, 0));
    Math::setVectorElement(rightVector, matrixNodeIndex(1, 2),
                           -mEquivCurrent(2, 0));
  }
}

void DP::Ph3::Capacitor::mnaCompAddPreStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes) {
  // actually depends on C, but then we'd have to modify the system matrix anyway
  modifiedAttributes.push_back(mRightVector);
  prevStepDependencies.push_back(mIntfCurrent);
  prevStepDependencies.push_back(mIntfVoltage);
}

void DP::Ph3::Capacitor::mnaCompPreStep(Real time, Int timeStepCount) {
  mnaCompApplyRightSideVectorStamp(**mRightVector);
}

void DP::Ph3::Capacitor::mnaCompAddPostStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes,
    Attribute<Matrix>::Ptr &leftVector) {
  attributeDependencies.push_back(leftVector);
  modifiedAttributes.push_back(mIntfVoltage);
  modifiedAttributes.push_back(mIntfCurrent);
}

void DP::Ph3::Capacitor::mnaCompPostStep(Real time, Int timeStepCount,
                                         Attribute<Matrix>::Ptr &leftVector) {
  mnaCompUpdateVoltage(**leftVector);
  mnaCompUpdateCurrent(**leftVector);
}

void DP::Ph3::Capacitor::mnaCompUpdateVoltage(const Matrix &leftVector) {
  // v1 - v0
  **mIntfVoltage = Matrix::Zero(3, 1);
  if (terminalNotGrounded(1)) {
    (**mIntfVoltage)(0, 0) =
        Math::complexFromVectorElement(leftVector, matrixNodeIndex(1, 0));
    (**mIntfVoltage)(1, 0) =
        Math::complexFromVectorElement(leftVector, matrixNodeIndex(1, 1));
    (**mIntfVoltage)(2, 0) =
        Math::complexFromVectorElement(leftVector, matrixNodeIndex(1, 2));
  }
  if (terminalNotGrounded(0)) {
    (**mIntfVoltage)(0, 0) =
        (**mIntfVoltage)(0, 0) -
        Math::complexFromVectorElement(leftVector, matrixNodeIndex(0, 0));
    (**mIntfVoltage)(1, 0) =
        (**mIntfVoltage)(1, 0) -
        Math::complexFromVectorElement(leftVector, matrixNodeIndex(0, 1));
    (**mIntfVoltage)(2, 0) =
        (**mIntfVoltage)(2, 0) -
        Math::complexFromVectorElement(leftVector, matrixNodeIndex(0, 2));
  }
}

void DP::Ph3::Capacitor::mnaCompUpdateCurrent(const Matrix &leftVector) {
  **mIntfCurrent = mEquivCond * **mIntfVoltage + mEquivCurrent;
}
