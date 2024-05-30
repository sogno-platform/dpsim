/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/SP/SP_Ph3_Capacitor.h>

using namespace CPS;

SP::Ph3::Capacitor::Capacitor(String uid, String name, Logger::Level logLevel)
    : MNASimPowerComp<Complex>(uid, name, false, true, logLevel),
      Base::Ph3::Capacitor(mAttributes) {
  mPhaseType = PhaseType::ABC;
  setTerminalNumber(2);
  **mIntfVoltage = MatrixComp::Zero(3, 1);
  **mIntfCurrent = MatrixComp::Zero(3, 1);
}

SimPowerComp<Complex>::Ptr SP::Ph3::Capacitor::clone(String name) {
  auto copy = Capacitor::make(name, mLogLevel);
  copy->setParameters(**mCapacitance);
  return copy;
}

void SP::Ph3::Capacitor::initializeFromNodesAndTerminals(Real frequency) {

  Real omega = 2 * PI * frequency;
  mSusceptance = Matrix::Zero(3, 3);
  mSusceptance << Complex(0, omega * (**mCapacitance)(0, 0)),
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
  (**mIntfVoltage)(1, 0) = (**mIntfVoltage)(0, 0) *
                           Complex(cos(-2. / 3. * M_PI), sin(-2. / 3. * M_PI));
  (**mIntfVoltage)(2, 0) = (**mIntfVoltage)(0, 0) *
                           Complex(cos(2. / 3. * M_PI), sin(2. / 3. * M_PI));

  **mIntfCurrent = mSusceptance * **mIntfVoltage;
  // TODO: add updated logger
  /*
	mLog.info() << "\n--- Initialize from power flow ---" << std::endl
		<< "Impedance: " << impedance << std::endl
		<< "Voltage across: " << std::abs((**mIntfVoltage)(0, 0))
		<< "<" << Math::phaseDeg((**mIntfVoltage)(0, 0)) << std::endl
		<< "Current: " << std::abs((**mIntfCurrent)(0, 0))
		<< "<" << Math::phaseDeg((**mIntfCurrent)(0, 0)) << std::endl
		<< "Terminal 0 voltage: " << std::abs(initialSingleVoltage(0))
		<< "<" << Math::phaseDeg(initialSingleVoltage(0)) << std::endl
		<< "Terminal 1 voltage: " << std::abs(initialSingleVoltage(1))
		<< "<" << Math::phaseDeg(initialSingleVoltage(1)) << std::endl
		<< "--- Power flow initialization finished ---" << std::endl;*/
}

void SP::Ph3::Capacitor::mnaCompInitialize(Real omega, Real timeStep,
                                           Attribute<Matrix>::Ptr leftVector) {
  updateMatrixNodeIndices();
  // TODO add updated logger
  /*mLog.info() << "\n--- MNA Initialization ---" << std::endl
		<< "Initial voltage " << Math::abs((**mIntfVoltage)(0, 0))
		<< "<" << Math::phaseDeg((**mIntfVoltage)(0, 0)) << std::endl
		<< "Initial current " << Math::abs((**mIntfCurrent)(0, 0))
		<< "<" << Math::phaseDeg((**mIntfCurrent)(0, 0)) << std::endl
		<< "--- MNA initialization finished ---" << std::endl;*/
}

void SP::Ph3::Capacitor::mnaCompApplySystemMatrixStamp(
    SparseMatrixRow &systemMatrix) {
  MNAStampUtils::stampAdmittanceMatrix(
      mSusceptance, systemMatrix, matrixNodeIndex(0), matrixNodeIndex(1),
      terminalNotGrounded(0), terminalNotGrounded(1), mSLog);
}

void SP::Ph3::Capacitor::mnaCompAddPostStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes,
    Attribute<Matrix>::Ptr &leftVector) {
  attributeDependencies.push_back(leftVector);
  modifiedAttributes.push_back(mIntfVoltage);
  modifiedAttributes.push_back(mIntfCurrent);
}

void SP::Ph3::Capacitor::mnaCompPostStep(Real time, Int timeStepCount,
                                         Attribute<Matrix>::Ptr &leftVector) {
  mnaCompUpdateVoltage(**leftVector);
  mnaCompUpdateCurrent(**leftVector);
}

void SP::Ph3::Capacitor::mnaCompUpdateVoltage(const Matrix &leftVector) {
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

void SP::Ph3::Capacitor::mnaCompUpdateCurrent(const Matrix &leftVector) {
  **mIntfCurrent = mSusceptance * **mIntfVoltage;
}
