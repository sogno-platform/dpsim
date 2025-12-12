// SPDX-FileCopyrightText: 2025 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <dpsim-models/EMT/EMT_Ph1_SSN_Full_Serial_RLC.h>

using namespace CPS;

EMT::Ph1::SSN::Full_Serial_RLC::Full_Serial_RLC(String uid, String name,
                                                Logger::Level logLevel)
    : MNASimPowerComp<Real>(uid, name, true, true, logLevel),
      Base::Ph1::Resistor(mAttributes), Base::Ph1::Inductor(mAttributes),
      Base::Ph1::Capacitor(mAttributes) {

  mPhaseType = PhaseType::Single;

  **mIntfVoltage = Matrix::Zero(1, 1);
  **mIntfCurrent = Matrix::Zero(1, 1);
  setTerminalNumber(2);
}

SimPowerComp<Real>::Ptr EMT::Ph1::SSN::Full_Serial_RLC::clone(String name) {
  auto copy = Full_Serial_RLC::make(name, mLogLevel);
  copy->setParameters(**mResistance, **mInductance, **mCapacitance);
  return copy;
}

void EMT::Ph1::SSN::Full_Serial_RLC::initializeFromNodesAndTerminals(
    Real frequency) {

  Real omega = 2 * PI * frequency;

  Complex impedance =
      Complex((**mResistance),
              omega * (**mInductance) - 1. / (omega * (**mCapacitance)));

  Complex vInit = RMS3PH_TO_PEAK1PH * initialSingleVoltage(1) -
                  RMS3PH_TO_PEAK1PH * initialSingleVoltage(0);
  (**mIntfVoltage)(0, 0) = vInit.real();
  (**mIntfCurrent)(0, 0) = (vInit / impedance).real();

  SPDLOG_LOGGER_INFO(mSLog,
                     "\n--- Initialization from powerflow ---"
                     "\nVoltage across: {:f}"
                     "\nCurrent: {:f}"
                     "\nTerminal 0 voltage: {:f}"
                     "\nTerminal 1 voltage: {:f}"
                     "\n--- Initialization from powerflow finished ---",
                     (**mIntfVoltage)(0, 0), (**mIntfCurrent)(0, 0),
                     (RMS3PH_TO_PEAK1PH * initialSingleVoltage(0)).real(),
                     (RMS3PH_TO_PEAK1PH * initialSingleVoltage(1)).real());
}

void EMT::Ph1::SSN::Full_Serial_RLC::mnaCompInitialize(
    Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {

  updateMatrixNodeIndices();

  mState = Matrix::Zero(2, 1);
  mYHistory = 0;

  //Fill mDufourAKHat
  //State Equation one: top left entry
  mDufourAKHat(0, 0) =
      1. - ((2. * (timeStep * timeStep)) /
            (4. * (**mInductance) * (**mCapacitance) +
             2. * timeStep * (**mCapacitance) * (**mResistance) +
             timeStep * timeStep));
  //State Equation one: top right entry
  mDufourAKHat(0, 1) =
      (timeStep / (2. * (**mCapacitance))) *
      (1. + ((4. * (**mInductance) * (**mCapacitance) -
              2. * timeStep * (**mResistance) * (**mCapacitance) -
              (timeStep * timeStep)) /
             (4. * (**mInductance) * (**mCapacitance) +
              2. * timeStep * (**mResistance) * (**mCapacitance) +
              (timeStep * timeStep))));
  //State Equation two: bottom left entry
  mDufourAKHat(1, 0) = -((4. * (**mCapacitance) * timeStep) /
                         (4. * (**mInductance) * (**mCapacitance) +
                          2. * timeStep * (**mCapacitance) * (**mResistance) +
                          (timeStep * timeStep)));
  //State Equation two: bottom right entry
  mDufourAKHat(1, 1) = (4. * (**mInductance) * (**mCapacitance) -
                        2. * timeStep * (**mResistance) * (**mCapacitance) -
                        (timeStep * timeStep)) /
                       (4. * (**mInductance) * (**mCapacitance) +
                        2. * timeStep * (**mResistance) * (**mCapacitance) +
                        (timeStep * timeStep));

  ///Fill mDufourBKHat
  //State Equation one: top entry
  mDufourBKHat(0, 0) = (timeStep * timeStep) /
                       (4. * (**mInductance) * (**mCapacitance) +
                        2. * timeStep * (**mCapacitance) * (**mResistance) +
                        (timeStep * timeStep));

  //State Equation two: bottom entry
  mDufourBKHat(1, 0) = (timeStep * 2. * (**mCapacitance)) /
                       (4. * (**mInductance) * (**mCapacitance) +
                        2. * timeStep * (**mCapacitance) * (**mResistance) +
                        (timeStep * timeStep));

  mDufourBKNHat = mDufourBKHat;

  mDufourCKN(0, 1) = 1.;

  mDufourWKN = (mDufourCKN * mDufourBKHat)(0, 0);

  mState(1, 0) = (**mIntfCurrent)(0, 0);
  ssnUpdateState();
  mState(1, 0) = (**mIntfCurrent)(0, 0);

  **mRightVector = Matrix::Zero(leftVector->get().rows(), 1);

  SPDLOG_LOGGER_INFO(mSLog,
                     "\n--- MNA initialization ---"
                     "\nInitial voltage {:s}"
                     "\nInitial current {:s}"
                     "\n--- MNA initialization finished ---",
                     Logger::matrixToString(**mIntfVoltage),
                     Logger::matrixToString(**mIntfCurrent));
  mSLog->flush();
}

void EMT::Ph1::SSN::Full_Serial_RLC::mnaCompApplySystemMatrixStamp(
    SparseMatrixRow &systemMatrix) {
  MNAStampUtils::stampConductance(mDufourWKN, systemMatrix, matrixNodeIndex(0),
                                  matrixNodeIndex(1), terminalNotGrounded(0),
                                  terminalNotGrounded(1), mSLog);
}

void EMT::Ph1::SSN::Full_Serial_RLC::mnaCompApplyRightSideVectorStamp(
    Matrix &rightVector) {
  // Update history term
  mYHistory = (mDufourCKN *
               (mDufourAKHat * mState + mDufourBKHat * **mIntfVoltage))(0, 0);

  if (terminalNotGrounded(0)) {
    Math::setVectorElement(rightVector, matrixNodeIndex(0, 0), mYHistory);
  }
  if (terminalNotGrounded(1)) {
    Math::setVectorElement(rightVector, matrixNodeIndex(1, 0), -mYHistory);
  }
  SPDLOG_LOGGER_DEBUG(
      mSLog, "\nHistory current term (mnaCompApplyRightSideVectorStamp): {:s}",
      Logger::matrixToString(mYHistory));
  mSLog->flush();
}

void EMT::Ph1::SSN::Full_Serial_RLC::mnaCompAddPreStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes) {
  // actually depends on L,C, but then we'd have to modify the system matrix anyway
  prevStepDependencies.push_back(mIntfVoltage);
  prevStepDependencies.push_back(mIntfCurrent);
  modifiedAttributes.push_back(mRightVector);
}

void EMT::Ph1::SSN::Full_Serial_RLC::mnaCompPreStep(Real time,
                                                    Int timeStepCount) {
  mnaCompApplyRightSideVectorStamp(**mRightVector);
}

void EMT::Ph1::SSN::Full_Serial_RLC::mnaCompAddPostStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes,
    Attribute<Matrix>::Ptr &leftVector) {
  attributeDependencies.push_back(leftVector);
  modifiedAttributes.push_back(mIntfVoltage);
  modifiedAttributes.push_back(mIntfCurrent);
}

void EMT::Ph1::SSN::Full_Serial_RLC::mnaCompPostStep(
    Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
  mnaCompUpdateVoltage(**leftVector);
  mnaCompUpdateCurrent(**leftVector);
  ssnUpdateState();
}

void EMT::Ph1::SSN::Full_Serial_RLC::mnaCompUpdateVoltage(
    const Matrix &leftVector) {
  // v1 - v0
  mDufourUNT = (**mIntfVoltage)(0, 0);
  **mIntfVoltage = Matrix::Zero(1, 1);
  if (terminalNotGrounded(1)) {
    (**mIntfVoltage)(0, 0) =
        Math::realFromVectorElement(leftVector, matrixNodeIndex(1, 0));
  }
  if (terminalNotGrounded(0)) {
    (**mIntfVoltage)(0, 0) =
        (**mIntfVoltage)(0, 0) -
        Math::realFromVectorElement(leftVector, matrixNodeIndex(0, 0));
  }
  SPDLOG_LOGGER_DEBUG(mSLog, "\nUpdate Voltage: {:s}",
                      Logger::matrixToString(**mIntfVoltage));
  mSLog->flush();
}

void EMT::Ph1::SSN::Full_Serial_RLC::mnaCompUpdateCurrent(
    const Matrix &leftVector) {
  (**mIntfCurrent)(0, 0) = mYHistory + (mDufourWKN * **mIntfVoltage)(0, 0);

  SPDLOG_LOGGER_DEBUG(mSLog, "\nUpdate Current: {:s}",
                      Logger::matrixToString(**mIntfCurrent));
  mSLog->flush();
}

void EMT::Ph1::SSN::Full_Serial_RLC::setParameters(Real resistance,
                                                   Real inductance,
                                                   Real capacitance) {
  **mInductance = inductance;
  **mCapacitance = capacitance;
  **mResistance = resistance;
  mParametersSet = true;
}

void EMT::Ph1::SSN::Full_Serial_RLC::ssnUpdateState() {
  mState = mDufourAKHat * mState + mDufourBKHat * mDufourUNT +
           mDufourBKNHat * **mIntfVoltage;
}
