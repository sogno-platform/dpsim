/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/DP/DP_Ph3_VoltageSource.h>

using namespace CPS;

DP::Ph3::VoltageSource::VoltageSource(String uid, String name,
                                      Logger::Level logLevel)
    : MNASimPowerComp<Complex>(uid, name, true, true, logLevel),
      mVoltageRef(mAttributes->create<Complex>("V_ref")) {
  mPhaseType = PhaseType::ABC;
  setVirtualNodeNumber(1);
  setTerminalNumber(2);
  **mIntfVoltage = MatrixComp::Zero(3, 1);
  **mIntfCurrent = MatrixComp::Zero(3, 1);
}

SimPowerComp<Complex>::Ptr DP::Ph3::VoltageSource::clone(String name) {
  auto copy = VoltageSource::make(name, mLogLevel);
  copy->setParameters(**mVoltageRef);
  return copy;
}

void DP::Ph3::VoltageSource::setParameters(Complex voltageRef) {
  **mVoltageRef = voltageRef;
  mParametersSet = true;
}

void DP::Ph3::VoltageSource::initializeFromNodesAndTerminals(Real frequency) {
  if (**mVoltageRef == Complex(0, 0))
    **mVoltageRef = initialSingleVoltage(1) - initialSingleVoltage(0);

#if 0
  mLog.info() << "--- Initialize according to power flow ---" << std::endl;
  mLog.info() << "Terminal 0 voltage: " << std::abs(initialSingleVoltage(0))
    << "<" << std::arg(initialSingleVoltage(0)) << std::endl;
  mLog.info() << "Terminal 1 voltage: " << std::abs(initialSingleVoltage(1))
    << "<" << std::arg(initialSingleVoltage(1)) << std::endl;
  mLog.info() << "Voltage across: " << std::abs(mVoltageRef->get()) << "<" << std::arg(mVoltageRef->get()) << std::endl;
#endif
}

void DP::Ph3::VoltageSource::mnaCompInitialize(
    Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
  updateMatrixNodeIndices();
  (**mIntfVoltage)(0, 0) = **mVoltageRef;
  (**mIntfVoltage)(1, 0) =
      Complex(Math::abs(**mVoltageRef) *
                  cos(Math::phase(**mVoltageRef) - 2. / 3. * M_PI),
              Math::abs(**mVoltageRef) *
                  sin(Math::phase(**mVoltageRef) - 2. / 3. * M_PI));
  (**mIntfVoltage)(2, 0) =
      Complex(Math::abs(**mVoltageRef) *
                  cos(Math::phase(**mVoltageRef) + 2. / 3. * M_PI),
              Math::abs(**mVoltageRef) *
                  sin(Math::phase(**mVoltageRef) + 2. / 3. * M_PI));
}

void DP::Ph3::VoltageSource::mnaCompApplySystemMatrixStamp(
    SparseMatrixRow &systemMatrix) {
  if (terminalNotGrounded(0)) {
    Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 0),
                             mVirtualNodes[0]->matrixNodeIndex(PhaseType::A),
                             Complex(-1, 0));
    Math::addToMatrixElement(systemMatrix,
                             mVirtualNodes[0]->matrixNodeIndex(PhaseType::A),
                             matrixNodeIndex(0, 0), Complex(-1, 0));

    Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 1),
                             mVirtualNodes[0]->matrixNodeIndex(PhaseType::B),
                             Complex(-1, 0));
    Math::addToMatrixElement(systemMatrix,
                             mVirtualNodes[0]->matrixNodeIndex(PhaseType::B),
                             matrixNodeIndex(0, 1), Complex(-1, 0));

    Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 2),
                             mVirtualNodes[0]->matrixNodeIndex(PhaseType::C),
                             Complex(-1, 0));
    Math::addToMatrixElement(systemMatrix,
                             mVirtualNodes[0]->matrixNodeIndex(PhaseType::C),
                             matrixNodeIndex(0, 2), Complex(-1, 0));
  }
  if (terminalNotGrounded(1)) {
    Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 0),
                             mVirtualNodes[0]->matrixNodeIndex(PhaseType::A),
                             Complex(1, 0));
    Math::addToMatrixElement(systemMatrix,
                             mVirtualNodes[0]->matrixNodeIndex(PhaseType::A),
                             matrixNodeIndex(1, 0), Complex(1, 0));

    Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 1),
                             mVirtualNodes[0]->matrixNodeIndex(PhaseType::B),
                             Complex(1, 0));
    Math::addToMatrixElement(systemMatrix,
                             mVirtualNodes[0]->matrixNodeIndex(PhaseType::B),
                             matrixNodeIndex(1, 1), Complex(1, 0));

    Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 2),
                             mVirtualNodes[0]->matrixNodeIndex(PhaseType::C),
                             Complex(1, 0));
    Math::addToMatrixElement(systemMatrix,
                             mVirtualNodes[0]->matrixNodeIndex(PhaseType::C),
                             matrixNodeIndex(1, 2), Complex(1, 0));
  }

#if 0
  mLog.info() << "--- System matrix stamp ---" << std::endl;
  if (terminalNotGrounded(0)) {
    mLog.info() << "Add " << Complex(-1, 0) << " to " << matrixNodeIndex(0) << "," << mVirtualNodes[0]->matrixNodeIndex(PhaseType::A) << std::endl;
    mLog.info() << "Add " << Complex(-1, 0) << " to " << mVirtualNodes[0]->matrixNodeIndex(PhaseType::A) << "," << matrixNodeIndex(0) << std::endl;
  }
  if (terminalNotGrounded(1)) {
    mLog.info() << "Add " << Complex(1, 0) << " to " << mVirtualNodes[0]->matrixNodeIndex(PhaseType::A) << "," << matrixNodeIndex(1) << std::endl;
    mLog.info() << "Add " << Complex(1, 0) << " to " << matrixNodeIndex(1) << "," << mVirtualNodes[0]->matrixNodeIndex(PhaseType::A) << std::endl;
  }
#endif
}

void DP::Ph3::VoltageSource::mnaCompApplyRightSideVectorStamp(
    Matrix &rightVector) {
  Math::setVectorElement(rightVector,
                         mVirtualNodes[0]->matrixNodeIndex(PhaseType::A),
                         (**mIntfVoltage)(0, 0));
  Math::setVectorElement(rightVector,
                         mVirtualNodes[0]->matrixNodeIndex(PhaseType::B),
                         (**mIntfVoltage)(1, 0));
  Math::setVectorElement(rightVector,
                         mVirtualNodes[0]->matrixNodeIndex(PhaseType::C),
                         (**mIntfVoltage)(2, 0));

  //mLog.debug() << "Add " << (**mIntfVoltage)(0,0) << " to source vector " << mVirtualNodes[0]->matrixNodeIndex(PhaseType::A) << std::endl;
}

void DP::Ph3::VoltageSource::updateVoltage(Real time) {
  // can't we just do nothing??
  (**mIntfVoltage)(0, 0) = **mVoltageRef;
  (**mIntfVoltage)(1, 0) =
      Complex(Math::abs(**mVoltageRef) *
                  cos(Math::phase(**mVoltageRef) - 2. / 3. * M_PI),
              Math::abs(**mVoltageRef) *
                  sin(Math::phase(**mVoltageRef) - 2. / 3. * M_PI));
  (**mIntfVoltage)(2, 0) =
      Complex(Math::abs(**mVoltageRef) *
                  cos(Math::phase(**mVoltageRef) + 2. / 3. * M_PI),
              Math::abs(**mVoltageRef) *
                  sin(Math::phase(**mVoltageRef) + 2. / 3. * M_PI));
}

void DP::Ph3::VoltageSource::mnaCompAddPreStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes) {
  attributeDependencies.push_back(mVoltageRef);
  modifiedAttributes.push_back(mRightVector);
  modifiedAttributes.push_back(mIntfVoltage);
}

void DP::Ph3::VoltageSource::mnaCompPreStep(Real time, Int timeStepCount) {
  updateVoltage(time);
  mnaCompApplyRightSideVectorStamp(**mRightVector);
}

void DP::Ph3::VoltageSource::mnaCompAddPostStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes,
    Attribute<Matrix>::Ptr &leftVector) {
  attributeDependencies.push_back(leftVector);
  modifiedAttributes.push_back(mIntfCurrent);
}

void DP::Ph3::VoltageSource::mnaCompPostStep(
    Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
  mnaCompUpdateCurrent(**leftVector);
}

void DP::Ph3::VoltageSource::mnaCompUpdateCurrent(const Matrix &leftVector) {
  (**mIntfCurrent)(0, 0) = Math::complexFromVectorElement(
      leftVector, mVirtualNodes[0]->matrixNodeIndex(PhaseType::A));
  (**mIntfCurrent)(1, 0) = Math::complexFromVectorElement(
      leftVector, mVirtualNodes[0]->matrixNodeIndex(PhaseType::B));
  (**mIntfCurrent)(2, 0) = Math::complexFromVectorElement(
      leftVector, mVirtualNodes[0]->matrixNodeIndex(PhaseType::C));
}

void DP::Ph3::VoltageSource::daeResidual(double ttime, const double state[],
                                         const double dstate_dt[],
                                         double resid[],
                                         std::vector<int> &off) {
  /* New state vector definintion:
   *  state[0]=node0_voltage
   *  state[1]=node1_voltage
   *  ....
   *  state[n]=noden_voltage
   *  state[n+1]=component0_voltage
   *  state[n+2]=component0_inductance (not yet implemented)
   *  ...
   *  state[m-1]=componentm_voltage
   *  state[m]=componentm_inductance
   */

  //int Pos1 = matrixNodeIndex(0);
  //int Pos2 = matrixNodeIndex(1);
  //int c_offset = off[0] + off[1]; //current offset for component
  //int n_offset_1 = c_offset + Pos1 + 1;// current offset for first nodal equation
  //int n_offset_2 = c_offset + Pos2 + 1;// current offset for second nodal equation
  //resid[c_offset] = (state[Pos2] - state[Pos1]) - state[c_offset]; // Voltage equation for Resistor
  ////resid[++c_offset] = ; //TODO : add inductance equation
  //resid[n_offset_1] += std::real(current());
  //resid[n_offset_2] += std::real(current());
  //off[1] += 1;
}

Complex DP::Ph3::VoltageSource::daeInitialize() {
  (**mIntfVoltage)(0, 0) = **mVoltageRef;
  return **mVoltageRef;
}
