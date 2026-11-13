/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/EMT/EMT_Ph3_ReducedOrderSynchronGeneratorVBR.h>

#include <cmath>

using namespace CPS;

EMT::Ph3::ReducedOrderSynchronGeneratorVBR::ReducedOrderSynchronGeneratorVBR(
    const String &uid, const String &name, Logger::Level logLevel)
    : Base::ReducedOrderSynchronGenerator<Real>(uid, name, logLevel) {
  mVdq0Value = &mVdq0->get();
  mIdq0Value = &mIdq0->get();
  mEfValue = &mEf->get();

  mPhaseType = PhaseType::ABC;
  setTerminalNumber(1);

  // model variable
  **mIntfVoltage = Matrix::Zero(3, 1);
  **mIntfCurrent = Matrix::Zero(3, 1);
}

EMT::Ph3::ReducedOrderSynchronGeneratorVBR::ReducedOrderSynchronGeneratorVBR(
    const String &name, Logger::Level logLevel)
    : ReducedOrderSynchronGeneratorVBR(name, name, logLevel) {}

void EMT::Ph3::ReducedOrderSynchronGeneratorVBR::initializeResistanceMatrix() {
  // dq0 resistance matrix
  mResistanceMatrixDq0 = MatrixFixedSize<3, 3>::Zero(3, 3);
  mResistanceMatrixDq0 << 0.0, mA, 0.0, mB, 0.0, 0.0, 0.0, 0.0, mL0;

  // This impedance is constant in the rotor frame. Compute its inverse once;
  // each step only rotates the conductance into the abc frame.
  mConductanceMatrixDq0 = mResistanceMatrixDq0.inverse() / mBase_Z;
  mConductanceMatrix = MatrixFixedSize<3, 3>::Zero(3, 3);
}

void EMT::Ph3::ReducedOrderSynchronGeneratorVBR::calculateResistanceMatrix() {
  mConductanceMatrix.noalias() = mDq0ToAbc * mConductanceMatrixDq0 * mAbcToDq0;
}

void EMT::Ph3::ReducedOrderSynchronGeneratorVBR::mnaCompInitialize(
    Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {

  Base::ReducedOrderSynchronGenerator<Real>::mnaCompInitialize(omega, timeStep,
                                                               leftVector);

  if (mModelAsNortonSource) {
    mVariableSystemMatrixEntries.push_back(std::make_pair<UInt, UInt>(
        matrixNodeIndex(0, 0), matrixNodeIndex(0, 0)));
    mVariableSystemMatrixEntries.push_back(std::make_pair<UInt, UInt>(
        matrixNodeIndex(0, 0), matrixNodeIndex(0, 1)));
    mVariableSystemMatrixEntries.push_back(std::make_pair<UInt, UInt>(
        matrixNodeIndex(0, 0), matrixNodeIndex(0, 2)));
    mVariableSystemMatrixEntries.push_back(std::make_pair<UInt, UInt>(
        matrixNodeIndex(0, 1), matrixNodeIndex(0, 0)));
    mVariableSystemMatrixEntries.push_back(std::make_pair<UInt, UInt>(
        matrixNodeIndex(0, 1), matrixNodeIndex(0, 1)));
    mVariableSystemMatrixEntries.push_back(std::make_pair<UInt, UInt>(
        matrixNodeIndex(0, 1), matrixNodeIndex(0, 2)));
    mVariableSystemMatrixEntries.push_back(std::make_pair<UInt, UInt>(
        matrixNodeIndex(0, 2), matrixNodeIndex(0, 0)));
    mVariableSystemMatrixEntries.push_back(std::make_pair<UInt, UInt>(
        matrixNodeIndex(0, 2), matrixNodeIndex(0, 1)));
    mVariableSystemMatrixEntries.push_back(std::make_pair<UInt, UInt>(
        matrixNodeIndex(0, 2), matrixNodeIndex(0, 2)));
  } else {
    // upper left block
    mVariableSystemMatrixEntries.push_back(std::make_pair<UInt, UInt>(
        mVirtualNodes[0]->matrixNodeIndex(PhaseType::A),
        mVirtualNodes[0]->matrixNodeIndex(PhaseType::A)));
    mVariableSystemMatrixEntries.push_back(std::make_pair<UInt, UInt>(
        mVirtualNodes[0]->matrixNodeIndex(PhaseType::A),
        mVirtualNodes[0]->matrixNodeIndex(PhaseType::B)));
    mVariableSystemMatrixEntries.push_back(std::make_pair<UInt, UInt>(
        mVirtualNodes[0]->matrixNodeIndex(PhaseType::A),
        mVirtualNodes[0]->matrixNodeIndex(PhaseType::C)));
    mVariableSystemMatrixEntries.push_back(std::make_pair<UInt, UInt>(
        mVirtualNodes[0]->matrixNodeIndex(PhaseType::B),
        mVirtualNodes[0]->matrixNodeIndex(PhaseType::A)));
    mVariableSystemMatrixEntries.push_back(std::make_pair<UInt, UInt>(
        mVirtualNodes[0]->matrixNodeIndex(PhaseType::B),
        mVirtualNodes[0]->matrixNodeIndex(PhaseType::B)));
    mVariableSystemMatrixEntries.push_back(std::make_pair<UInt, UInt>(
        mVirtualNodes[0]->matrixNodeIndex(PhaseType::B),
        mVirtualNodes[0]->matrixNodeIndex(PhaseType::C)));
    mVariableSystemMatrixEntries.push_back(std::make_pair<UInt, UInt>(
        mVirtualNodes[0]->matrixNodeIndex(PhaseType::C),
        mVirtualNodes[0]->matrixNodeIndex(PhaseType::A)));
    mVariableSystemMatrixEntries.push_back(std::make_pair<UInt, UInt>(
        mVirtualNodes[0]->matrixNodeIndex(PhaseType::C),
        mVirtualNodes[0]->matrixNodeIndex(PhaseType::B)));
    mVariableSystemMatrixEntries.push_back(std::make_pair<UInt, UInt>(
        mVirtualNodes[0]->matrixNodeIndex(PhaseType::C),
        mVirtualNodes[0]->matrixNodeIndex(PhaseType::C)));

    // buttom right block
    mVariableSystemMatrixEntries.push_back(std::make_pair<UInt, UInt>(
        matrixNodeIndex(0, 0), matrixNodeIndex(0, 0)));
    mVariableSystemMatrixEntries.push_back(std::make_pair<UInt, UInt>(
        matrixNodeIndex(0, 0), matrixNodeIndex(0, 1)));
    mVariableSystemMatrixEntries.push_back(std::make_pair<UInt, UInt>(
        matrixNodeIndex(0, 0), matrixNodeIndex(0, 2)));
    mVariableSystemMatrixEntries.push_back(std::make_pair<UInt, UInt>(
        matrixNodeIndex(0, 1), matrixNodeIndex(0, 0)));
    mVariableSystemMatrixEntries.push_back(std::make_pair<UInt, UInt>(
        matrixNodeIndex(0, 1), matrixNodeIndex(0, 1)));
    mVariableSystemMatrixEntries.push_back(std::make_pair<UInt, UInt>(
        matrixNodeIndex(0, 1), matrixNodeIndex(0, 2)));
    mVariableSystemMatrixEntries.push_back(std::make_pair<UInt, UInt>(
        matrixNodeIndex(0, 2), matrixNodeIndex(0, 0)));
    mVariableSystemMatrixEntries.push_back(std::make_pair<UInt, UInt>(
        matrixNodeIndex(0, 2), matrixNodeIndex(0, 1)));
    mVariableSystemMatrixEntries.push_back(std::make_pair<UInt, UInt>(
        matrixNodeIndex(0, 2), matrixNodeIndex(0, 2)));

    // off diagonal blocks
    mVariableSystemMatrixEntries.push_back(std::make_pair<UInt, UInt>(
        mVirtualNodes[0]->matrixNodeIndex(PhaseType::A),
        matrixNodeIndex(0, 0)));
    mVariableSystemMatrixEntries.push_back(std::make_pair<UInt, UInt>(
        mVirtualNodes[0]->matrixNodeIndex(PhaseType::A),
        matrixNodeIndex(0, 1)));
    mVariableSystemMatrixEntries.push_back(std::make_pair<UInt, UInt>(
        mVirtualNodes[0]->matrixNodeIndex(PhaseType::A),
        matrixNodeIndex(0, 2)));
    mVariableSystemMatrixEntries.push_back(std::make_pair<UInt, UInt>(
        mVirtualNodes[0]->matrixNodeIndex(PhaseType::B),
        matrixNodeIndex(0, 0)));
    mVariableSystemMatrixEntries.push_back(std::make_pair<UInt, UInt>(
        mVirtualNodes[0]->matrixNodeIndex(PhaseType::B),
        matrixNodeIndex(0, 1)));
    mVariableSystemMatrixEntries.push_back(std::make_pair<UInt, UInt>(
        mVirtualNodes[0]->matrixNodeIndex(PhaseType::B),
        matrixNodeIndex(0, 2)));
    mVariableSystemMatrixEntries.push_back(std::make_pair<UInt, UInt>(
        mVirtualNodes[0]->matrixNodeIndex(PhaseType::C),
        matrixNodeIndex(0, 0)));
    mVariableSystemMatrixEntries.push_back(std::make_pair<UInt, UInt>(
        mVirtualNodes[0]->matrixNodeIndex(PhaseType::C),
        matrixNodeIndex(0, 1)));
    mVariableSystemMatrixEntries.push_back(std::make_pair<UInt, UInt>(
        mVirtualNodes[0]->matrixNodeIndex(PhaseType::C),
        matrixNodeIndex(0, 2)));

    mVariableSystemMatrixEntries.push_back(std::make_pair<UInt, UInt>(
        matrixNodeIndex(0, 0),
        mVirtualNodes[0]->matrixNodeIndex(PhaseType::A)));
    mVariableSystemMatrixEntries.push_back(std::make_pair<UInt, UInt>(
        matrixNodeIndex(0, 0),
        mVirtualNodes[0]->matrixNodeIndex(PhaseType::B)));
    mVariableSystemMatrixEntries.push_back(std::make_pair<UInt, UInt>(
        matrixNodeIndex(0, 0),
        mVirtualNodes[0]->matrixNodeIndex(PhaseType::C)));
    mVariableSystemMatrixEntries.push_back(std::make_pair<UInt, UInt>(
        matrixNodeIndex(0, 1),
        mVirtualNodes[0]->matrixNodeIndex(PhaseType::A)));
    mVariableSystemMatrixEntries.push_back(std::make_pair<UInt, UInt>(
        matrixNodeIndex(0, 1),
        mVirtualNodes[0]->matrixNodeIndex(PhaseType::B)));
    mVariableSystemMatrixEntries.push_back(std::make_pair<UInt, UInt>(
        matrixNodeIndex(0, 1),
        mVirtualNodes[0]->matrixNodeIndex(PhaseType::C)));
    mVariableSystemMatrixEntries.push_back(std::make_pair<UInt, UInt>(
        matrixNodeIndex(0, 2),
        mVirtualNodes[0]->matrixNodeIndex(PhaseType::A)));
    mVariableSystemMatrixEntries.push_back(std::make_pair<UInt, UInt>(
        matrixNodeIndex(0, 2),
        mVirtualNodes[0]->matrixNodeIndex(PhaseType::B)));
    mVariableSystemMatrixEntries.push_back(std::make_pair<UInt, UInt>(
        matrixNodeIndex(0, 2),
        mVirtualNodes[0]->matrixNodeIndex(PhaseType::C)));
  }

  SPDLOG_LOGGER_INFO(mSLog, "List of index pairs of varying matrix entries: ");
  for (auto indexPair : mVariableSystemMatrixEntries)
    SPDLOG_LOGGER_INFO(mSLog, "({}, {})", indexPair.first, indexPair.second);
}

void EMT::Ph3::ReducedOrderSynchronGeneratorVBR::mnaCompApplySystemMatrixStamp(
    SparseMatrixRow &systemMatrix) {

  if (mDeviceMatrixManaged)
    return;

  if (mModelAsNortonSource) {
    MNAStampUtils::stamp3x3ConductanceMatrixNodeToGround(
        mConductanceMatrix, systemMatrix, matrixNodeIndex(0), mSLog);
  } else {
    // Stamp voltage source
    Math::addToMatrixElement(
        systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::A),
        mVirtualNodes[1]->matrixNodeIndex(PhaseType::A), -1);
    Math::addToMatrixElement(
        systemMatrix, mVirtualNodes[1]->matrixNodeIndex(PhaseType::A),
        mVirtualNodes[0]->matrixNodeIndex(PhaseType::A), 1);
    Math::addToMatrixElement(
        systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::B),
        mVirtualNodes[1]->matrixNodeIndex(PhaseType::B), -1);
    Math::addToMatrixElement(
        systemMatrix, mVirtualNodes[1]->matrixNodeIndex(PhaseType::B),
        mVirtualNodes[0]->matrixNodeIndex(PhaseType::B), 1);
    Math::addToMatrixElement(
        systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::C),
        mVirtualNodes[1]->matrixNodeIndex(PhaseType::C), -1);
    Math::addToMatrixElement(
        systemMatrix, mVirtualNodes[1]->matrixNodeIndex(PhaseType::C),
        mVirtualNodes[0]->matrixNodeIndex(PhaseType::C), 1);

    // Stamp conductance matrix
    MNAStampUtils::stamp3x3ConductanceMatrixBetween2Nodes(
        mConductanceMatrix, systemMatrix,
        mVirtualNodes[0]->matrixNodeIndex(PhaseType::A), matrixNodeIndex(0),
        mSLog);
  }
}

void EMT::Ph3::ReducedOrderSynchronGeneratorVBR::
    mnaCompApplyRightSideVectorStamp(Matrix &rightVector) {
  if (mModelAsNortonSource) {
    // compute equivalent northon circuit in abc reference frame
    mIvbr = mConductanceMatrix * mEvbr;

    Math::setVectorElement(rightVector, matrixNodeIndex(0, 0), mIvbr(0, 0));
    Math::setVectorElement(rightVector, matrixNodeIndex(0, 1), mIvbr(1, 0));
    Math::setVectorElement(rightVector, matrixNodeIndex(0, 2), mIvbr(2, 0));
  } else {
    Math::setVectorElement(rightVector,
                           mVirtualNodes[1]->matrixNodeIndex(PhaseType::A),
                           mEvbr(0, 0));
    Math::setVectorElement(rightVector,
                           mVirtualNodes[1]->matrixNodeIndex(PhaseType::B),
                           mEvbr(1, 0));
    Math::setVectorElement(rightVector,
                           mVirtualNodes[1]->matrixNodeIndex(PhaseType::C),
                           mEvbr(2, 0));
  }
}

void EMT::Ph3::ReducedOrderSynchronGeneratorVBR::
    mnaAddCompactRightSideVectorStamp(Matrix &rightVector) {
  if (mDeviceRhsManaged)
    return;
  const Matrix &stamp = **mRightVector;
  const PhaseType phases[] = {PhaseType::A, PhaseType::B, PhaseType::C};
  for (UInt phase = 0; phase < 3; ++phase) {
    const UInt index = mModelAsNortonSource
                           ? matrixNodeIndex(0, phase)
                           : mVirtualNodes[1]->matrixNodeIndex(phases[phase]);
    rightVector(index, 0) += stamp(index, 0);
  }
}

void EMT::Ph3::ReducedOrderSynchronGeneratorVBR::mnaCompPostStep(
    const Matrix &leftVector) {
  // update armature voltage
  (**mIntfVoltage)(0, 0) =
      Math::realFromVectorElement(leftVector, matrixNodeIndex(0, 0));
  (**mIntfVoltage)(1, 0) =
      Math::realFromVectorElement(leftVector, matrixNodeIndex(0, 1));
  (**mIntfVoltage)(2, 0) =
      Math::realFromVectorElement(leftVector, matrixNodeIndex(0, 2));

  // convert terminal voltage into dq reference frame
  **mVdq0 = mAbcToDq0 * **mIntfVoltage / mBase_V;

  // update armature current
  if (mModelAsNortonSource) {
    **mIntfCurrent = mIvbr - mConductanceMatrix * **mIntfVoltage;
  } else {
    (**mIntfCurrent)(0, 0) = Math::realFromVectorElement(
        leftVector, mVirtualNodes[1]->matrixNodeIndex(PhaseType::A));
    (**mIntfCurrent)(1, 0) = Math::realFromVectorElement(
        leftVector, mVirtualNodes[1]->matrixNodeIndex(PhaseType::B));
    (**mIntfCurrent)(2, 0) = Math::realFromVectorElement(
        leftVector, mVirtualNodes[1]->matrixNodeIndex(PhaseType::C));
  }

  // convert armature current into dq reference frame
  **mIdq0 = mAbcToDq0 * **mIntfCurrent / mBase_I;
}

MatrixFixedSize<3, 3>
EMT::Ph3::ReducedOrderSynchronGeneratorVBR::get_parkTransformMatrix() const {
  MatrixFixedSize<3, 3> abcToDq0;

  abcToDq0 << 2. / 3. * cos(**mThetaMech),
      2. / 3. * cos(**mThetaMech - 2. * PI / 3.),
      2. / 3. * cos(**mThetaMech + 2. * PI / 3.), -2. / 3. * sin(**mThetaMech),
      -2. / 3. * sin(**mThetaMech - 2. * PI / 3.),
      -2. / 3. * sin(**mThetaMech + 2. * PI / 3.), 1. / 3., 1. / 3., 1. / 3.;

  return abcToDq0;
}

MatrixFixedSize<3, 3>
EMT::Ph3::ReducedOrderSynchronGeneratorVBR::get_inverseParkTransformMatrix()
    const {
  MatrixFixedSize<3, 3> dq0ToAbc;

  dq0ToAbc << cos(**mThetaMech), -sin(**mThetaMech), 1.,
      cos(**mThetaMech - 2. * PI / 3.), -sin(**mThetaMech - 2. * PI / 3.), 1.,
      cos(**mThetaMech + 2. * PI / 3.), -sin(**mThetaMech + 2. * PI / 3.), 1.;

  return dq0ToAbc;
}

void EMT::Ph3::ReducedOrderSynchronGeneratorVBR::updateParkTransformMatrices() {
  const Real theta = **mThetaMech;
  const Real thetaB = theta - 2. * PI / 3.;
  const Real thetaC = theta + 2. * PI / 3.;
  const Real cosA = std::cos(theta);
  const Real sinA = std::sin(theta);
  const Real cosB = std::cos(thetaB);
  const Real sinB = std::sin(thetaB);
  const Real cosC = std::cos(thetaC);
  const Real sinC = std::sin(thetaC);

  mAbcToDq0 << 2. / 3. * cosA, 2. / 3. * cosB, 2. / 3. * cosC, -2. / 3. * sinA,
      -2. / 3. * sinB, -2. / 3. * sinC, 1. / 3., 1. / 3., 1. / 3.;
  mDq0ToAbc << cosA, -sinA, 1., cosB, -sinB, 1., cosC, -sinC, 1.;
}
