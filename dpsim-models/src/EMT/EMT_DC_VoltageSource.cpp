// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <dpsim-models/EMT/EMT_DC_VoltageSource.h>

using namespace CPS;

EMT::DC::VoltageSource::VoltageSource(String uid, String name,
                                      Logger::Level logLevel)
    : MNASimPowerComp<Real>(uid, name, true, true, logLevel),
      mVoltageRef(mAttributes->createDynamic<Real>("V_ref")) {
  mPhaseType = PhaseType::DC;
  setVirtualNodeNumber(1);
  setTerminalNumber(2);
  **mIntfVoltage = Matrix::Zero(1, 1);
  **mIntfCurrent = Matrix::Zero(1, 1);
  **mVoltageRef = 0.0;
}

SimPowerComp<Real>::Ptr EMT::DC::VoltageSource::clone(String name) {
  auto copy = VoltageSource::make(name, mLogLevel);
  copy->setParameters(**mVoltageRef);
  return copy;
}

void EMT::DC::VoltageSource::setParameters(Real voltage) {
  if (!Math::isFinite(voltage))
    throw std::invalid_argument("DC source voltage must be finite.");
  **mVoltageRef = voltage;
  mParametersSet = true;
}

void EMT::DC::VoltageSource::validateDCTerminals() const {
  for (UInt terminalIdx = 0; terminalIdx < 2; ++terminalIdx) {
    const auto terminalNode =
        const_cast<VoltageSource *>(this)->node(terminalIdx);
    if (!terminalNode->isGround() && terminalNode->phaseType() != PhaseType::DC)
      throw std::invalid_argument(
          "DC voltage source requires DC nodes or ground at both terminals.");
  }
}

void EMT::DC::VoltageSource::initializeFromNodesAndTerminals(Real) {
  if (!mParametersSet)
    throw std::logic_error(
        "setParameters() must be called before source initialization.");
  validateDCTerminals();
  (**mIntfVoltage)(0, 0) = **mVoltageRef;
  (**mIntfCurrent)(0, 0) = 0.0;
}

void EMT::DC::VoltageSource::mnaCompInitialize(Real, Real,
                                               Attribute<Matrix>::Ptr) {
  if (!mParametersSet)
    throw std::logic_error(
        "setParameters() must be called before source initialization.");
  validateDCTerminals();
  updateMatrixNodeIndices();
  (**mIntfVoltage)(0, 0) = **mVoltageRef;
}

void EMT::DC::VoltageSource::mnaCompApplySystemMatrixStamp(
    SparseMatrixRow &systemMatrix) {
  const UInt branchIdx = mVirtualNodes.at(0)->matrixNodeIndices().at(0);
  if (terminalNotGrounded(0)) {
    Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0), branchIdx, -1.0);
    Math::addToMatrixElement(systemMatrix, branchIdx, matrixNodeIndex(0), -1.0);
  }
  if (terminalNotGrounded(1)) {
    Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1), branchIdx, 1.0);
    Math::addToMatrixElement(systemMatrix, branchIdx, matrixNodeIndex(1), 1.0);
  }
}

void EMT::DC::VoltageSource::mnaCompApplyRightSideVectorStamp(
    Matrix &rightVector) {
  Math::setVectorElement(rightVector,
                         mVirtualNodes.at(0)->matrixNodeIndices().at(0),
                         (**mIntfVoltage)(0, 0));
}

void EMT::DC::VoltageSource::mnaCompUpdateCurrent(const Matrix &leftVector) {
  (**mIntfCurrent)(0, 0) = Math::realFromVectorElement(
      leftVector, mVirtualNodes.at(0)->matrixNodeIndices().at(0));
  if (!Math::isFinite((**mIntfCurrent)(0, 0)))
    throw std::runtime_error(
        "DC voltage-source current contains a non-finite value.");
}

void EMT::DC::VoltageSource::mnaCompPreStep(Real, Int) {
  if (!Math::isFinite(**mVoltageRef))
    throw std::runtime_error("DC source voltage became non-finite.");
  (**mIntfVoltage)(0, 0) = **mVoltageRef;
  mnaCompApplyRightSideVectorStamp(**mRightVector);
}

void EMT::DC::VoltageSource::mnaCompPostStep(
    Real, Int, Attribute<Matrix>::Ptr &leftVector) {
  mnaCompUpdateCurrent(**leftVector);
}

void EMT::DC::VoltageSource::mnaCompAddPreStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes) {
  // Keep the post-step branch-current reconstruction in the schedule even
  // when no logger or downstream component consumes the public output.
  prevStepDependencies.push_back(mIntfCurrent);
  attributeDependencies.push_back(mVoltageRef);
  modifiedAttributes.push_back(mRightVector);
  modifiedAttributes.push_back(mIntfVoltage);
}

void EMT::DC::VoltageSource::mnaCompAddPostStepDependencies(
    AttributeBase::List &, AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes,
    Attribute<Matrix>::Ptr &leftVector) {
  attributeDependencies.push_back(leftVector);
  modifiedAttributes.push_back(mIntfCurrent);
}
