// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <dpsim-models/EMT/EMT_DC_CurrentSource.h>

using namespace CPS;

EMT::DC::CurrentSource::CurrentSource(String uid, String name,
                                      Logger::Level logLevel)
    : MNASimPowerComp<Real>(uid, name, true, true, logLevel),
      mCurrentRef(mAttributes->createDynamic<Real>("I_ref")) {
  mPhaseType = PhaseType::DC;
  setTerminalNumber(2);
  **mIntfVoltage = Matrix::Zero(1, 1);
  **mIntfCurrent = Matrix::Zero(1, 1);
  **mCurrentRef = 0.0;
}

SimPowerComp<Real>::Ptr EMT::DC::CurrentSource::clone(String name) {
  auto copy = CurrentSource::make(name, mLogLevel);
  copy->setParameters(**mCurrentRef);
  return copy;
}

void EMT::DC::CurrentSource::setParameters(Real current) {
  if (!Math::isFinite(current))
    throw std::invalid_argument("DC source current must be finite.");
  **mCurrentRef = current;
  mParametersSet = true;
}

void EMT::DC::CurrentSource::validateDCTerminals() const {
  for (UInt terminalIdx = 0; terminalIdx < 2; ++terminalIdx) {
    const auto terminalNode =
        const_cast<CurrentSource *>(this)->node(terminalIdx);
    if (!terminalNode->isGround() && terminalNode->phaseType() != PhaseType::DC)
      throw std::invalid_argument(
          "DC current source requires DC nodes or ground at both terminals.");
  }
}

void EMT::DC::CurrentSource::initializeFromNodesAndTerminals(Real) {
  if (!mParametersSet)
    throw std::logic_error(
        "setParameters() must be called before source initialization.");
  validateDCTerminals();
  (**mIntfCurrent)(0, 0) = **mCurrentRef;
  const Complex voltage = initialSingleVoltage(1) - initialSingleVoltage(0);
  if (!Math::isFinite(voltage) ||
      std::abs(voltage.imag()) > std::numeric_limits<Real>::epsilon())
    throw std::invalid_argument(
        "DC initial node voltages must be finite real scalar values.");
  (**mIntfVoltage)(0, 0) = voltage.real();
}

void EMT::DC::CurrentSource::mnaCompInitialize(Real, Real,
                                               Attribute<Matrix>::Ptr) {
  if (!mParametersSet)
    throw std::logic_error(
        "setParameters() must be called before source initialization.");
  validateDCTerminals();
  updateMatrixNodeIndices();
  (**mIntfCurrent)(0, 0) = **mCurrentRef;
}

void EMT::DC::CurrentSource::mnaCompApplyRightSideVectorStamp(
    Matrix &rightVector) {
  if (terminalNotGrounded(0))
    Math::setVectorElement(rightVector, matrixNodeIndex(0),
                           (**mIntfCurrent)(0, 0));
  if (terminalNotGrounded(1))
    Math::setVectorElement(rightVector, matrixNodeIndex(1),
                           -(**mIntfCurrent)(0, 0));
}

void EMT::DC::CurrentSource::mnaCompUpdateVoltage(const Matrix &leftVector) {
  (**mIntfVoltage)(0, 0) = 0.0;
  if (terminalNotGrounded(1))
    (**mIntfVoltage)(0, 0) =
        Math::realFromVectorElement(leftVector, matrixNodeIndex(1));
  if (terminalNotGrounded(0))
    (**mIntfVoltage)(0, 0) -=
        Math::realFromVectorElement(leftVector, matrixNodeIndex(0));
  if (!Math::isFinite((**mIntfVoltage)(0, 0)))
    throw std::runtime_error(
        "DC current-source voltage contains a non-finite value.");
}

void EMT::DC::CurrentSource::mnaCompPreStep(Real, Int) {
  if (!Math::isFinite(**mCurrentRef))
    throw std::runtime_error("DC source current became non-finite.");
  (**mIntfCurrent)(0, 0) = **mCurrentRef;
  mnaCompApplyRightSideVectorStamp(**mRightVector);
}

void EMT::DC::CurrentSource::mnaCompPostStep(
    Real, Int, Attribute<Matrix>::Ptr &leftVector) {
  mnaCompUpdateVoltage(**leftVector);
}

void EMT::DC::CurrentSource::mnaCompAddPreStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes) {
  // Keep the post-step terminal-voltage reconstruction in the schedule even
  // when no logger or downstream component consumes the public output.
  prevStepDependencies.push_back(mIntfVoltage);
  attributeDependencies.push_back(mCurrentRef);
  modifiedAttributes.push_back(mRightVector);
  modifiedAttributes.push_back(mIntfCurrent);
}

void EMT::DC::CurrentSource::mnaCompAddPostStepDependencies(
    AttributeBase::List &, AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes,
    Attribute<Matrix>::Ptr &leftVector) {
  attributeDependencies.push_back(leftVector);
  modifiedAttributes.push_back(mIntfVoltage);
}
