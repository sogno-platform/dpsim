// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <dpsim-models/EMT/EMT_DC_SSN_PiLine.h>

using namespace CPS;

EMT::DC::SSN::PiLine::PiLine(String uid, String name,
                             Logger::Level logLevel)
    : CompositePowerComp<Real>(uid, name, true, true, logLevel),
      Base::DC::PiLine(mAttributes) {
  mPhaseType = PhaseType::DC;
  setVirtualNodeNumber(1);
  setTerminalNumber(2);
  **mIntfVoltage = Matrix::Zero(1, 1);
  **mIntfCurrent = Matrix::Zero(1, 1);
}

SimPowerComp<Real>::Ptr EMT::DC::SSN::PiLine::clone(String name) {
  auto copy = PiLine::make(name, mLogLevel);
  copy->setParameters(**mSeriesResistance, **mSeriesInductance,
                      **mParallelCapacitance, **mParallelConductance,
                      **mInitialCurrent);
  return copy;
}

void EMT::DC::SSN::PiLine::setParameters(
    Real seriesResistance, Real seriesInductance, Real parallelCapacitance,
    Real parallelConductance, Real initialCurrent) {
  const Real epsilon = std::numeric_limits<Real>::epsilon();
  if (!Math::isFinite(seriesResistance) || seriesResistance <= epsilon)
    throw std::invalid_argument(
        "DC pi-line series resistance must be finite and positive.");
  if (!Math::isFinite(seriesInductance) || seriesInductance <= epsilon)
    throw std::invalid_argument(
        "DC pi-line series inductance must be finite and positive.");
  if (!Math::isFinite(parallelCapacitance) || parallelCapacitance < 0.0 ||
      (parallelCapacitance > 0.0 &&
       parallelCapacitance / 2.0 <= epsilon))
    throw std::invalid_argument(
        "DC pi-line shunt capacitance must be zero or safely positive.");
  if (!Math::isFinite(parallelConductance) || parallelConductance < 0.0 ||
      (parallelConductance > 0.0 && parallelConductance <= epsilon))
    throw std::invalid_argument(
        "DC pi-line shunt conductance must be zero or safely positive.");
  if (!Math::isFinite(initialCurrent))
    throw std::invalid_argument("DC pi-line initial current must be finite.");

  **mSeriesResistance = seriesResistance;
  **mSeriesInductance = seriesInductance;
  **mParallelCapacitance = parallelCapacitance;
  **mParallelConductance = parallelConductance;
  **mInitialCurrent = initialCurrent;
  mParametersSet = true;
}

void EMT::DC::SSN::PiLine::validateDCTerminals() const {
  for (UInt terminalIdx = 0; terminalIdx < 2; ++terminalIdx) {
    const auto terminalNode = const_cast<PiLine *>(this)->node(terminalIdx);
    if (!terminalNode->isGround() &&
        terminalNode->phaseType() != PhaseType::DC)
      throw std::invalid_argument(
          "DC pi-line requires DC nodes or ground at both terminals.");
  }
}

void EMT::DC::SSN::PiLine::createSubComponents() {
  if (mSubCompCreated)
    return;
  if (!mParametersSet)
    throw std::logic_error(
        "setParameters() must be called before pi-line construction.");
  validateDCTerminals();
  mSubCompCreated = true;

  mSeriesResistor = Resistor::make(**mName + "_series_R", mLogLevel);
  mSeriesResistor->setParameters(**mSeriesResistance);
  mSeriesResistor->connect({mTerminals[0]->node(), mVirtualNodes[0]});
  addMNASubComponent(mSeriesResistor,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, false);

  mSeriesInductor = Inductor::make(**mName + "_series_L", mLogLevel);
  mSeriesInductor->setParameters(**mSeriesInductance, **mInitialCurrent);
  mSeriesInductor->connect({mVirtualNodes[0], mTerminals[1]->node()});
  addMNASubComponent(mSeriesInductor,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);

  if (**mParallelConductance > 0.0) {
    const Real halfShuntResistance = 2.0 / **mParallelConductance;
    if (!Math::isFinite(halfShuntResistance))
      throw std::invalid_argument(
          "DC pi-line shunt resistance is non-finite.");

    mShuntResistor0 = Resistor::make(**mName + "_shunt_R0", mLogLevel);
    mShuntResistor0->setParameters(halfShuntResistance);
    mShuntResistor0->connect({EMT::SimNode::GND, mTerminals[0]->node()});
    addMNASubComponent(mShuntResistor0,
                       MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                       MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, false);

    mShuntResistor1 = Resistor::make(**mName + "_shunt_R1", mLogLevel);
    mShuntResistor1->setParameters(halfShuntResistance);
    mShuntResistor1->connect({EMT::SimNode::GND, mTerminals[1]->node()});
    addMNASubComponent(mShuntResistor1,
                       MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                       MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, false);
  }

  if (**mParallelCapacitance > 0.0) {
    const Real halfCapacitance = **mParallelCapacitance / 2.0;
    mShuntCapacitor0 = Capacitor::make(**mName + "_shunt_C0", mLogLevel);
    mShuntCapacitor0->setParameters(halfCapacitance);
    mShuntCapacitor0->connect({EMT::SimNode::GND, mTerminals[0]->node()});
    addMNASubComponent(mShuntCapacitor0,
                       MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                       MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);

    mShuntCapacitor1 = Capacitor::make(**mName + "_shunt_C1", mLogLevel);
    mShuntCapacitor1->setParameters(halfCapacitance);
    mShuntCapacitor1->connect({EMT::SimNode::GND, mTerminals[1]->node()});
    addMNASubComponent(mShuntCapacitor1,
                       MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                       MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);
  }
}

void EMT::DC::SSN::PiLine::initializeParentFromNodesAndTerminals(Real) {
  validateDCTerminals();
  const Complex voltage0 = initialSingleVoltage(0);
  const Complex voltage1 = initialSingleVoltage(1);
  if (!Math::isFinite(voltage0) || !Math::isFinite(voltage1) ||
      std::abs(voltage0.imag()) > std::numeric_limits<Real>::epsilon() ||
      std::abs(voltage1.imag()) > std::numeric_limits<Real>::epsilon())
    throw std::invalid_argument(
        "DC pi-line initial node voltages must be finite real values.");

  (**mIntfVoltage)(0, 0) = voltage1.real() - voltage0.real();
  (**mIntfCurrent)(0, 0) = **mInitialCurrent;

  const Real virtualVoltage =
      voltage0.real() + **mSeriesResistance * **mInitialCurrent;
  mVirtualNodes[0]->setInitialVoltage(Complex(virtualVoltage, 0.0));
}

void EMT::DC::SSN::PiLine::mnaCompUpdateVoltage(const Matrix &leftVector) {
  (**mIntfVoltage)(0, 0) = 0.0;
  if (terminalNotGrounded(1))
    (**mIntfVoltage)(0, 0) =
        Math::realFromVectorElement(leftVector, matrixNodeIndex(1));
  if (terminalNotGrounded(0))
    (**mIntfVoltage)(0, 0) -=
        Math::realFromVectorElement(leftVector, matrixNodeIndex(0));
  if (!Math::isFinite((**mIntfVoltage)(0, 0)))
    throw std::runtime_error(
        "DC pi-line voltage update produced a non-finite value.");
}

void EMT::DC::SSN::PiLine::mnaCompUpdateCurrent(const Matrix &) {
  **mIntfCurrent = mSeriesInductor->intfCurrent();
  if (!(**mIntfCurrent).allFinite())
    throw std::runtime_error(
        "DC pi-line current update produced a non-finite value.");
}

void EMT::DC::SSN::PiLine::mnaParentPreStep(Real, Int) {
  mnaCompApplyRightSideVectorStamp(**mRightVector);
}

void EMT::DC::SSN::PiLine::mnaParentPostStep(
    Real, Int, Attribute<Matrix>::Ptr &leftVector) {
  mnaCompUpdateVoltage(**leftVector);
  mnaCompUpdateCurrent(**leftVector);
}

void EMT::DC::SSN::PiLine::mnaParentAddPreStepDependencies(
    AttributeBase::List &, AttributeBase::List &,
    AttributeBase::List &modifiedAttributes) {
  modifiedAttributes.push_back(mRightVector);
}

void EMT::DC::SSN::PiLine::mnaParentAddPostStepDependencies(
    AttributeBase::List &, AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes,
    Attribute<Matrix>::Ptr &leftVector) {
  attributeDependencies.push_back(leftVector);
  modifiedAttributes.push_back(mIntfVoltage);
  modifiedAttributes.push_back(mIntfCurrent);
}
