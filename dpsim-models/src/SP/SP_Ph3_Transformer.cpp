// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <dpsim-models/SP/SP_Ph3_Transformer.h>

using namespace CPS;

SP::Ph3::Transformer::Transformer(String uid, String name,
                                  Logger::Level logLevel,
                                  Bool withResistiveLosses)
    : Base::Ph3::Transformer(mAttributes),
      CompositePowerComp<Complex>(uid, name, true, true, logLevel) {
  mPhaseType = PhaseType::ABC;
  mWithResistiveLosses = withResistiveLosses;
  if (withResistiveLosses)
    setVirtualNodeNumber(5);
  else
    setVirtualNodeNumber(3);
  setTerminalNumber(2);

  SPDLOG_LOGGER_INFO(mSLog, "Create {} {}", this->type(), name);
  **mIntfVoltage = MatrixComp::Zero(3, 1);
  **mIntfCurrent = MatrixComp::Zero(3, 1);
}

SimPowerComp<Complex>::Ptr SP::Ph3::Transformer::clone(String name) {
  auto copy = Transformer::make(name, mLogLevel);
  copy->setParameters(mNominalVoltagePrimary, mNominalVoltageSecondary,
                      mRatedPower, std::abs(**mRatio), std::arg(**mRatio),
                      mResistance, mInductance);
  return copy;
}

void SP::Ph3::Transformer::setParameters(Real nomVoltagePrimary,
                                         Real nomVoltageSecondary,
                                         Real ratedPower, Real ratioAbs,
                                         Real ratioPhase, Matrix resistance,
                                         Matrix inductance) {
  Base::Ph3::Transformer::setParameters(nomVoltagePrimary, nomVoltageSecondary,
                                        ratedPower, ratioAbs, ratioPhase,
                                        resistance, inductance);
  mParametersSet = true;
}

void SP::Ph3::Transformer::resolveWindingRoles() {
  switch (mReferenceWinding) {
  case WindingReference::Primary:
    mReferenceTerminal = 0;
    break;
  case WindingReference::Secondary:
    mReferenceTerminal = 1;
    break;
  case WindingReference::Tertiary:
    SPDLOG_LOGGER_ERROR(mSLog,
                        "Transformer {}: three-winding transformers are "
                        "not implemented",
                        this->name());
    throw InvalidArgumentException();
  case WindingReference::Auto:
    mReferenceTerminal =
        (mNominalVoltagePrimary >= mNominalVoltageSecondary) ? 0 : 1;
    break;
  }

  mRatioFromReference = (mReferenceTerminal == 0) ? **mRatio : 1. / **mRatio;
  mOrientationSign = (mReferenceTerminal == 0) ? 1. : -1.;

  SPDLOG_LOGGER_INFO(
      mSLog,
      "Impedance referred to winding {} ({} [V]), remaining winding "
      "at terminal {} ({} [V]), ratio = {}",
      mReferenceTerminal + 1, nominalVoltageAt(mReferenceTerminal),
      nonReferenceTerminal(), nominalVoltageAt(nonReferenceTerminal()),
      Logger::complexToString(mRatioFromReference));
}

void SP::Ph3::Transformer::createSubComponents() {
  if (mSubCompCreated)
    return;
  mSubCompCreated = true;

  resolveWindingRoles();

  auto midpoint = mVirtualNodes[2];

  mSubInductor =
      std::make_shared<SP::Ph3::Inductor>(**mName + "_ind", mLogLevel);
  mSubInductor->setParameters(mInductance / 2.);
  addMNASubComponent(mSubInductor, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);

  mSubInductor2 =
      std::make_shared<SP::Ph3::Inductor>(**mName + "_ind2", mLogLevel);
  mSubInductor2->setParameters(mInductance / 2.);
  addMNASubComponent(mSubInductor2, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);

  if (mNumVirtualNodes == 5) {
    mSubResistor =
        std::make_shared<SP::Ph3::Resistor>(**mName + "_res", mLogLevel);
    mSubResistor->setParameters(mResistance / 2.);
    mSubResistor->connect({node(mReferenceTerminal), mVirtualNodes[3]});
    mSubInductor->connect({mVirtualNodes[3], midpoint});
    addMNASubComponent(mSubResistor, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                       MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);

    mSubResistor2 =
        std::make_shared<SP::Ph3::Resistor>(**mName + "_res2", mLogLevel);
    mSubResistor2->setParameters(mResistance / 2.);
    mSubResistor2->connect({midpoint, mVirtualNodes[4]});
    mSubInductor2->connect({mVirtualNodes[4], mVirtualNodes[0]});
    addMNASubComponent(mSubResistor2,
                       MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                       MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);
  } else {
    mSubInductor->connect({node(mReferenceTerminal), midpoint});
    mSubInductor2->connect({midpoint, mVirtualNodes[0]});
  }

  if (mRatedPower <= 0) {
    SPDLOG_LOGGER_WARN(mSLog,
                       "Transformer {}: rated power is {} [VA], so the "
                       "magnetizing branch cannot be sized and is omitted",
                       this->name(), mRatedPower);
    return;
  }

  if (mNoLoadCurrent <= mNoLoadLoss) {
    SPDLOG_LOGGER_ERROR(mSLog,
                        "Transformer {}: no-load current {} must exceed the "
                        "no-load loss {}",
                        this->name(), mNoLoadCurrent, mNoLoadLoss);
    throw InvalidArgumentException();
  }

  Real magnetizingResistance =
      std::pow(nominalVoltageAt(mReferenceTerminal), 2) /
      (mNoLoadLoss * mRatedPower);
  mMagnetizingResistance =
      Math::singlePhaseParameterToThreePhase(magnetizingResistance);
  mSubMagnetizingResistor =
      std::make_shared<SP::Ph3::Resistor>(**mName + "_mag_res", mLogLevel);
  mSubMagnetizingResistor->setParameters(mMagnetizingResistance);
  mSubMagnetizingResistor->connect({midpoint, SP::SimNode::GND});
  addMNASubComponent(mSubMagnetizingResistor,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);

  mSubMagnetizingInductor =
      std::make_shared<SP::Ph3::Inductor>(**mName + "_mag_ind", mLogLevel);
  mSubMagnetizingInductor->connect({midpoint, SP::SimNode::GND});
  addMNASubComponent(mSubMagnetizingInductor,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);
}

void SP::Ph3::Transformer::initializeParentFromNodesAndTerminals(
    Real frequency) {
  Real omega = 2. * PI * frequency;

  if (mSubMagnetizingInductor) {
    Real magnetizingSusceptance =
        std::sqrt(std::pow(mNoLoadCurrent, 2) - std::pow(mNoLoadLoss, 2)) *
        mRatedPower / std::pow(nominalVoltageAt(mReferenceTerminal), 2);
    Real magnetizingInductance = 1. / (omega * magnetizingSusceptance);
    mMagnetizingInductance =
        Math::singlePhaseParameterToThreePhase(magnetizingInductance);
    mSubMagnetizingInductor->setParameters(mMagnetizingInductance);
  }

  MatrixComp secondaryVoltage = initialVoltage(nonReferenceTerminal());
  mVirtualNodes[0]->setInitialVoltage(secondaryVoltage * mRatioFromReference);

  MatrixComp midpointVoltage = 0.5 * (initialVoltage(mReferenceTerminal) +
                                      mVirtualNodes[0]->initialVoltage());
  mVirtualNodes[2]->setInitialVoltage(midpointVoltage);

  if (mNumVirtualNodes == 5) {
    mVirtualNodes[3]->setInitialVoltage(initialVoltage(mReferenceTerminal));
    mVirtualNodes[4]->setInitialVoltage(midpointVoltage);
  }

  **mIntfVoltage = initialVoltage(1) - initialVoltage(0);

  MatrixComp impedance = MatrixComp::Zero(3, 3);
  for (UInt row = 0; row < 3; row++)
    for (UInt col = 0; col < 3; col++)
      impedance(row, col) =
          Complex(mResistance(row, col), omega * mInductance(row, col));

  MatrixComp impedanceVoltage =
      mOrientationSign *
      (mVirtualNodes[0]->initialVoltage() - initialVoltage(mReferenceTerminal));
  **mIntfCurrent = impedance.inverse() * impedanceVoltage;
}

void SP::Ph3::Transformer::mnaParentApplySystemMatrixStamp(
    SparseMatrixRow &systemMatrix) {
  if (terminalNotGrounded(mReferenceTerminal)) {
    for (UInt phase = 0; phase < 3; phase++) {
      Math::setMatrixElement(
          systemMatrix,
          mVirtualNodes[0]->matrixNodeIndex(static_cast<PhaseType>(phase)),
          mVirtualNodes[1]->matrixNodeIndex(static_cast<PhaseType>(phase)),
          Complex(-1.0, 0));
      Math::setMatrixElement(
          systemMatrix,
          mVirtualNodes[1]->matrixNodeIndex(static_cast<PhaseType>(phase)),
          mVirtualNodes[0]->matrixNodeIndex(static_cast<PhaseType>(phase)),
          Complex(1.0, 0));
    }
  }
  if (terminalNotGrounded(nonReferenceTerminal())) {
    for (UInt phase = 0; phase < 3; phase++) {
      Math::setMatrixElement(
          systemMatrix, matrixNodeIndex(nonReferenceTerminal(), phase),
          mVirtualNodes[1]->matrixNodeIndex(static_cast<PhaseType>(phase)),
          std::conj(mRatioFromReference));
      Math::setMatrixElement(
          systemMatrix,
          mVirtualNodes[1]->matrixNodeIndex(static_cast<PhaseType>(phase)),
          matrixNodeIndex(nonReferenceTerminal(), phase), -mRatioFromReference);
    }
  }
}

void SP::Ph3::Transformer::mnaParentAddPreStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes) {
  prevStepDependencies.push_back(mIntfCurrent);
  prevStepDependencies.push_back(mIntfVoltage);
  modifiedAttributes.push_back(mRightVector);
}

void SP::Ph3::Transformer::mnaParentPreStep(Real time, Int timeStepCount) {
  this->mnaApplyRightSideVectorStamp(**this->mRightVector);
}

void SP::Ph3::Transformer::mnaParentAddPostStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes,
    Attribute<Matrix>::Ptr &leftVector) {
  attributeDependencies.push_back(leftVector);
  modifiedAttributes.push_back(mIntfVoltage);
  modifiedAttributes.push_back(mIntfCurrent);
}

void SP::Ph3::Transformer::mnaParentPostStep(
    Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
  mnaCompUpdateVoltage(**leftVector);
  mnaCompUpdateCurrent(**leftVector);
}

void SP::Ph3::Transformer::mnaCompUpdateVoltage(const Matrix &leftVector) {
  **mIntfVoltage = MatrixComp::Zero(3, 1);
  for (UInt phase = 0; phase < 3; phase++) {
    if (terminalNotGrounded(1))
      (**mIntfVoltage)(phase, 0) =
          Math::complexFromVectorElement(leftVector, matrixNodeIndex(1, phase));
    if (terminalNotGrounded(0))
      (**mIntfVoltage)(phase, 0) =
          (**mIntfVoltage)(phase, 0) -
          Math::complexFromVectorElement(leftVector, matrixNodeIndex(0, phase));
  }
}

void SP::Ph3::Transformer::mnaCompUpdateCurrent(const Matrix &leftVector) {
  **mIntfCurrent = mOrientationSign * mSubInductor->intfCurrent();
}
