/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/EMT/EMT_Ph3_Transformer.h>

using namespace CPS;

EMT::Ph3::Transformer::Transformer(String uid, String name,
                                   Logger::Level logLevel,
                                   Bool withResistiveLosses)
    : Base::Ph3::Transformer(mAttributes),
      CompositePowerComp<Real>(uid, name, true, true, logLevel) {
  mPhaseType = PhaseType::ABC;
  if (withResistiveLosses)
    setVirtualNodeNumber(5);
  else
    setVirtualNodeNumber(3);

  setTerminalNumber(2);

  SPDLOG_LOGGER_INFO(mSLog, "Create {} {}", this->type(), name);
  **mIntfVoltage = Matrix::Zero(3, 1);
  **mIntfCurrent = Matrix::Zero(1, 1);
}

/// DEPRECATED: Delete method
SimPowerComp<Real>::Ptr EMT::Ph3::Transformer::clone(String name) {
  auto copy = Transformer::make(name, mLogLevel);
  copy->setParameters(mNominalVoltagePrimary, mNominalVoltageSecondary,
                      mRatedPower, std::abs(**mRatio), std::arg(**mRatio),
                      mResistance, mInductance);
  return copy;
}

void EMT::Ph3::Transformer::setParameters(Real nomVoltagePrimary,
                                          Real nomVoltageSecondary,
                                          Real ratedPower, Real ratioAbs,
                                          Real ratioPhase, Matrix resistance,
                                          Matrix inductance) {

  Base::Ph3::Transformer::setParameters(nomVoltagePrimary, nomVoltageSecondary,
                                        ratedPower, ratioAbs, ratioPhase,
                                        resistance, inductance);

  SPDLOG_LOGGER_INFO(
      mSLog,
      "Nominal Voltage Primary = {} [V] Nominal Voltage Secondary = {} [V]",
      mNominalVoltagePrimary, mNominalVoltageSecondary);
  SPDLOG_LOGGER_INFO(mSLog, "Rated Apparent Power  = {} [VA]", mRatedPower);
  SPDLOG_LOGGER_INFO(mSLog, "Tap Ratio = {} [ ] Phase Shift = {} [deg]",
                     std::abs(**mRatio), std::arg(**mRatio));

  mParametersSet = true;
}

void EMT::Ph3::Transformer::resolveWindingRoles() {
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

  if (Math::abs(std::arg(mRatioFromReference)) > 1e-9) {
    SPDLOG_LOGGER_ERROR(mSLog,
                        "Turns ratio {} has a phase shift of {} rad. "
                        "EMT::Ph3::Transformer models in-phase turns ratios "
                        "only; a phase-shifting winding connection is not "
                        "implemented.",
                        Logger::complexToString(mRatioFromReference),
                        std::arg(mRatioFromReference));
    throw InvalidArgumentException();
  }
}

void EMT::Ph3::Transformer::createSubComponents() {
  if (mSubCompCreated)
    return;
  mSubCompCreated = true;

  resolveWindingRoles();

  auto midpoint = mVirtualNodes[2];

  mSubInductor =
      std::make_shared<EMT::Ph3::Inductor>(**mName + "_ind", mLogLevel);
  mSubInductor->setParameters(mInductance / 2.);
  addMNASubComponent(mSubInductor, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);

  mSubInductor2 =
      std::make_shared<EMT::Ph3::Inductor>(**mName + "_ind2", mLogLevel);
  mSubInductor2->setParameters(mInductance / 2.);
  addMNASubComponent(mSubInductor2, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);

  if (mNumVirtualNodes == 5) {
    mSubResistor =
        std::make_shared<EMT::Ph3::Resistor>(**mName + "_res", mLogLevel);
    mSubResistor->setParameters(mResistance / 2.);
    mSubResistor->connect({node(mReferenceTerminal), mVirtualNodes[3]});
    mSubInductor->connect({mVirtualNodes[3], midpoint});
    addMNASubComponent(mSubResistor, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                       MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);

    mSubResistor2 =
        std::make_shared<EMT::Ph3::Resistor>(**mName + "_res2", mLogLevel);
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
      std::make_shared<EMT::Ph3::Resistor>(**mName + "_mag_res", mLogLevel);
  mSubMagnetizingResistor->setParameters(mMagnetizingResistance);
  mSubMagnetizingResistor->connect({midpoint, EMT::SimNode::GND});
  addMNASubComponent(mSubMagnetizingResistor,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);

  mSubMagnetizingInductor =
      std::make_shared<EMT::Ph3::Inductor>(**mName + "_mag_ind", mLogLevel);
  mSubMagnetizingInductor->connect({midpoint, EMT::SimNode::GND});
  addMNASubComponent(mSubMagnetizingInductor,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);
}

void EMT::Ph3::Transformer::initializeParentFromNodesAndTerminals(
    Real frequency) {
  // Set initial voltage of virtual node in between
  mVirtualNodes[0]->setInitialVoltage(
      initialSingleVoltage(nonReferenceTerminal()) * mRatioFromReference);

  // Static calculations from load flow data
  Real omega = 2. * PI * frequency;

  if (mSubMagnetizingInductor) {
    Real magnetizingSusceptance =
        std::sqrt(std::pow(mNoLoadCurrent, 2) - std::pow(mNoLoadLoss, 2)) *
        mRatedPower / std::pow(nominalVoltageAt(mReferenceTerminal), 2);
    Real magnetizingInductance = 1. / (omega * magnetizingSusceptance);
    mMagnetizingInductance =
        Math::singlePhaseParameterToThreePhase(magnetizingInductance);
    mSubMagnetizingInductor->setParameters(mMagnetizingInductance);
    SPDLOG_LOGGER_INFO(mSLog, "Magnetizing inductance = {} [H]",
                       Logger::matrixToString(mMagnetizingInductance));
  }

  MatrixComp impedance = MatrixComp::Zero(3, 3);
  impedance << Complex(mResistance(0, 0), omega * mInductance(0, 0)),
      Complex(mResistance(0, 1), omega * mInductance(0, 1)),
      Complex(mResistance(0, 2), omega * mInductance(0, 2)),
      Complex(mResistance(1, 0), omega * mInductance(1, 0)),
      Complex(mResistance(1, 1), omega * mInductance(1, 1)),
      Complex(mResistance(1, 2), omega * mInductance(1, 2)),
      Complex(mResistance(2, 0), omega * mInductance(2, 0)),
      Complex(mResistance(2, 1), omega * mInductance(2, 1)),
      Complex(mResistance(2, 2), omega * mInductance(2, 2));

  SPDLOG_LOGGER_INFO(mSLog,
                     "Resistance (referred to higher voltage side) = {} [Ohm]",
                     Logger::matrixToString(mResistance));
  SPDLOG_LOGGER_INFO(mSLog,
                     "Inductance (referred to higher voltage side) = {} [H]",
                     Logger::matrixToString(mInductance));
  SPDLOG_LOGGER_INFO(mSLog,
                     "Reactance (referred to higher voltage side) = {} [Ohm]",
                     Logger::matrixToString(omega * mInductance));

  MatrixComp vInitABC = MatrixComp::Zero(3, 1);
  vInitABC(0, 0) =
      RMS3PH_TO_PEAK1PH * (mVirtualNodes[0]->initialSingleVoltage() -
                           initialSingleVoltage(mReferenceTerminal));
  vInitABC(1, 0) = vInitABC(0, 0) * SHIFT_TO_PHASE_B;
  vInitABC(2, 0) = vInitABC(0, 0) * SHIFT_TO_PHASE_C;

  MatrixComp vTerminalABC = MatrixComp::Zero(3, 1);
  vTerminalABC(0, 0) =
      RMS3PH_TO_PEAK1PH * (initialSingleVoltage(1) - initialSingleVoltage(0));
  vTerminalABC(1, 0) = vTerminalABC(0, 0) * SHIFT_TO_PHASE_B;
  vTerminalABC(2, 0) = vTerminalABC(0, 0) * SHIFT_TO_PHASE_C;

  MatrixComp iInit = impedance.inverse() * vInitABC;
  **mIntfCurrent = iInit.real();
  **mIntfVoltage = vTerminalABC.real();

  if (mNumVirtualNodes == 3)
    mVirtualNodes[2]->setInitialVoltage(
        initialSingleVoltage(mReferenceTerminal));

  SPDLOG_LOGGER_INFO(
      mSLog,
      "\n--- Initialization from powerflow ---"
      "\nVoltage across: {:s}"
      "\nCurrent: {:s}"
      "\nTerminal 0 voltage: {:s}"
      "\nTerminal 1 voltage: {:s}"
      "\nVirtual Node 1 voltage: {:s}"
      "\n--- Initialization from powerflow finished ---",
      Logger::matrixToString(**mIntfVoltage),
      Logger::matrixToString(**mIntfCurrent),
      Logger::phasorToString(RMS3PH_TO_PEAK1PH * initialSingleVoltage(0)),
      Logger::phasorToString(RMS3PH_TO_PEAK1PH * initialSingleVoltage(1)),
      Logger::phasorToString(RMS3PH_TO_PEAK1PH *
                             mVirtualNodes[0]->initialSingleVoltage()));
}

void EMT::Ph3::Transformer::mnaParentInitialize(
    Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
  SPDLOG_LOGGER_INFO(
      mSLog,
      "\nTerminal 0 connected to {:s} = sim node {:d}"
      "\nTerminal 1 connected to {:s} = sim node {:d}",
      mTerminals[0]->node()->name(), mTerminals[0]->node()->matrixNodeIndex(),
      mTerminals[1]->node()->name(), mTerminals[1]->node()->matrixNodeIndex());
}

void EMT::Ph3::Transformer::mnaCompApplySystemMatrixStamp(
    SparseMatrixRow &systemMatrix) {
  // Ideal transformer equations
  if (terminalNotGrounded(mReferenceTerminal)) {
    Math::setMatrixElement(
        systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::A),
        mVirtualNodes[1]->matrixNodeIndex(PhaseType::A), -1.);
    Math::setMatrixElement(
        systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::B),
        mVirtualNodes[1]->matrixNodeIndex(PhaseType::B), -1.);
    Math::setMatrixElement(
        systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::C),
        mVirtualNodes[1]->matrixNodeIndex(PhaseType::C), -1.);

    Math::setMatrixElement(systemMatrix,
                           mVirtualNodes[1]->matrixNodeIndex(PhaseType::A),
                           mVirtualNodes[0]->matrixNodeIndex(PhaseType::A), 1.);
    Math::setMatrixElement(systemMatrix,
                           mVirtualNodes[1]->matrixNodeIndex(PhaseType::B),
                           mVirtualNodes[0]->matrixNodeIndex(PhaseType::B), 1.);
    Math::setMatrixElement(systemMatrix,
                           mVirtualNodes[1]->matrixNodeIndex(PhaseType::C),
                           mVirtualNodes[0]->matrixNodeIndex(PhaseType::C), 1.);
  }
  if (terminalNotGrounded(nonReferenceTerminal())) {
    Math::setMatrixElement(systemMatrix,
                           matrixNodeIndex(nonReferenceTerminal(), 0),
                           mVirtualNodes[1]->matrixNodeIndex(PhaseType::A),
                           mRatioFromReference.real());
    Math::setMatrixElement(systemMatrix,
                           matrixNodeIndex(nonReferenceTerminal(), 1),
                           mVirtualNodes[1]->matrixNodeIndex(PhaseType::B),
                           mRatioFromReference.real());
    Math::setMatrixElement(systemMatrix,
                           matrixNodeIndex(nonReferenceTerminal(), 2),
                           mVirtualNodes[1]->matrixNodeIndex(PhaseType::C),
                           mRatioFromReference.real());
    Math::setMatrixElement(systemMatrix,
                           mVirtualNodes[1]->matrixNodeIndex(PhaseType::A),
                           matrixNodeIndex(nonReferenceTerminal(), 0),
                           -mRatioFromReference.real());
    Math::setMatrixElement(systemMatrix,
                           mVirtualNodes[1]->matrixNodeIndex(PhaseType::B),
                           matrixNodeIndex(nonReferenceTerminal(), 1),
                           -mRatioFromReference.real());
    Math::setMatrixElement(systemMatrix,
                           mVirtualNodes[1]->matrixNodeIndex(PhaseType::C),
                           matrixNodeIndex(nonReferenceTerminal(), 2),
                           -mRatioFromReference.real());
  }

  // Add subcomps to system matrix
  for (auto subcomp : mSubComponents)
    if (auto mnasubcomp = std::dynamic_pointer_cast<MNAInterface>(subcomp))
      mnasubcomp->mnaApplySystemMatrixStamp(systemMatrix);

  if (terminalNotGrounded(0)) {
    SPDLOG_LOGGER_INFO(mSLog, "Add {:s} to system at ({:d},{:d})",
                       Logger::complexToString(Complex(-1.0, 0)),
                       mVirtualNodes[0]->matrixNodeIndex(PhaseType::A),
                       mVirtualNodes[1]->matrixNodeIndex(PhaseType::A));
    SPDLOG_LOGGER_INFO(mSLog, "Add {:s} to system at ({:d},{:d})",
                       Logger::complexToString(Complex(-1.0, 0)),
                       mVirtualNodes[0]->matrixNodeIndex(PhaseType::B),
                       mVirtualNodes[1]->matrixNodeIndex(PhaseType::B));
    SPDLOG_LOGGER_INFO(mSLog, "Add {:s} to system at ({:d},{:d})",
                       Logger::complexToString(Complex(-1.0, 0)),
                       mVirtualNodes[0]->matrixNodeIndex(PhaseType::C),
                       mVirtualNodes[1]->matrixNodeIndex(PhaseType::C));

    SPDLOG_LOGGER_INFO(mSLog, "Add {:s} to system at ({:d},{:d})",
                       Logger::complexToString(Complex(1.0, 0)),
                       mVirtualNodes[1]->matrixNodeIndex(PhaseType::A),
                       mVirtualNodes[0]->matrixNodeIndex(PhaseType::A));
    SPDLOG_LOGGER_INFO(mSLog, "Add {:s} to system at ({:d},{:d})",
                       Logger::complexToString(Complex(1.0, 0)),
                       mVirtualNodes[1]->matrixNodeIndex(PhaseType::B),
                       mVirtualNodes[0]->matrixNodeIndex(PhaseType::B));
    SPDLOG_LOGGER_INFO(mSLog, "Add {:s} to system at ({:d},{:d})",
                       Logger::complexToString(Complex(1.0, 0)),
                       mVirtualNodes[1]->matrixNodeIndex(PhaseType::C),
                       mVirtualNodes[0]->matrixNodeIndex(PhaseType::C));
  }
  if (terminalNotGrounded(1)) {
    SPDLOG_LOGGER_INFO(mSLog, "Add {:s} to system at ({:d},{:d})",
                       Logger::complexToString(**mRatio), matrixNodeIndex(1, 0),
                       mVirtualNodes[1]->matrixNodeIndex(PhaseType::A));
    SPDLOG_LOGGER_INFO(mSLog, "Add {:s} to system at ({:d},{:d})",
                       Logger::complexToString(**mRatio), matrixNodeIndex(1, 1),
                       mVirtualNodes[1]->matrixNodeIndex(PhaseType::B));
    SPDLOG_LOGGER_INFO(mSLog, "Add {:s} to system at ({:d},{:d})",
                       Logger::complexToString(**mRatio), matrixNodeIndex(1, 2),
                       mVirtualNodes[1]->matrixNodeIndex(PhaseType::C));

    SPDLOG_LOGGER_INFO(mSLog, "Add {:s} to system at ({:d},{:d})",
                       Logger::complexToString(-**mRatio),
                       mVirtualNodes[1]->matrixNodeIndex(PhaseType::A),
                       matrixNodeIndex(1, 0));
    SPDLOG_LOGGER_INFO(mSLog, "Add {:s} to system at ({:d},{:d})",
                       Logger::complexToString(-**mRatio),
                       mVirtualNodes[1]->matrixNodeIndex(PhaseType::B),
                       matrixNodeIndex(1, 1));
    SPDLOG_LOGGER_INFO(mSLog, "Add {:s} to system at ({:d},{:d})",
                       Logger::complexToString(-**mRatio),
                       mVirtualNodes[1]->matrixNodeIndex(PhaseType::C),
                       matrixNodeIndex(1, 2));
  }
}

void EMT::Ph3::Transformer::mnaParentAddPreStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes) {
  prevStepDependencies.push_back(mIntfCurrent);
  prevStepDependencies.push_back(mIntfVoltage);
  modifiedAttributes.push_back(mRightVector);
}

void EMT::Ph3::Transformer::mnaParentPreStep(Real time, Int timeStepCount) {
  mnaCompApplyRightSideVectorStamp(**mRightVector);
}

void EMT::Ph3::Transformer::mnaParentAddPostStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes,
    Attribute<Matrix>::Ptr &leftVector) {
  attributeDependencies.push_back(leftVector);
  modifiedAttributes.push_back(mIntfVoltage);
  modifiedAttributes.push_back(mIntfCurrent);
}

void EMT::Ph3::Transformer::mnaParentPostStep(
    Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
  mnaCompUpdateVoltage(**leftVector);
  mnaCompUpdateCurrent(**leftVector);
}

void EMT::Ph3::Transformer::mnaCompUpdateCurrent(const Matrix &leftVector) {
  **mIntfCurrent = mOrientationSign * mSubInductor->intfCurrent();
}

void EMT::Ph3::Transformer::mnaCompUpdateVoltage(const Matrix &leftVector) {
  // v1 - v0
  **mIntfVoltage = Matrix::Zero(3, 1);
  if (terminalNotGrounded(1)) {
    (**mIntfVoltage)(0, 0) =
        Math::realFromVectorElement(leftVector, matrixNodeIndex(1, 0));
    (**mIntfVoltage)(1, 0) =
        Math::realFromVectorElement(leftVector, matrixNodeIndex(1, 1));
    (**mIntfVoltage)(2, 0) =
        Math::realFromVectorElement(leftVector, matrixNodeIndex(1, 2));
  }
  if (terminalNotGrounded(0)) {
    (**mIntfVoltage)(0, 0) =
        (**mIntfVoltage)(0, 0) -
        Math::realFromVectorElement(leftVector, matrixNodeIndex(0, 0));
    (**mIntfVoltage)(1, 0) =
        (**mIntfVoltage)(1, 0) -
        Math::realFromVectorElement(leftVector, matrixNodeIndex(0, 1));
    (**mIntfVoltage)(2, 0) =
        (**mIntfVoltage)(2, 0) -
        Math::realFromVectorElement(leftVector, matrixNodeIndex(0, 2));
  }
}
