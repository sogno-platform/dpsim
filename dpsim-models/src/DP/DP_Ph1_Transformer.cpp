/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/DP/DP_Ph1_Transformer.h>
#include <dpsim-models/MNAStampUtils.h>

using namespace CPS;

DP::Ph1::Transformer::Transformer(String uid, String name,
                                  Logger::Level logLevel,
                                  Bool withResistiveLosses)
    : Transformer(uid, name, true, logLevel, withResistiveLosses) {}

DP::Ph1::Transformer::Transformer(String uid, String name,
                                  Bool useSubcomponents, Logger::Level logLevel,
                                  Bool withResistiveLosses)
    : Base::Ph1::Transformer(mAttributes),
      CompositePowerComp<Complex>(uid, name, true, true, logLevel),
      mUseSubcomponents(useSubcomponents) {
  if (withResistiveLosses)
    setVirtualNodeNumber(5);
  else
    setVirtualNodeNumber(3);

  setTerminalNumber(2);

  SPDLOG_LOGGER_INFO(mSLog, "Create {} {}", this->type(), name);
  **mIntfVoltage = MatrixComp::Zero(1, 1);
  **mIntfCurrent = MatrixComp::Zero(1, 1);
}

/// DEPRECATED: Delete method
SimPowerComp<Complex>::Ptr DP::Ph1::Transformer::clone(String name) {
  auto copy = Transformer::make(name, mUseSubcomponents, mLogLevel,
                                mNumVirtualNodes == 5);
  copy->setParameters(mNominalVoltagePrimary, mNominalVoltageSecondary,
                      std::abs(**mRatio), std::arg(**mRatio), **mResistance,
                      **mInductance);
  copy->setTapRatio(tapRatio());
  return copy;
}

void DP::Ph1::Transformer::setParameters(Real nomVoltagePrimary,
                                         Real nomVoltageSecondary,
                                         Real ratioAbs, Real ratioPhase,
                                         Real resistance, Real inductance) {

  Base::Ph1::Transformer::setParameters(nomVoltagePrimary, nomVoltageSecondary,
                                        ratioAbs, ratioPhase, resistance,
                                        inductance);

  SPDLOG_LOGGER_INFO(
      mSLog, "Nominal Voltage Primary={} [V] Nominal Voltage Secondary={} [V]",
      mNominalVoltagePrimary, mNominalVoltageSecondary);
  SPDLOG_LOGGER_INFO(
      mSLog,
      "Resistance={} [Ohm] Inductance={} [Ohm] (referred to primary side)",
      **mResistance, **mInductance);
  SPDLOG_LOGGER_INFO(mSLog, "Tap Ratio={} [ ] Phase Shift={} [deg]",
                     std::abs(effectiveRatio()), std::arg(effectiveRatio()));
  SPDLOG_LOGGER_INFO(mSLog, "Rated Power ={} [W]", **mRatedPower);

  mParametersSet = true;
}

void DP::Ph1::Transformer::setParameters(Real nomVoltagePrimary,
                                         Real nomVoltageSecondary,
                                         Real ratedPower, Real ratioAbs,
                                         Real ratioPhase, Real resistance,
                                         Real inductance) {

  **mRatedPower = ratedPower;
  SPDLOG_LOGGER_INFO(mSLog, "Rated Power ={} [W]", **mRatedPower);

  DP::Ph1::Transformer::setParameters(nomVoltagePrimary, nomVoltageSecondary,
                                      ratioAbs, ratioPhase, resistance,
                                      inductance);
}

void DP::Ph1::Transformer::resolveWindingRoles() {
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

  mRatioFromReference =
      (mReferenceTerminal == 0) ? effectiveRatio() : 1. / effectiveRatio();
  mOrientationSign = (mReferenceTerminal == 0) ? 1. : -1.;

  if (mReferenceWinding == WindingReference::Auto &&
      std::abs(mNominalVoltagePrimary - mNominalVoltageSecondary) <=
          DOUBLE_EPSILON * mNominalVoltagePrimary)
    SPDLOG_LOGGER_WARN(mSLog,
                       "Transformer {}: both windings are nominally {} [V], so "
                       "the reference winding cannot be resolved from the "
                       "nominal voltages; winding 1 assumed. Pass an explicit "
                       "WindingReference to remove the ambiguity.",
                       this->name(), mNominalVoltagePrimary);

  SPDLOG_LOGGER_INFO(
      mSLog,
      "Impedance referred to winding {} ({} [V]) {}, remaining "
      "winding at terminal {} ({} [V]), ratio = {}",
      mReferenceTerminal + 1, nominalVoltageAt(mReferenceTerminal),
      mReferenceWinding == WindingReference::Auto
          ? "(resolved from the nominal voltages)"
          : "(set explicitly)",
      nonReferenceTerminal(), nominalVoltageAt(nonReferenceTerminal()),
      Logger::complexToString(mRatioFromReference));
}

void DP::Ph1::Transformer::createSubComponents() {
  if (!mUseSubcomponents)
    return;
  if (mSubCompCreated)
    return;
  mSubCompCreated = true;

  resolveWindingRoles();

  auto midpoint = mVirtualNodes[2];

  mSubInductor =
      std::make_shared<DP::Ph1::Inductor>(**mName + "_ind", mLogLevel);
  mSubInductor->setParameters(**mInductance / 2.);
  addMNASubComponent(mSubInductor, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);

  mSubInductor2 =
      std::make_shared<DP::Ph1::Inductor>(**mName + "_ind2", mLogLevel);
  mSubInductor2->setParameters(**mInductance / 2.);
  addMNASubComponent(mSubInductor2, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);

  if (mNumVirtualNodes == 5) {
    mSubResistor =
        std::make_shared<DP::Ph1::Resistor>(**mName + "_res", mLogLevel);
    mSubResistor->setParameters(**mResistance / 2.);
    mSubResistor->connect({node(mReferenceTerminal), mVirtualNodes[3]});
    mSubInductor->connect({mVirtualNodes[3], midpoint});
    addMNASubComponent(mSubResistor, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                       MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);

    mSubResistor2 =
        std::make_shared<DP::Ph1::Resistor>(**mName + "_res2", mLogLevel);
    mSubResistor2->setParameters(**mResistance / 2.);
    mSubResistor2->connect({midpoint, mVirtualNodes[4]});
    mSubInductor2->connect({mVirtualNodes[4], mVirtualNodes[0]});
    addMNASubComponent(mSubResistor2,
                       MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                       MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);
  } else {
    mSubInductor->connect({node(mReferenceTerminal), midpoint});
    mSubInductor2->connect({midpoint, mVirtualNodes[0]});
  }

  if (**mRatedPower <= 0) {
    SPDLOG_LOGGER_WARN(mSLog,
                       "Transformer {}: rated power is {} [VA], so the "
                       "magnetizing branch cannot be sized and is omitted",
                       this->name(), **mRatedPower);
    return;
  }

  if (mNoLoadCurrent <= mNoLoadLoss) {
    SPDLOG_LOGGER_ERROR(mSLog,
                        "Transformer {}: no-load current {} must exceed the "
                        "no-load loss {}; the magnetizing branch has no "
                        "reactive part otherwise",
                        this->name(), mNoLoadCurrent, mNoLoadLoss);
    throw InvalidArgumentException();
  }

  mMagnetizingResistance = std::pow(nominalVoltageAt(mReferenceTerminal), 2) /
                           (mNoLoadLoss * **mRatedPower);
  mSubMagnetizingResistor =
      std::make_shared<DP::Ph1::Resistor>(**mName + "_mag_res", mLogLevel);
  mSubMagnetizingResistor->setParameters(mMagnetizingResistance);
  mSubMagnetizingResistor->connect({midpoint, DP::SimNode::GND});
  SPDLOG_LOGGER_INFO(mSLog,
                     "Magnetizing resistance = {} [Ohm] at the T midpoint",
                     Logger::realToString(mMagnetizingResistance));
  addMNASubComponent(mSubMagnetizingResistor,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);

  mSubMagnetizingInductor =
      std::make_shared<DP::Ph1::Inductor>(**mName + "_mag_ind", mLogLevel);
  mSubMagnetizingInductor->connect({midpoint, DP::SimNode::GND});
  addMNASubComponent(mSubMagnetizingInductor,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);
}

void DP::Ph1::Transformer::initializeParentFromNodesAndTerminals(
    Real frequency) {
  Real omega = 2. * PI * frequency;
  if (!mUseSubcomponents)
    resolveWindingRoles();
  if (mUseSubcomponents ? static_cast<Bool>(mSubMagnetizingInductor)
                        : **mRatedPower > 0.) {
    if (!mUseSubcomponents) {
      if (mNoLoadCurrent <= mNoLoadLoss)
        throw InvalidArgumentException();
      mMagnetizingResistance =
          std::pow(nominalVoltageAt(mReferenceTerminal), 2) /
          (mNoLoadLoss * **mRatedPower);
    }
    Real magnetizingSusceptance =
        std::sqrt(std::pow(mNoLoadCurrent, 2) - std::pow(mNoLoadLoss, 2)) *
        **mRatedPower / std::pow(nominalVoltageAt(mReferenceTerminal), 2);
    mMagnetizingInductance = 1. / (omega * magnetizingSusceptance);
    if (mUseSubcomponents)
      mSubMagnetizingInductor->setParameters(mMagnetizingInductance);
    SPDLOG_LOGGER_INFO(mSLog, "Magnetizing inductance = {} [H]",
                       Logger::realToString(mMagnetizingInductance));
  }

  // Set initial voltage of virtual node in between
  mVirtualNodes[0]->setInitialVoltage(
      initialSingleVoltage(nonReferenceTerminal()) * mRatioFromReference);

  Complex halfImpedance =
      Complex(**mResistance / 2., omega * **mInductance / 2.);
  Complex magnetizingAdmittance = Complex(0, 0);
  if (mUseSubcomponents ? static_cast<Bool>(mSubMagnetizingInductor)
                        : **mRatedPower > 0.)
    magnetizingAdmittance = 1. / mMagnetizingResistance +
                            1. / Complex(0, omega * mMagnetizingInductance);

  Complex referenceVoltage = initialSingleVoltage(mReferenceTerminal);
  Complex midpointVoltage =
      (referenceVoltage + mVirtualNodes[0]->initialSingleVoltage()) /
      (2. + halfImpedance * magnetizingAdmittance);
  mVirtualNodes[2]->setInitialVoltage(midpointVoltage);

  if (mNumVirtualNodes == 5) {
    Complex referenceCurrent =
        (referenceVoltage - midpointVoltage) / halfImpedance;
    Complex nonReferenceCurrent =
        (midpointVoltage - mVirtualNodes[0]->initialSingleVoltage()) /
        halfImpedance;
    mVirtualNodes[3]->setInitialVoltage(referenceVoltage -
                                        referenceCurrent * **mResistance / 2.);
    mVirtualNodes[4]->setInitialVoltage(
        midpointVoltage - nonReferenceCurrent * **mResistance / 2.);
  }

  // Static calculations from load flow data
  Complex impedance = {**mResistance, omega * **mInductance};
  SPDLOG_LOGGER_INFO(mSLog, "Reactance={} [Ohm] (referred to primary side)",
                     omega * **mInductance);
  Complex impedanceVoltage =
      mOrientationSign * (mVirtualNodes[0]->initialSingleVoltage() -
                          initialSingleVoltage(mReferenceTerminal));
  (**mIntfVoltage)(0, 0) = initialSingleVoltage(1) - initialSingleVoltage(0);
  (**mIntfCurrent)(0, 0) = impedanceVoltage / impedance;

  SPDLOG_LOGGER_INFO(
      mSLog,
      "\n--- Initialization from powerflow ---"
      "\nVoltage across: {:s}"
      "\nCurrent: {:s}"
      "\nTerminal 0 voltage: {:s}"
      "\nTerminal 1 voltage: {:s}"
      "\nVirtual Node 1 voltage: {:s}"
      "\n--- Initialization from powerflow finished ---",
      Logger::phasorToString((**mIntfVoltage)(0, 0)),
      Logger::phasorToString((**mIntfCurrent)(0, 0)),
      Logger::phasorToString(initialSingleVoltage(0)),
      Logger::phasorToString(initialSingleVoltage(1)),
      Logger::phasorToString(mVirtualNodes[0]->initialSingleVoltage()));
}

void DP::Ph1::Transformer::mnaParentInitialize(
    Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
  if (!mUseSubcomponents) {
    const Real b = timeStep * omega / 2.;
    const Real inductance = **mInductance / 2.;
    const Real a = timeStep / (2. * inductance);
    const Complex y(a / (1. + b * b), -a * b / (1. + b * b));
    const Complex f((1. - b * b) / (1. + b * b), -2. * b / (1. + b * b));
    mDirectInductorAdmittance = MatrixComp::Constant(mNumFreqs, 1, y);
    mDirectInductorPrevCurrentFactor = MatrixComp::Constant(mNumFreqs, 1, f);
    mDirectInductorCurrent0 = MatrixComp::Zero(mNumFreqs, 1);
    mDirectInductorCurrent1 = MatrixComp::Zero(mNumFreqs, 1);
    mDirectInductorVoltage0 = MatrixComp::Zero(mNumFreqs, 1);
    mDirectInductorVoltage1 = MatrixComp::Zero(mNumFreqs, 1);
    mDirectInductorHistory0 = MatrixComp::Zero(mNumFreqs, 1);
    mDirectInductorHistory1 = MatrixComp::Zero(mNumFreqs, 1);
    mDirectMagnetizingAdmittance = MatrixComp::Zero(mNumFreqs, 1);
    mDirectMagnetizingPrevCurrentFactor = MatrixComp::Zero(mNumFreqs, 1);
    mDirectMagnetizingCurrent = MatrixComp::Zero(mNumFreqs, 1);
    mDirectMagnetizingVoltage = MatrixComp::Zero(mNumFreqs, 1);
    mDirectMagnetizingHistory = MatrixComp::Zero(mNumFreqs, 1);
    if (**mRatedPower > 0.) {
      const Real am = timeStep / (2. * mMagnetizingInductance);
      mDirectMagnetizingAdmittance(0, 0) =
          Complex(am / (1. + b * b), -am * b / (1. + b * b));
      mDirectMagnetizingPrevCurrentFactor(0, 0) = f;
    }
    const auto voltageAt = [&](UInt nodeIndex) {
      return nodeIndex == 0
                 ? Complex(0., 0.)
                 : mVirtualNodes[nodeIndex - 1]->initialSingleVoltage();
    };
    const Complex midpoint = mVirtualNodes[2]->initialSingleVoltage();
    const Complex firstNode = mNumVirtualNodes == 5
                                  ? mVirtualNodes[3]->initialSingleVoltage()
                                  : initialSingleVoltage(mReferenceTerminal);
    const Complex secondNode = mNumVirtualNodes == 5
                                   ? mVirtualNodes[4]->initialSingleVoltage()
                                   : midpoint;
    const Complex ratioNode = mVirtualNodes[0]->initialSingleVoltage();
    for (UInt freq = 0; freq < mNumFreqs; ++freq) {
      const Real harmonicOmega = 2. * PI * mFrequencies(freq, 0);
      if (harmonicOmega != 0.) {
        mDirectInductorCurrent0(freq, 0) =
            (midpoint - firstNode) / Complex(0., harmonicOmega * inductance);
        mDirectInductorCurrent1(freq, 0) =
            (ratioNode - secondNode) / Complex(0., harmonicOmega * inductance);
        if (**mRatedPower > 0.)
          mDirectMagnetizingCurrent(freq, 0) =
              -midpoint / Complex(0., harmonicOmega * mMagnetizingInductance);
      }
      mDirectInductorVoltage0(freq, 0) = midpoint - firstNode;
      mDirectInductorVoltage1(freq, 0) = ratioNode - secondNode;
      mDirectMagnetizingVoltage(freq, 0) = -midpoint;
    }
    return;
  }
  SPDLOG_LOGGER_INFO(
      mSLog,
      "\nTerminal 0 connected to {:s} = sim node {:d}"
      "\nTerminal 1 connected to {:s} = sim node {:d}",
      mTerminals[0]->node()->name(), mTerminals[0]->node()->matrixNodeIndex(),
      mTerminals[1]->node()->name(), mTerminals[1]->node()->matrixNodeIndex());
}

void DP::Ph1::Transformer::mnaCompApplySystemMatrixStamp(
    SparseMatrixRow &systemMatrix) {
  if (!mUseSubcomponents) {
    if (terminalNotGrounded(mReferenceTerminal)) {
      Math::setMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(),
                             mVirtualNodes[1]->matrixNodeIndex(), -1.);
      Math::setMatrixElement(systemMatrix, mVirtualNodes[1]->matrixNodeIndex(),
                             mVirtualNodes[0]->matrixNodeIndex(), 1.);
    }
    if (terminalNotGrounded(nonReferenceTerminal())) {
      Math::setMatrixElement(
          systemMatrix, matrixNodeIndex(nonReferenceTerminal()),
          mVirtualNodes[1]->matrixNodeIndex(), std::conj(mRatioFromReference));
      Math::setMatrixElement(systemMatrix, mVirtualNodes[1]->matrixNodeIndex(),
                             matrixNodeIndex(nonReferenceTerminal()),
                             -mRatioFromReference);
    }
    const UInt midpoint = mVirtualNodes[2]->matrixNodeIndex();
    const UInt ratioNode = mVirtualNodes[0]->matrixNodeIndex();
    const UInt firstNode = mNumVirtualNodes == 5
                               ? mVirtualNodes[3]->matrixNodeIndex()
                               : matrixNodeIndex(mReferenceTerminal);
    const UInt secondNode =
        mNumVirtualNodes == 5 ? mVirtualNodes[4]->matrixNodeIndex() : midpoint;
    if (mNumVirtualNodes == 5) {
      MNAStampUtils::stampAdmittance(
          Complex(2. / **mResistance, 0.), systemMatrix,
          matrixNodeIndex(mReferenceTerminal), firstNode,
          terminalNotGrounded(mReferenceTerminal), true, mSLog);
      MNAStampUtils::stampAdmittance(Complex(2. / **mResistance, 0.),
                                     systemMatrix, midpoint, secondNode, true,
                                     true, mSLog);
    }
    MNAStampUtils::stampAdmittance(mDirectInductorAdmittance(0, 0),
                                   systemMatrix, firstNode, midpoint, true,
                                   true, mSLog);
    MNAStampUtils::stampAdmittance(mDirectInductorAdmittance(0, 0),
                                   systemMatrix, secondNode, ratioNode, true,
                                   true, mSLog);
    if (**mRatedPower > 0.) {
      MNAStampUtils::stampAdmittance(Complex(1. / mMagnetizingResistance, 0.),
                                     systemMatrix, midpoint, 0, true, false,
                                     mSLog);
      MNAStampUtils::stampAdmittance(mDirectMagnetizingAdmittance(0, 0),
                                     systemMatrix, midpoint, 0, true, false,
                                     mSLog);
    }
    return;
  }
  // Ideal transformer equations
  if (terminalNotGrounded(mReferenceTerminal)) {
    Math::setMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(),
                           mVirtualNodes[1]->matrixNodeIndex(),
                           Complex(-1.0, 0));
    Math::setMatrixElement(systemMatrix, mVirtualNodes[1]->matrixNodeIndex(),
                           mVirtualNodes[0]->matrixNodeIndex(),
                           Complex(1.0, 0));
  }
  if (terminalNotGrounded(nonReferenceTerminal())) {
    Math::setMatrixElement(
        systemMatrix, matrixNodeIndex(nonReferenceTerminal()),
        mVirtualNodes[1]->matrixNodeIndex(), std::conj(mRatioFromReference));
    Math::setMatrixElement(systemMatrix, mVirtualNodes[1]->matrixNodeIndex(),
                           matrixNodeIndex(nonReferenceTerminal()),
                           -mRatioFromReference);
  }

  // Add subcomps to system matrix
  for (auto subcomp : mSubComponents)
    if (auto mnasubcomp = std::dynamic_pointer_cast<MNAInterface>(subcomp))
      mnasubcomp->mnaApplySystemMatrixStamp(systemMatrix);

  if (terminalNotGrounded(mReferenceTerminal)) {
    SPDLOG_LOGGER_INFO(mSLog, "Add {:s} to system at ({:d},{:d})",
                       Logger::complexToString(Complex(-1.0, 0)),
                       mVirtualNodes[0]->matrixNodeIndex(),
                       mVirtualNodes[1]->matrixNodeIndex());
    SPDLOG_LOGGER_INFO(mSLog, "Add {:s} to system at ({:d},{:d})",
                       Logger::complexToString(Complex(1.0, 0)),
                       mVirtualNodes[1]->matrixNodeIndex(),
                       mVirtualNodes[0]->matrixNodeIndex());
  }
  if (terminalNotGrounded(nonReferenceTerminal())) {
    SPDLOG_LOGGER_INFO(mSLog, "Add {:s} to system at ({:d},{:d})",
                       Logger::complexToString(mRatioFromReference),
                       matrixNodeIndex(nonReferenceTerminal()),
                       mVirtualNodes[1]->matrixNodeIndex());
    SPDLOG_LOGGER_INFO(mSLog, "Add {:s} to system at ({:d},{:d})",
                       Logger::complexToString(-mRatioFromReference),
                       mVirtualNodes[1]->matrixNodeIndex(),
                       matrixNodeIndex(nonReferenceTerminal()));
  }
}

void DP::Ph1::Transformer::mnaParentApplyRightSideVectorStamp(
    Matrix &rightVector) {
  if (mUseSubcomponents)
    return;
  const UInt midpoint = mVirtualNodes[2]->matrixNodeIndex();
  const UInt ratioNode = mVirtualNodes[0]->matrixNodeIndex();
  const UInt firstNode = mNumVirtualNodes == 5
                             ? mVirtualNodes[3]->matrixNodeIndex()
                             : matrixNodeIndex(mReferenceTerminal);
  const UInt secondNode =
      mNumVirtualNodes == 5 ? mVirtualNodes[4]->matrixNodeIndex() : midpoint;
  for (UInt freq = 0; freq < mNumFreqs; ++freq) {
    const Complex y = mDirectInductorAdmittance(freq, 0);
    mDirectInductorHistory0(freq, 0) =
        y * mDirectInductorVoltage0(freq, 0) +
        mDirectInductorPrevCurrentFactor(freq, 0) *
            mDirectInductorCurrent0(freq, 0);
    mDirectInductorHistory1(freq, 0) =
        y * mDirectInductorVoltage1(freq, 0) +
        mDirectInductorPrevCurrentFactor(freq, 0) *
            mDirectInductorCurrent1(freq, 0);
    if (mNumVirtualNodes == 5 || terminalNotGrounded(mReferenceTerminal))
      Math::setVectorElement(rightVector, firstNode,
                             mDirectInductorHistory0(freq, 0), mNumFreqs, freq);
    Complex midpointHistory = -mDirectInductorHistory0(freq, 0);
    if (mNumVirtualNodes == 5)
      Math::setVectorElement(rightVector, secondNode,
                             mDirectInductorHistory1(freq, 0), mNumFreqs, freq);
    else
      midpointHistory += mDirectInductorHistory1(freq, 0);
    Math::setVectorElement(rightVector, ratioNode,
                           -mDirectInductorHistory1(freq, 0), mNumFreqs, freq);
    if (**mRatedPower > 0.) {
      mDirectMagnetizingHistory(freq, 0) =
          mDirectMagnetizingAdmittance(freq, 0) *
              mDirectMagnetizingVoltage(freq, 0) +
          mDirectMagnetizingPrevCurrentFactor(freq, 0) *
              mDirectMagnetizingCurrent(freq, 0);
      midpointHistory += mDirectMagnetizingHistory(freq, 0);
    }
    Math::setVectorElement(rightVector, midpoint, midpointHistory, mNumFreqs,
                           freq);
  }
}

void DP::Ph1::Transformer::mnaParentAddPreStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes) {
  prevStepDependencies.push_back(mIntfCurrent);
  prevStepDependencies.push_back(mIntfVoltage);
  modifiedAttributes.push_back(mRightVector);
}

void DP::Ph1::Transformer::mnaParentPreStep(Real time, Int timeStepCount) {
  this->mnaApplyRightSideVectorStamp(**this->mRightVector);
}

void DP::Ph1::Transformer::mnaParentAddPostStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes,
    Attribute<Matrix>::Ptr &leftVector) {
  attributeDependencies.push_back(leftVector);
  modifiedAttributes.push_back(mIntfVoltage);
  modifiedAttributes.push_back(mIntfCurrent);
}

void DP::Ph1::Transformer::mnaParentPostStep(
    Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
  this->mnaUpdateVoltage(**leftVector);
  this->mnaUpdateCurrent(**leftVector);
}

void DP::Ph1::Transformer::mnaCompUpdateCurrent(const Matrix &leftVector) {
  if (mUseSubcomponents) {
    (**mIntfCurrent)(0, 0) =
        mOrientationSign * mSubInductor->intfCurrent()(0, 0);
    return;
  }
  const UInt midpoint = mVirtualNodes[2]->matrixNodeIndex();
  const UInt firstNode = mNumVirtualNodes == 5
                             ? mVirtualNodes[3]->matrixNodeIndex()
                             : matrixNodeIndex(mReferenceTerminal);
  const auto voltage = [&](UInt row) {
    return Math::complexFromVectorElement(leftVector, row);
  };
  mDirectInductorVoltage0(0, 0) = voltage(midpoint) - voltage(firstNode);
  mDirectInductorVoltage1(0, 0) =
      voltage(mVirtualNodes[0]->matrixNodeIndex()) -
      (mNumVirtualNodes == 5 ? voltage(mVirtualNodes[4]->matrixNodeIndex())
                             : voltage(midpoint));
  mDirectInductorCurrent0(0, 0) =
      mDirectInductorAdmittance(0, 0) * mDirectInductorVoltage0(0, 0) +
      mDirectInductorHistory0(0, 0);
  mDirectInductorCurrent1(0, 0) =
      mDirectInductorAdmittance(0, 0) * mDirectInductorVoltage1(0, 0) +
      mDirectInductorHistory1(0, 0);
  mDirectMagnetizingVoltage(0, 0) = -voltage(midpoint);
  if (**mRatedPower > 0.)
    mDirectMagnetizingCurrent(0, 0) =
        mDirectMagnetizingAdmittance(0, 0) * mDirectMagnetizingVoltage(0, 0) +
        mDirectMagnetizingHistory(0, 0);
  (**mIntfCurrent)(0, 0) = mOrientationSign * mDirectInductorCurrent0(0, 0);
}

void DP::Ph1::Transformer::mnaCompUpdateVoltage(const Matrix &leftVector) {
  // v1 - v0
  (**mIntfVoltage)(0, 0) = 0;
  if (terminalNotGrounded(1))
    (**mIntfVoltage)(0, 0) =
        Math::complexFromVectorElement(leftVector, matrixNodeIndex(1));
  if (terminalNotGrounded(0))
    (**mIntfVoltage)(0, 0) =
        (**mIntfVoltage)(0, 0) -
        Math::complexFromVectorElement(leftVector, matrixNodeIndex(0));

  SPDLOG_LOGGER_DEBUG(mSLog, "Voltage {:s}",
                      Logger::phasorToString((**mIntfVoltage)(0, 0)));
}
