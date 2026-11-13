// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <dpsim-models/DP/DP_Ph3_Transformer.h>
#include <dpsim-models/MNAStampUtils.h>

using namespace CPS;

DP::Ph3::Transformer::Transformer(String uid, String name,
                                  Logger::Level logLevel,
                                  Bool withResistiveLosses)
    : Transformer(uid, name, true, logLevel, withResistiveLosses) {}

DP::Ph3::Transformer::Transformer(String uid, String name,
                                  Bool useSubcomponents, Logger::Level logLevel,
                                  Bool withResistiveLosses)
    : Base::Ph3::Transformer(mAttributes),
      CompositePowerComp<Complex>(uid, name, true, true, logLevel),
      mUseSubcomponents(useSubcomponents) {
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

SimPowerComp<Complex>::Ptr DP::Ph3::Transformer::clone(String name) {
  auto copy = Transformer::make(name, mUseSubcomponents, mLogLevel,
                                mNumVirtualNodes == 5);
  copy->setParameters(mNominalVoltagePrimary, mNominalVoltageSecondary,
                      mRatedPower, std::abs(**mRatio), std::arg(**mRatio),
                      mResistance, mInductance);
  copy->setTapRatio(tapRatio());
  return copy;
}

void DP::Ph3::Transformer::setParameters(Real nomVoltagePrimary,
                                         Real nomVoltageSecondary,
                                         Real ratedPower, Real ratioAbs,
                                         Real ratioPhase, Matrix resistance,
                                         Matrix inductance) {
  Base::Ph3::Transformer::setParameters(nomVoltagePrimary, nomVoltageSecondary,
                                        ratedPower, ratioAbs, ratioPhase,
                                        resistance, inductance);
  mParametersSet = true;
}

void DP::Ph3::Transformer::resolveWindingRoles() {
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

  SPDLOG_LOGGER_INFO(
      mSLog,
      "Impedance referred to winding {} ({} [V]), remaining winding "
      "at terminal {} ({} [V]), ratio = {}",
      mReferenceTerminal + 1, nominalVoltageAt(mReferenceTerminal),
      nonReferenceTerminal(), nominalVoltageAt(nonReferenceTerminal()),
      Logger::complexToString(mRatioFromReference));
}

void DP::Ph3::Transformer::createSubComponents() {
  if (!mUseSubcomponents)
    return;
  if (mSubCompCreated)
    return;
  mSubCompCreated = true;

  resolveWindingRoles();

  auto midpoint = mVirtualNodes[2];

  mSubInductor =
      std::make_shared<DP::Ph3::Inductor>(**mName + "_ind", mLogLevel);
  mSubInductor->setParameters(mInductance / 2.);
  addMNASubComponent(mSubInductor, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);

  mSubInductor2 =
      std::make_shared<DP::Ph3::Inductor>(**mName + "_ind2", mLogLevel);
  mSubInductor2->setParameters(mInductance / 2.);
  addMNASubComponent(mSubInductor2, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);

  if (mNumVirtualNodes == 5) {
    mSubResistor =
        std::make_shared<DP::Ph3::Resistor>(**mName + "_res", mLogLevel);
    mSubResistor->setParameters(mResistance / 2.);
    mSubResistor->connect({node(mReferenceTerminal), mVirtualNodes[3]});
    mSubInductor->connect({mVirtualNodes[3], midpoint});
    addMNASubComponent(mSubResistor, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                       MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);

    mSubResistor2 =
        std::make_shared<DP::Ph3::Resistor>(**mName + "_res2", mLogLevel);
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
      std::make_shared<DP::Ph3::Resistor>(**mName + "_mag_res", mLogLevel);
  mSubMagnetizingResistor->setParameters(mMagnetizingResistance);
  mSubMagnetizingResistor->connect({midpoint, DP::SimNode::GND});
  addMNASubComponent(mSubMagnetizingResistor,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);

  mSubMagnetizingInductor =
      std::make_shared<DP::Ph3::Inductor>(**mName + "_mag_ind", mLogLevel);
  mSubMagnetizingInductor->connect({midpoint, DP::SimNode::GND});
  addMNASubComponent(mSubMagnetizingInductor,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);
}

void DP::Ph3::Transformer::initializeParentFromNodesAndTerminals(
    Real frequency) {
  Real omega = 2. * PI * frequency;
  if (!mUseSubcomponents)
    resolveWindingRoles();

  // Keep the stored PF phasors on virtual nodes in RMS form so that the
  // attached primitive components can apply their usual PF-to-DP conversion
  // exactly once. Scale only the initialized node state itself.
  for (auto &virtualNode : mVirtualNodes)
    virtualNode->setComplexInitialVoltageScale(RMS3PH_TO_PEAK1PH);

  if (mUseSubcomponents ? static_cast<Bool>(mSubMagnetizingInductor)
                        : mRatedPower > 0.) {
    if (!mUseSubcomponents) {
      if (mNoLoadCurrent <= mNoLoadLoss)
        throw InvalidArgumentException();
      const Real resistance =
          std::pow(nominalVoltageAt(mReferenceTerminal), 2) /
          (mNoLoadLoss * mRatedPower);
      mMagnetizingResistance =
          Math::singlePhaseParameterToThreePhase(resistance);
    }
    Real magnetizingSusceptance =
        std::sqrt(std::pow(mNoLoadCurrent, 2) - std::pow(mNoLoadLoss, 2)) *
        mRatedPower / std::pow(nominalVoltageAt(mReferenceTerminal), 2);
    Real magnetizingInductance = 1. / (omega * magnetizingSusceptance);
    mMagnetizingInductance =
        Math::singlePhaseParameterToThreePhase(magnetizingInductance);
    if (mUseSubcomponents)
      mSubMagnetizingInductor->setParameters(mMagnetizingInductance);
  }

  // Power-flow voltages are line-line RMS phasors. Keep these initial values
  // raw for child components; their initializers convert their own states.
  MatrixComp secondaryVoltage = initialVoltage(nonReferenceTerminal());
  mVirtualNodes[0]->setInitialVoltage(secondaryVoltage * mRatioFromReference);

  Complex halfImpedance =
      Complex(mResistance(0, 0) / 2., omega * mInductance(0, 0) / 2.);
  Complex magnetizingAdmittance = Complex(0, 0);
  if (mUseSubcomponents ? static_cast<Bool>(mSubMagnetizingInductor)
                        : mRatedPower > 0.)
    magnetizingAdmittance =
        1. / mMagnetizingResistance(0, 0) +
        1. / Complex(0, omega * mMagnetizingInductance(0, 0));

  MatrixComp referenceVoltage = initialVoltage(mReferenceTerminal);
  MatrixComp midpointVoltage =
      (referenceVoltage + mVirtualNodes[0]->initialVoltage()) /
      (2. + halfImpedance * magnetizingAdmittance);
  mVirtualNodes[2]->setInitialVoltage(midpointVoltage);

  if (mNumVirtualNodes == 5) {
    MatrixComp referenceCurrent =
        (referenceVoltage - midpointVoltage) / halfImpedance;
    MatrixComp nonReferenceCurrent =
        (midpointVoltage - mVirtualNodes[0]->initialVoltage()) / halfImpedance;
    mVirtualNodes[3]->setInitialVoltage(
        referenceVoltage - referenceCurrent * mResistance(0, 0) / 2.);
    mVirtualNodes[4]->setInitialVoltage(
        midpointVoltage - nonReferenceCurrent * mResistance(0, 0) / 2.);
  }

  **mIntfVoltage = RMS3PH_TO_PEAK1PH * (initialVoltage(1) - initialVoltage(0));

  MatrixComp impedance = MatrixComp::Zero(3, 3);
  for (UInt row = 0; row < 3; row++)
    for (UInt col = 0; col < 3; col++)
      impedance(row, col) =
          Complex(mResistance(row, col), omega * mInductance(row, col));

  MatrixComp impedanceVoltage =
      mOrientationSign * RMS3PH_TO_PEAK1PH *
      (mVirtualNodes[0]->initialVoltage() - referenceVoltage);
  **mIntfCurrent = impedance.inverse() * impedanceVoltage;
}

void DP::Ph3::Transformer::mnaParentInitialize(
    Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
  if (mUseSubcomponents)
    return;
  updateMatrixNodeIndices();
  const Real b = timeStep * omega / 2.;
  const Matrix halfInductance = mInductance / 2.;
  const Matrix a = (timeStep / 2.) * halfInductance.inverse();
  mDirectInductorAdmittance = MatrixComp::Zero(3, 3);
  mDirectInductorAdmittance.real() = a / (1. + b * b);
  mDirectInductorAdmittance.imag() = -a * b / (1. + b * b);
  mDirectInductorPrevCurrentFactor =
      Complex((1. - b * b) / (1. + b * b), -2. * b / (1. + b * b));
  mDirectInductorCurrent0.setZero();
  mDirectInductorCurrent1.setZero();
  mDirectInductorVoltage0.setZero();
  mDirectInductorVoltage1.setZero();
  mDirectInductorHistory0.setZero();
  mDirectInductorHistory1.setZero();
  mDirectMagnetizingAdmittance.setZero();
  mDirectMagnetizingCurrent.setZero();
  mDirectMagnetizingVoltage.setZero();
  mDirectMagnetizingHistory.setZero();
  if (mRatedPower > 0.) {
    const Matrix magA = (timeStep / 2.) * mMagnetizingInductance.inverse();
    mDirectMagnetizingAdmittance.real() = magA / (1. + b * b);
    mDirectMagnetizingAdmittance.imag() = -magA * b / (1. + b * b);
    mDirectMagnetizingPrevCurrentFactor = mDirectInductorPrevCurrentFactor;
  }

  const MatrixComp midpoint = mVirtualNodes[2]->initialVoltage();
  const MatrixComp ratioNode = mVirtualNodes[0]->initialVoltage();
  const MatrixComp firstNode = mNumVirtualNodes == 5
                                   ? mVirtualNodes[3]->initialVoltage()
                                   : initialVoltage(mReferenceTerminal);
  const MatrixComp secondNode =
      mNumVirtualNodes == 5 ? mVirtualNodes[4]->initialVoltage() : midpoint;
  // Virtual-node initial phasors are stored in RMS form. Match the primitive
  // DP inductor initializers by converting them to phase-peak envelopes here.
  mDirectInductorVoltage0 = RMS3PH_TO_PEAK1PH * (midpoint - firstNode);
  mDirectInductorVoltage1 = RMS3PH_TO_PEAK1PH * (ratioNode - secondNode);
  mDirectInductorCurrent0 =
      (Complex(0., omega) * halfInductance.cast<Complex>()).inverse() *
      mDirectInductorVoltage0;
  mDirectInductorCurrent1 =
      (Complex(0., omega) * halfInductance.cast<Complex>()).inverse() *
      mDirectInductorVoltage1;
  if (mRatedPower > 0.) {
    mDirectMagnetizingVoltage = -RMS3PH_TO_PEAK1PH * midpoint;
    mDirectMagnetizingCurrent =
        (Complex(0., omega) * mMagnetizingInductance.cast<Complex>())
            .inverse() *
        mDirectMagnetizingVoltage;
  }
}

void DP::Ph3::Transformer::mnaParentApplySystemMatrixStamp(
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
  if (!mUseSubcomponents) {
    const UInt midpoint = mVirtualNodes[2]->matrixNodeIndex(PhaseType::A);
    const UInt ratioNode = mVirtualNodes[0]->matrixNodeIndex(PhaseType::A);
    const UInt firstNode = mNumVirtualNodes == 5
                               ? mVirtualNodes[3]->matrixNodeIndex(PhaseType::A)
                               : matrixNodeIndex(mReferenceTerminal, 0);
    const UInt secondNode =
        mNumVirtualNodes == 5 ? mVirtualNodes[4]->matrixNodeIndex(PhaseType::A)
                              : midpoint;
    if (mNumVirtualNodes == 5) {
      const MatrixComp resistorY = (mResistance / 2.).inverse().cast<Complex>();
      MNAStampUtils::stampAdmittanceMatrix(
          resistorY, systemMatrix, matrixNodeIndex(mReferenceTerminal, 0),
          firstNode, terminalNotGrounded(mReferenceTerminal), true, mSLog);
      MNAStampUtils::stampAdmittanceMatrix(resistorY, systemMatrix, midpoint,
                                           secondNode, true, true, mSLog);
    }
    MNAStampUtils::stampAdmittanceMatrix(mDirectInductorAdmittance,
                                         systemMatrix, firstNode, midpoint,
                                         true, true, mSLog);
    MNAStampUtils::stampAdmittanceMatrix(mDirectInductorAdmittance,
                                         systemMatrix, secondNode, ratioNode,
                                         true, true, mSLog);
    if (mRatedPower > 0.) {
      MNAStampUtils::stampAdmittanceMatrix(
          mMagnetizingResistance.inverse().cast<Complex>(), systemMatrix,
          midpoint, 0, true, false, mSLog);
      MNAStampUtils::stampAdmittanceMatrix(mDirectMagnetizingAdmittance,
                                           systemMatrix, midpoint, 0, true,
                                           false, mSLog);
    }
  }
}

void DP::Ph3::Transformer::mnaParentApplyRightSideVectorStamp(
    Matrix &rightVector) {
  if (mUseSubcomponents)
    return;
  const UInt midpoint = mVirtualNodes[2]->matrixNodeIndex(PhaseType::A);
  const UInt ratioNode = mVirtualNodes[0]->matrixNodeIndex(PhaseType::A);
  const UInt firstNode = mNumVirtualNodes == 5
                             ? mVirtualNodes[3]->matrixNodeIndex(PhaseType::A)
                             : matrixNodeIndex(mReferenceTerminal, 0);
  const UInt secondNode = mNumVirtualNodes == 5
                              ? mVirtualNodes[4]->matrixNodeIndex(PhaseType::A)
                              : midpoint;
  mDirectInductorHistory0 =
      mDirectInductorAdmittance * mDirectInductorVoltage0 +
      mDirectInductorPrevCurrentFactor * mDirectInductorCurrent0;
  mDirectInductorHistory1 =
      mDirectInductorAdmittance * mDirectInductorVoltage1 +
      mDirectInductorPrevCurrentFactor * mDirectInductorCurrent1;
  if (mRatedPower > 0.)
    mDirectMagnetizingHistory =
        mDirectMagnetizingAdmittance * mDirectMagnetizingVoltage +
        mDirectMagnetizingPrevCurrentFactor * mDirectMagnetizingCurrent;
  for (UInt phase = 0; phase < 3; ++phase) {
    if (mNumVirtualNodes == 5 || terminalNotGrounded(mReferenceTerminal))
      Math::setVectorElement(rightVector, firstNode + phase,
                             mDirectInductorHistory0(phase, 0));
    Complex midpointHistory = -mDirectInductorHistory0(phase, 0);
    if (mNumVirtualNodes == 5)
      Math::setVectorElement(rightVector, secondNode + phase,
                             mDirectInductorHistory1(phase, 0));
    else
      midpointHistory += mDirectInductorHistory1(phase, 0);
    if (mRatedPower > 0.)
      midpointHistory += mDirectMagnetizingHistory(phase, 0);
    Math::setVectorElement(rightVector, midpoint + phase, midpointHistory);
    Math::setVectorElement(rightVector, ratioNode + phase,
                           -mDirectInductorHistory1(phase, 0));
  }
}

void DP::Ph3::Transformer::mnaParentAddPreStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes) {
  prevStepDependencies.push_back(mIntfCurrent);
  prevStepDependencies.push_back(mIntfVoltage);
  modifiedAttributes.push_back(mRightVector);
}

void DP::Ph3::Transformer::mnaParentPreStep(Real time, Int timeStepCount) {
  this->mnaApplyRightSideVectorStamp(**this->mRightVector);
}

void DP::Ph3::Transformer::mnaParentAddPostStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes,
    Attribute<Matrix>::Ptr &leftVector) {
  attributeDependencies.push_back(leftVector);
  modifiedAttributes.push_back(mIntfVoltage);
  modifiedAttributes.push_back(mIntfCurrent);
}

void DP::Ph3::Transformer::mnaParentPostStep(
    Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
  mnaCompUpdateVoltage(**leftVector);
  mnaCompUpdateCurrent(**leftVector);
}

void DP::Ph3::Transformer::mnaCompUpdateVoltage(const Matrix &leftVector) {
  (**mIntfVoltage).setZero();
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

void DP::Ph3::Transformer::mnaCompUpdateCurrent(const Matrix &leftVector) {
  if (mUseSubcomponents) {
    **mIntfCurrent = mOrientationSign * mSubInductor->intfCurrent();
    return;
  }
  const UInt midpoint = mVirtualNodes[2]->matrixNodeIndex(PhaseType::A);
  const UInt firstNode = mNumVirtualNodes == 5
                             ? mVirtualNodes[3]->matrixNodeIndex(PhaseType::A)
                             : matrixNodeIndex(mReferenceTerminal, 0);
  const UInt secondNode = mNumVirtualNodes == 5
                              ? mVirtualNodes[4]->matrixNodeIndex(PhaseType::A)
                              : midpoint;
  for (UInt phase = 0; phase < 3; ++phase) {
    mDirectInductorVoltage0(phase, 0) =
        Math::complexFromVectorElement(leftVector, midpoint + phase) -
        (mNumVirtualNodes == 5 || terminalNotGrounded(mReferenceTerminal)
             ? Math::complexFromVectorElement(leftVector, firstNode + phase)
             : Complex(0., 0.));
    mDirectInductorVoltage1(phase, 0) =
        Math::complexFromVectorElement(
            leftVector,
            mVirtualNodes[0]->matrixNodeIndex(PhaseType::A) + phase) -
        Math::complexFromVectorElement(leftVector, secondNode + phase);
    mDirectMagnetizingVoltage(phase, 0) =
        -Math::complexFromVectorElement(leftVector, midpoint + phase);
  }
  mDirectInductorCurrent0 =
      mDirectInductorAdmittance * mDirectInductorVoltage0 +
      mDirectInductorHistory0;
  mDirectInductorCurrent1 =
      mDirectInductorAdmittance * mDirectInductorVoltage1 +
      mDirectInductorHistory1;
  if (mRatedPower > 0.)
    mDirectMagnetizingCurrent =
        mDirectMagnetizingAdmittance * mDirectMagnetizingVoltage +
        mDirectMagnetizingHistory;
  **mIntfCurrent = mOrientationSign * mDirectInductorCurrent0;
}
