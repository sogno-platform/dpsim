/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/MathUtils.h>
#include <dpsim-models/SP/SP_Ph1_Transformer.h>

using namespace CPS;

// #### General ####
SP::Ph1::Transformer::Transformer(String uid, String name,
                                  Logger::Level logLevel,
                                  Bool withResistiveLosses)
    : Base::Ph1::Transformer(mAttributes),
      CompositePowerComp<Complex>(uid, name, true, true, logLevel),
      mBaseVoltage(mAttributes->create<Real>("base_Voltage")),
      mCurrent(mAttributes->create<MatrixComp>("current_vector")),
      mActivePowerBranch(mAttributes->create<Matrix>("p_branch_vector")),
      mReactivePowerBranch(mAttributes->create<Matrix>("q_branch_vector")),
      mActivePowerInjection(mAttributes->create<Real>("p_inj")),
      mReactivePowerInjection(mAttributes->create<Real>("q_inj")) {
  if (withResistiveLosses)
    setVirtualNodeNumber(5);
  else
    setVirtualNodeNumber(3);

  SPDLOG_LOGGER_INFO(mSLog, "Create {} {}", this->type(), name);
  **mIntfVoltage = MatrixComp::Zero(1, 1);
  **mIntfCurrent = MatrixComp::Zero(1, 1);
  setTerminalNumber(2);

  **mCurrent = MatrixComp::Zero(2, 1);
  **mActivePowerBranch = Matrix::Zero(2, 1);
  **mReactivePowerBranch = Matrix::Zero(2, 1);
}

void SP::Ph1::Transformer::setParameters(Real nomVoltagePrimary,
                                         Real nomVoltageSecondary,
                                         Real ratioAbs, Real ratioPhase,
                                         Real resistance, Real inductance) {

  // Note: to be consistent impedance values must be referred to high voltage side (and base voltage set to higher voltage)
  Base::Ph1::Transformer::setParameters(nomVoltagePrimary, nomVoltageSecondary,
                                        ratioAbs, ratioPhase, resistance,
                                        inductance);

  SPDLOG_LOGGER_INFO(
      mSLog, "Nominal Voltage Primary={} [V] Nominal Voltage Secondary={} [V]",
      mNominalVoltagePrimary, mNominalVoltageSecondary);
  SPDLOG_LOGGER_INFO(
      mSLog, "Resistance={} [Ohm] Inductance={} [H] (referred to primary side)",
      **mResistance, **mInductance);
  SPDLOG_LOGGER_INFO(mSLog, "Tap Ratio={} [/] Phase Shift={} [deg]",
                     std::abs(**mRatio), std::arg(**mRatio));
  SPDLOG_LOGGER_INFO(mSLog, "Rated Power ={} [VA]", **mRatedPower);

  mRatioAbs = std::abs(**mRatio);
  mRatioPhase = std::arg(**mRatio);

  mParametersSet = true;
}

void SP::Ph1::Transformer::setParameters(Real nomVoltagePrimary,
                                         Real nomVoltageSecondary,
                                         Real ratedPower, Real ratioAbs,
                                         Real ratioPhase, Real resistance,
                                         Real inductance) {

  // Rated power is the nameplate apparent-power magnitude |S|, so it cannot be
  // negative (a negative value is a caller error, not the unset default of 0).
  if (ratedPower < 0) {
    SPDLOG_LOGGER_ERROR(mSLog, "Rated power {} [VA] is negative; must be >= 0",
                        ratedPower);
    throw InvalidArgumentException();
  }

  **mRatedPower = ratedPower;
  SPDLOG_LOGGER_INFO(mSLog, "Rated Power ={} [VA]", **mRatedPower);

  SP::Ph1::Transformer::setParameters(nomVoltagePrimary, nomVoltageSecondary,
                                      ratioAbs, ratioPhase, resistance,
                                      inductance);
}

/// DEPRECATED: Delete method
SimPowerComp<Complex>::Ptr SP::Ph1::Transformer::clone(String name) {
  auto copy = Transformer::make(name, mLogLevel);
  copy->setParameters(mNominalVoltagePrimary, mNominalVoltageSecondary,
                      **mRatedPower, std::abs(**mRatio), std::arg(**mRatio),
                      **mResistance, **mInductance);
  return copy;
}

void SP::Ph1::Transformer::resolveWindingRoles() {
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

  if ((mReferenceTerminal == 0) != (Math::abs(**mRatio) >= 1.) &&
      Math::abs(Math::abs(**mRatio) - 1.) > 1e-9)
    SPDLOG_LOGGER_WARN(
        mSLog,
        "Nominal voltages put the higher-voltage winding at terminal {} ({} "
        "[V] against {} [V]) but the turns ratio {} points the other way; "
        "check the argument order of setParameters()",
        mReferenceTerminal, nominalVoltageAt(mReferenceTerminal),
        nominalVoltageAt(nonReferenceTerminal()),
        Logger::complexToString(**mRatio));
}

void SP::Ph1::Transformer::createSubComponents() {
  if (mSubCompCreated)
    return;
  mSubCompCreated = true;

  resolveWindingRoles();

  auto midpoint = mVirtualNodes[2];

  mSubInductor = std::make_shared<SP::Ph1::Inductor>(
      **mUID + "_ind", **mName + "_ind", Logger::Level::off);
  mSubInductor->setParameters(**mInductance / 2.);
  addMNASubComponent(mSubInductor, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);

  mSubInductor2 = std::make_shared<SP::Ph1::Inductor>(
      **mUID + "_ind2", **mName + "_ind2", Logger::Level::off);
  mSubInductor2->setParameters(**mInductance / 2.);
  addMNASubComponent(mSubInductor2, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);

  if (mNumVirtualNodes == 5) {
    mSubResistor = std::make_shared<SP::Ph1::Resistor>(
        **mUID + "_res", **mName + "_res", Logger::Level::off);
    mSubResistor->setParameters(**mResistance / 2.);
    mSubResistor->connect({node(mReferenceTerminal), mVirtualNodes[3]});
    mSubInductor->connect({mVirtualNodes[3], midpoint});
    addMNASubComponent(mSubResistor, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                       MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);

    mSubResistor2 = std::make_shared<SP::Ph1::Resistor>(
        **mUID + "_res2", **mName + "_res2", Logger::Level::off);
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

  bool magnetizingEnabled =
      (mBehaviour == TopologicalPowerComp::Behaviour::Initialization ||
       mBehaviour == TopologicalPowerComp::Behaviour::MNASimulation);
  if (!magnetizingEnabled)
    return;

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
                        "no-load loss {}",
                        this->name(), mNoLoadCurrent, mNoLoadLoss);
    throw InvalidArgumentException();
  }

  mSubMagnetizingResistor = std::make_shared<SP::Ph1::Resistor>(
      **mUID + "_mag_res", **mName + "_mag_res", Logger::Level::off);
  mSubMagnetizingResistor->connect({midpoint, SP::SimNode::GND});
  addMNASubComponent(mSubMagnetizingResistor,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);

  mSubMagnetizingInductor = std::make_shared<SP::Ph1::Inductor>(
      **mUID + "_mag_ind", **mName + "_mag_ind", Logger::Level::off);
  mSubMagnetizingInductor->connect({midpoint, SP::SimNode::GND});
  addMNASubComponent(mSubMagnetizingInductor,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);
}

void SP::Ph1::Transformer::initializeParentFromNodesAndTerminals(
    Real frequency) {
  mNominalOmega = 2. * PI * frequency;
  mReactance = mNominalOmega * **mInductance;
  SPDLOG_LOGGER_INFO(mSLog, "Reactance={} [Ohm] (referred to primary side)",
                     mReactance);

  if (mSubMagnetizingResistor) {
    mMagnetizingResistance = std::pow(nominalVoltageAt(mReferenceTerminal), 2) /
                             (mNoLoadLoss * **mRatedPower);
    mSubMagnetizingResistor->setParameters(mMagnetizingResistance);
    mSubMagnetizingResistor->setBaseVoltage(
        nominalVoltageAt(mReferenceTerminal));

    Real magnetizingSusceptance =
        std::sqrt(std::pow(mNoLoadCurrent, 2) - std::pow(mNoLoadLoss, 2)) *
        **mRatedPower / std::pow(nominalVoltageAt(mReferenceTerminal), 2);
    mMagnetizingInductance = 1. / (mNominalOmega * magnetizingSusceptance);
    mSubMagnetizingInductor->setParameters(mMagnetizingInductance);

    SPDLOG_LOGGER_INFO(mSLog,
                       "Magnetizing resistance = {} [Ohm], inductance = {} [H]",
                       Logger::realToString(mMagnetizingResistance),
                       Logger::realToString(mMagnetizingInductance));
  }

  // Set initial voltage of virtual node in between
  mVirtualNodes[0]->setInitialVoltage(
      initialSingleVoltage(nonReferenceTerminal()) * mRatioFromReference);

  // Static calculations from load flow data
  Complex impedance = {**mResistance, mReactance};
  Complex impedanceVoltage =
      mOrientationSign * (mVirtualNodes[0]->initialSingleVoltage() -
                          initialSingleVoltage(mReferenceTerminal));
  (**mIntfVoltage)(0, 0) = initialSingleVoltage(1) - initialSingleVoltage(0);
  (**mIntfCurrent)(0, 0) = impedanceVoltage / impedance;

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
      Logger::phasorToString((**mIntfVoltage)(0, 0)),
      Logger::phasorToString((**mIntfCurrent)(0, 0)),
      Logger::phasorToString(initialSingleVoltage(0)),
      Logger::phasorToString(initialSingleVoltage(1)),
      Logger::phasorToString(mVirtualNodes[0]->initialSingleVoltage()));
}

// #### Powerflow section ####

Real SP::Ph1::Transformer::getNominalVoltagePrimary() const {
  return mNominalVoltagePrimary;
}

Real SP::Ph1::Transformer::getNominalVoltageSecondary() const {
  return mNominalVoltageSecondary;
}

void SP::Ph1::Transformer::setBaseVoltage(Real baseVoltage) {
  // Note: to be consistent set base voltage to higher voltage (and impedance values must be referred to high voltage side)
  // TODO: use attribute setter for setting base voltage
  **mBaseVoltage = baseVoltage;
}

void SP::Ph1::Transformer::calculatePerUnitParameters(Real baseApparentPower,
                                                      Real baseOmega) {
  SPDLOG_LOGGER_INFO(mSLog, "#### Calculate Per Unit Parameters for {}",
                     **mName);
  mBaseApparentPower = baseApparentPower;
  mBaseOmega = baseOmega;
  SPDLOG_LOGGER_INFO(mSLog, "Base Power={} [VA]  Base Omega={} [1/s]",
                     baseApparentPower, baseOmega);

  resolveWindingRoles();

  Real referenceVoltage = nominalVoltageAt(mReferenceTerminal);
  if (**mBaseVoltage <= 0) {
    SPDLOG_LOGGER_INFO(mSLog,
                       "Transformer {}: no base voltage was set, using the "
                       "reference winding {} [V]",
                       this->name(), referenceVoltage);
    **mBaseVoltage = referenceVoltage;
  } else if (std::abs(**mBaseVoltage - referenceVoltage) >
             DOUBLE_EPSILON * referenceVoltage) {
    SPDLOG_LOGGER_WARN(mSLog,
                       "Transformer {}: base voltage {} [V] does not match the "
                       "reference winding {} [V]; the impedances are referred "
                       "to the reference winding, so that is used as the base",
                       this->name(), **mBaseVoltage, referenceVoltage);
    **mBaseVoltage = referenceVoltage;
  }

  mBaseImpedance = **mBaseVoltage * **mBaseVoltage / mBaseApparentPower;
  mBaseAdmittance = 1.0 / mBaseImpedance;
  mBaseCurrent = baseApparentPower /
                 (**mBaseVoltage *
                  sqrt(3)); // I_base=(S_threephase/3)/(V_line_to_line/sqrt(3))
  SPDLOG_LOGGER_INFO(mSLog, "Base Voltage={} [V]  Base Impedance={} [Ohm]",
                     **mBaseVoltage, mBaseImpedance);

  mResistancePerUnit = **mResistance / mBaseImpedance;
  mReactancePerUnit = mReactance / mBaseImpedance;
  SPDLOG_LOGGER_INFO(mSLog, "Resistance={} [pu]  Reactance={} [pu]",
                     mResistancePerUnit, mReactancePerUnit);

  mBaseInductance = mBaseImpedance / mBaseOmega;
  mInductancePerUnit = **mInductance / mBaseInductance;
  // omega per unit=1, hence 1.0*mInductancePerUnit.
  mLeakagePerUnit = Complex(mResistancePerUnit, 1. * mInductancePerUnit);
  SPDLOG_LOGGER_INFO(mSLog, "Leakage Impedance={} [pu] ", mLeakagePerUnit);

  mRatioAbsPerUnit =
      mRatioAbs / mNominalVoltagePrimary * mNominalVoltageSecondary;
  mRatioPerUnit = mRatioFromReference /
                  Complex(nominalVoltageAt(mReferenceTerminal) /
                              nominalVoltageAt(nonReferenceTerminal()),
                          0.);
  SPDLOG_LOGGER_INFO(mSLog, "Tap Ratio={} [pu]", mRatioAbsPerUnit);

  // Calculate per unit parameters of subcomps
  if (mSubMagnetizingResistor && **mRatedPower > 0 &&
      mNoLoadCurrent > mNoLoadLoss)
    mMagnetizingPerUnit =
        Complex(mNoLoadLoss, -std::sqrt(std::pow(mNoLoadCurrent, 2) -
                                        std::pow(mNoLoadLoss, 2))) *
        **mRatedPower / std::pow(nominalVoltageAt(mReferenceTerminal), 2) *
        mBaseImpedance;
  else
    mMagnetizingPerUnit = Complex(0, 0);
}

void SP::Ph1::Transformer::pfApplyAdmittanceMatrixStamp(
    SparseMatrixCompRow &Y) {
  // calculate matrix stamp
  mY_element = MatrixComp(2, 2);
  Complex halfLeakage = mLeakagePerUnit / 2.;
  Complex determinant =
      mLeakagePerUnit + halfLeakage * halfLeakage * mMagnetizingPerUnit;
  Complex yShunted = (1. + halfLeakage * mMagnetizingPerUnit) / determinant;
  Complex ySeries = 1. / determinant;

  mY_element(0, 0) = yShunted;
  mY_element(0, 1) = -ySeries * mRatioPerUnit;
  mY_element(1, 0) = -ySeries * std::conj(mRatioPerUnit);
  mY_element(1, 1) = yShunted * std::norm(mRatioPerUnit);

  //check for inf or nan
  for (int i = 0; i < 2; i++)
    for (int j = 0; j < 2; j++)
      if (!Math::isFinite(mY_element.coeff(i, j))) {
        SPDLOG_LOGGER_ERROR(
            mSLog,
            "Transformer {}: non-finite per-unit admittance {} "
            "in element Y({},{}) (leakage {}, tap {})",
            this->name(), Logger::complexToString(mY_element.coeff(i, j)), i, j,
            Logger::complexToString(mLeakage), mRatioAbsPerUnit);
        throw InvalidArgumentException();
      }

  //set the circuit matrix values
  Y.coeffRef(this->matrixNodeIndex(mReferenceTerminal),
             this->matrixNodeIndex(mReferenceTerminal)) +=
      mY_element.coeff(0, 0);
  Y.coeffRef(this->matrixNodeIndex(mReferenceTerminal),
             this->matrixNodeIndex(nonReferenceTerminal())) +=
      mY_element.coeff(0, 1);
  Y.coeffRef(this->matrixNodeIndex(nonReferenceTerminal()),
             this->matrixNodeIndex(nonReferenceTerminal())) +=
      mY_element.coeff(1, 1);
  Y.coeffRef(this->matrixNodeIndex(nonReferenceTerminal()),
             this->matrixNodeIndex(mReferenceTerminal)) +=
      mY_element.coeff(1, 0);

  SPDLOG_LOGGER_INFO(mSLog, "#### Y matrix stamping: {}", mY_element);
}

void SP::Ph1::Transformer::updateBranchFlow(VectorComp &current,
                                            VectorComp &powerflow) {
  **mCurrent = current * mBaseCurrent;
  **mActivePowerBranch = powerflow.real() * mBaseApparentPower;
  **mReactivePowerBranch = powerflow.imag() * mBaseApparentPower;
}

void SP::Ph1::Transformer::storeNodalInjection(Complex powerInjection) {
  **mActivePowerInjection = std::real(powerInjection) * mBaseApparentPower;
  **mReactivePowerInjection = std::imag(powerInjection) * mBaseApparentPower;
}

MatrixComp SP::Ph1::Transformer::Y_element() { return mY_element; }

// #### MNA Section ####

void SP::Ph1::Transformer::mnaParentInitialize(
    Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
  SPDLOG_LOGGER_INFO(
      mSLog,
      "\nTerminal 0 connected to {:s} = sim node {:d}"
      "\nTerminal 1 connected to {:s} = sim node {:d}",
      mTerminals[0]->node()->name(), mTerminals[0]->node()->matrixNodeIndex(),
      mTerminals[1]->node()->name(), mTerminals[1]->node()->matrixNodeIndex());
}

void SP::Ph1::Transformer::mnaCompApplySystemMatrixStamp(
    SparseMatrixRow &systemMatrix) {
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

  if (terminalNotGrounded(0)) {
    SPDLOG_LOGGER_INFO(mSLog, "Add {:s} to system at ({:d},{:d})",
                       Logger::complexToString(Complex(-1.0, 0)),
                       mVirtualNodes[0]->matrixNodeIndex(),
                       mVirtualNodes[1]->matrixNodeIndex());
    SPDLOG_LOGGER_INFO(mSLog, "Add {:s} to system at ({:d},{:d})",
                       Logger::complexToString(Complex(1.0, 0)),
                       mVirtualNodes[1]->matrixNodeIndex(),
                       mVirtualNodes[0]->matrixNodeIndex());
  }
  if (terminalNotGrounded(1)) {
    SPDLOG_LOGGER_INFO(mSLog, "Add {:s} to system at ({:d},{:d})",
                       Logger::complexToString(**mRatio), matrixNodeIndex(1),
                       mVirtualNodes[1]->matrixNodeIndex());
    SPDLOG_LOGGER_INFO(mSLog, "Add {:s} to system at ({:d},{:d})",
                       Logger::complexToString(-**mRatio),
                       mVirtualNodes[1]->matrixNodeIndex(), matrixNodeIndex(1));
  }
}

void SP::Ph1::Transformer::mnaParentAddPreStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes) {
  prevStepDependencies.push_back(mIntfCurrent);
  prevStepDependencies.push_back(mIntfVoltage);
  modifiedAttributes.push_back(mRightVector);
}

void SP::Ph1::Transformer::mnaParentPreStep(Real time, Int timeStepCount) {
  mnaCompApplyRightSideVectorStamp(**mRightVector);
}

void SP::Ph1::Transformer::mnaParentAddPostStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes,
    Attribute<Matrix>::Ptr &leftVector) {
  attributeDependencies.push_back(leftVector);
  modifiedAttributes.push_back(mIntfVoltage);
  modifiedAttributes.push_back(mIntfCurrent);
}

void SP::Ph1::Transformer::mnaParentPostStep(
    Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
  this->mnaUpdateVoltage(**leftVector);
  this->mnaUpdateCurrent(**leftVector);
}

void SP::Ph1::Transformer::mnaCompUpdateCurrent(const Matrix &leftVector) {
  (**mIntfCurrent)(0, 0) = mOrientationSign * mSubInductor->intfCurrent()(0, 0);
  SPDLOG_LOGGER_DEBUG(mSLog, "Current {:s}",
                      Logger::phasorToString((**mIntfCurrent)(0, 0)));
}

void SP::Ph1::Transformer::mnaCompUpdateVoltage(const Matrix &leftVector) {
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
