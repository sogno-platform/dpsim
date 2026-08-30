/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/DP/DP_Ph1_Transformer.h>

using namespace CPS;

DP::Ph1::Transformer::Transformer(String uid, String name,
                                  Logger::Level logLevel,
                                  Bool withResistiveLosses)
    : Base::Ph1::Transformer(mAttributes),
      CompositePowerComp<Complex>(uid, name, true, true, logLevel) {
  if (withResistiveLosses)
    setVirtualNodeNumber(3);
  else
    setVirtualNodeNumber(2);

  setTerminalNumber(2);

  SPDLOG_LOGGER_INFO(mSLog, "Create {} {}", this->type(), name);
  **mIntfVoltage = MatrixComp::Zero(1, 1);
  **mIntfCurrent = MatrixComp::Zero(1, 1);
}

/// DEPRECATED: Delete method
SimPowerComp<Complex>::Ptr DP::Ph1::Transformer::clone(String name) {
  auto copy = Transformer::make(name, mLogLevel);
  copy->setParameters(mNominalVoltagePrimary, mNominalVoltageSecondary,
                      std::abs(**mRatio), std::arg(**mRatio), **mResistance,
                      **mInductance);
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
                     std::abs(**mRatio), std::arg(**mRatio));
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

void DP::Ph1::Transformer::resolveWindingOrientation() {
  mHVSide = (mNominalVoltagePrimary >= mNominalVoltageSecondary) ? 0 : 1;
  mLVSide = 1 - mHVSide;
  mRatioHVToLV = (mHVSide == 0) ? **mRatio : 1. / **mRatio;
  mOrientationSign = (mHVSide == 0) ? 1. : -1.;
  mNominalVoltageHV =
      (mHVSide == 0) ? mNominalVoltagePrimary : mNominalVoltageSecondary;
  mNominalVoltageLV =
      (mHVSide == 0) ? mNominalVoltageSecondary : mNominalVoltagePrimary;

  if ((mHVSide == 0) != (Math::abs(**mRatio) >= 1.) &&
      Math::abs(Math::abs(**mRatio) - 1.) > 1e-9)
    SPDLOG_LOGGER_WARN(
        mSLog,
        "Nominal voltages put the higher-voltage winding at terminal {} ({} "
        "[V] against {} [V]) but the turns ratio {} points the other way; "
        "check the argument order of setParameters()",
        mHVSide, mNominalVoltageHV, mNominalVoltageLV,
        Logger::complexToString(**mRatio));

  SPDLOG_LOGGER_INFO(mSLog,
                     "Higher-voltage winding at terminal {} ({} [V]), "
                     "lower-voltage winding at terminal {} ({} [V]), "
                     "ratio HV->LV = {}",
                     mHVSide, mNominalVoltageHV, mLVSide, mNominalVoltageLV,
                     Logger::complexToString(mRatioHVToLV));
}

void DP::Ph1::Transformer::createSubComponents() {
  if (mSubCompCreated)
    return;
  mSubCompCreated = true;

  resolveWindingOrientation();

  mSubInductor =
      std::make_shared<DP::Ph1::Inductor>(**mName + "_ind", mLogLevel);
  mSubInductor->setParameters(**mInductance);
  addMNASubComponent(mSubInductor, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);

  if (mNumVirtualNodes == 3) {
    mSubResistor =
        std::make_shared<DP::Ph1::Resistor>(**mName + "_res", mLogLevel);
    mSubResistor->setParameters(**mResistance);
    mSubResistor->connect({node(mHVSide), mVirtualNodes[2]});
    mSubInductor->connect({mVirtualNodes[2], mVirtualNodes[0]});
    addMNASubComponent(mSubResistor, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                       MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);
  } else {
    mSubInductor->connect({node(mHVSide), mVirtualNodes[0]});
  }

  Real pSnub = P_SNUB_TRANSFORMER * **mRatedPower;
  Real qSnub = Q_SNUB_TRANSFORMER * **mRatedPower;

  mSnubberResistance1 = std::pow(std::abs(mNominalVoltageHV), 2) / pSnub;
  mSubSnubResistor1 =
      std::make_shared<DP::Ph1::Resistor>(**mName + "_snub_res1", mLogLevel);
  mSubSnubResistor1->setParameters(mSnubberResistance1);
  mSubSnubResistor1->connect({node(mHVSide), DP::SimNode::GND});
  SPDLOG_LOGGER_INFO(
      mSLog,
      "Snubber Resistance 1 (connected to higher voltage side {}) = {} [Ohm]",
      node(mHVSide)->name(), Logger::realToString(mSnubberResistance1));
  addMNASubComponent(mSubSnubResistor1,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);

  mSnubberResistance2 = std::pow(std::abs(mNominalVoltageLV), 2) / pSnub;
  mSubSnubResistor2 =
      std::make_shared<DP::Ph1::Resistor>(**mName + "_snub_res2", mLogLevel);
  mSubSnubResistor2->setParameters(mSnubberResistance2);
  mSubSnubResistor2->connect({node(mLVSide), DP::SimNode::GND});
  SPDLOG_LOGGER_INFO(
      mSLog,
      "Snubber Resistance 2 (connected to lower voltage side {}) = {} [Ohm]",
      node(mLVSide)->name(), Logger::realToString(mSnubberResistance2));
  addMNASubComponent(mSubSnubResistor2,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);

  // Capacitor created here; its omega-dependent value is set in initializeParentFromNodesAndTerminals().
  mSubSnubCapacitor2 =
      std::make_shared<DP::Ph1::Capacitor>(**mName + "_snub_cap2", mLogLevel);
  mSubSnubCapacitor2->connect({node(mLVSide), DP::SimNode::GND});
  addMNASubComponent(mSubSnubCapacitor2,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);
}

void DP::Ph1::Transformer::initializeParentFromNodesAndTerminals(
    Real frequency) {
  Real omega = 2. * PI * frequency;
  Real qSnub = Q_SNUB_TRANSFORMER * **mRatedPower;
  mSnubberCapacitance2 =
      qSnub / std::pow(std::abs(mNominalVoltageLV), 2) / omega;
  mSubSnubCapacitor2->setParameters(mSnubberCapacitance2);
  SPDLOG_LOGGER_INFO(
      mSLog,
      "Snubber Capacitance 2 (connected to lower voltage side {}) = {} [F]",
      node(mLVSide)->name(), Logger::realToString(mSnubberCapacitance2));

  // Set initial voltage of virtual node in between
  mVirtualNodes[0]->setInitialVoltage(initialSingleVoltage(mLVSide) *
                                      mRatioHVToLV);

  if (mNumVirtualNodes == 3)
    mVirtualNodes[2]->setInitialVoltage(initialSingleVoltage(mHVSide));

  // Static calculations from load flow data
  Complex impedance = {**mResistance, omega * **mInductance};
  SPDLOG_LOGGER_INFO(mSLog, "Reactance={} [Ohm] (referred to primary side)",
                     omega * **mInductance);
  Complex impedanceVoltage =
      mOrientationSign * (mVirtualNodes[0]->initialSingleVoltage() -
                          initialSingleVoltage(mHVSide));
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
  SPDLOG_LOGGER_INFO(
      mSLog,
      "\nTerminal 0 connected to {:s} = sim node {:d}"
      "\nTerminal 1 connected to {:s} = sim node {:d}",
      mTerminals[0]->node()->name(), mTerminals[0]->node()->matrixNodeIndex(),
      mTerminals[1]->node()->name(), mTerminals[1]->node()->matrixNodeIndex());
}

void DP::Ph1::Transformer::mnaCompApplySystemMatrixStamp(
    SparseMatrixRow &systemMatrix) {
  // Ideal transformer equations
  if (terminalNotGrounded(mHVSide)) {
    Math::setMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(),
                           mVirtualNodes[1]->matrixNodeIndex(),
                           Complex(-1.0, 0));
    Math::setMatrixElement(systemMatrix, mVirtualNodes[1]->matrixNodeIndex(),
                           mVirtualNodes[0]->matrixNodeIndex(),
                           Complex(1.0, 0));
  }
  if (terminalNotGrounded(mLVSide)) {
    Math::setMatrixElement(systemMatrix, matrixNodeIndex(mLVSide),
                           mVirtualNodes[1]->matrixNodeIndex(),
                           std::conj(mRatioHVToLV));
    Math::setMatrixElement(systemMatrix, mVirtualNodes[1]->matrixNodeIndex(),
                           matrixNodeIndex(mLVSide), -mRatioHVToLV);
  }

  // Add subcomps to system matrix
  for (auto subcomp : mSubComponents)
    if (auto mnasubcomp = std::dynamic_pointer_cast<MNAInterface>(subcomp))
      mnasubcomp->mnaApplySystemMatrixStamp(systemMatrix);

  if (terminalNotGrounded(mHVSide)) {
    SPDLOG_LOGGER_INFO(mSLog, "Add {:s} to system at ({:d},{:d})",
                       Logger::complexToString(Complex(-1.0, 0)),
                       mVirtualNodes[0]->matrixNodeIndex(),
                       mVirtualNodes[1]->matrixNodeIndex());
    SPDLOG_LOGGER_INFO(mSLog, "Add {:s} to system at ({:d},{:d})",
                       Logger::complexToString(Complex(1.0, 0)),
                       mVirtualNodes[1]->matrixNodeIndex(),
                       mVirtualNodes[0]->matrixNodeIndex());
  }
  if (terminalNotGrounded(mLVSide)) {
    SPDLOG_LOGGER_INFO(mSLog, "Add {:s} to system at ({:d},{:d})",
                       Logger::complexToString(mRatioHVToLV),
                       matrixNodeIndex(mLVSide),
                       mVirtualNodes[1]->matrixNodeIndex());
    SPDLOG_LOGGER_INFO(mSLog, "Add {:s} to system at ({:d},{:d})",
                       Logger::complexToString(-mRatioHVToLV),
                       mVirtualNodes[1]->matrixNodeIndex(),
                       matrixNodeIndex(mLVSide));
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
  (**mIntfCurrent)(0, 0) = mOrientationSign * mSubInductor->intfCurrent()(0, 0);
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
