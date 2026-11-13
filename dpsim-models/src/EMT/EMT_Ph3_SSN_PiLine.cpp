/* SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
 * SPDX-License-Identifier: MPL-2.0
 */

#include <dpsim-models/EMT/EMT_Ph3_SSN_PiLine.h>
#include <dpsim-models/MNAStampUtils.h>

using namespace CPS;

namespace {
constexpr Int kSeriesCurrent = 0;
constexpr Int kSeriesInductorVoltage = 3;
constexpr Int kCapVoltage0 = 6;
constexpr Int kCapCurrent0 = 9;
constexpr Int kCapVoltage1 = 12;
constexpr Int kCapCurrent1 = 15;
} // namespace

EMT::Ph3::SSN::PiLine::PiLine(String uid, String name, Logger::Level logLevel)
    : MNASimPowerComp<Real>(uid, name, true, true, logLevel),
      Base::Ph3::PiLine(mAttributes),
      mState(mAttributes->create<Matrix>("state")), mStateValue(&mState->get()),
      mInductorAdmittance(Matrix::Zero(3, 3)),
      mSeriesAdmittance(Matrix::Zero(3, 3)),
      mSeriesSystemInverse(Matrix::Identity(3, 3)),
      mCapAdmittance(Matrix::Zero(3, 3)), mSeriesHistory(Matrix::Zero(3, 1)),
      mInductorHistory(Matrix::Zero(3, 1)), mCapHistory0(Matrix::Zero(3, 1)),
      mCapHistory1(Matrix::Zero(3, 1)) {
  mPhaseType = PhaseType::ABC;
  setTerminalNumber(2);
  *mIntfVoltageValue = Matrix::Zero(3, 1);
  *mIntfCurrentValue = Matrix::Zero(3, 1);
  *mStateValue = Matrix::Zero(18, 1);
}

SimPowerComp<Real>::Ptr EMT::Ph3::SSN::PiLine::clone(String copySuffix) {
  auto copy = SharedFactory<PiLine>::make(copySuffix, mLogLevel);
  copy->setParameters(**mSeriesRes, **mSeriesInd, **mParallelCap,
                      **mParallelCond);
  return copy;
}

void EMT::Ph3::SSN::PiLine::setParameters(const Matrix &seriesResistance,
                                          const Matrix &seriesInductance,
                                          const Matrix &parallelCapacitance,
                                          const Matrix &parallelConductance) {
  for (const Matrix *parameter : {&seriesResistance, &seriesInductance,
                                  &parallelCapacitance, &parallelConductance}) {
    if (parameter->rows() != 3 || parameter->cols() != 3)
      throw std::invalid_argument("PiLine parameter matrices must be 3 x 3.");
  }
  Base::Ph3::PiLine::setParameters(seriesResistance, seriesInductance,
                                   parallelCapacitance, parallelConductance);
}

void EMT::Ph3::SSN::PiLine::initializeFromNodesAndTerminals(Real frequency) {
  if ((**mParallelCond)(0, 0) <= 0.) {
    **mParallelCond = Matrix::Zero(3, 3);
    (**mParallelCond).diagonal().setConstant(1e-6);
  }

  const Real omega = 2. * PI * frequency;
  MatrixComp v0Phasor = MatrixComp::Zero(3, 1);
  MatrixComp v1Phasor = MatrixComp::Zero(3, 1);
  v0Phasor(0, 0) = RMS3PH_TO_PEAK1PH * initialSingleVoltage(0);
  v1Phasor(0, 0) = RMS3PH_TO_PEAK1PH * initialSingleVoltage(1);
  v0Phasor(1, 0) = v0Phasor(0, 0) * SHIFT_TO_PHASE_B;
  v0Phasor(2, 0) = v0Phasor(0, 0) * SHIFT_TO_PHASE_C;
  v1Phasor(1, 0) = v1Phasor(0, 0) * SHIFT_TO_PHASE_B;
  v1Phasor(2, 0) = v1Phasor(0, 0) * SHIFT_TO_PHASE_C;

  const MatrixComp voltagePhasor = v1Phasor - v0Phasor;
  const MatrixComp impedance =
      (**mSeriesRes).cast<Complex>() +
      Complex(0., omega) * (**mSeriesInd).cast<Complex>();
  const MatrixComp currentPhasor = impedance.inverse() * voltagePhasor;
  const MatrixComp inductorVoltagePhasor =
      voltagePhasor - (**mSeriesRes).cast<Complex>() * currentPhasor;
  const MatrixComp capHalf = ((**mParallelCap) / 2.).cast<Complex>();
  const MatrixComp capCurrent0 = Complex(0., omega) * capHalf * v0Phasor;
  const MatrixComp capCurrent1 = Complex(0., omega) * capHalf * v1Phasor;

  *mIntfVoltageValue = voltagePhasor.real();
  *mIntfCurrentValue = currentPhasor.real();
  mStateValue->setZero();
  mStateValue->block<3, 1>(kSeriesCurrent, 0) = currentPhasor.real();
  mStateValue->block<3, 1>(kSeriesInductorVoltage, 0) =
      inductorVoltagePhasor.real();
  mStateValue->block<3, 1>(kCapVoltage0, 0) = v0Phasor.real();
  mStateValue->block<3, 1>(kCapCurrent0, 0) = capCurrent0.real();
  mStateValue->block<3, 1>(kCapVoltage1, 0) = v1Phasor.real();
  mStateValue->block<3, 1>(kCapCurrent1, 0) = capCurrent1.real();
}

void EMT::Ph3::SSN::PiLine::exportFusedModel(LinearSSNModel &model) {
  model = LinearSSNModel{};
  model.kind = LinearSSNModel::Kind::PiLine;
  model.terminalCount = 2;
  for (UInt terminal = 0; terminal < 2; ++terminal) {
    model.grounded[terminal] = !terminalNotGrounded(terminal);
    for (UInt phase = 0; phase < 3; ++phase)
      model.nodeIndices[terminal * 3 + phase] =
          matrixNodeIndex(terminal, phase);
    for (UInt phase = 0; phase < 3; ++phase)
      model.nodeNames[terminal * 3 + phase] = node(terminal)->name();
  }
  model.state = *mStateValue;
  model.seriesResistance = **mSeriesRes;
  model.inductorAdmittance = mInductorAdmittance;
  model.seriesSystemInverse = mSeriesSystemInverse;
  model.seriesAdmittance = mSeriesAdmittance;
  model.capAdmittance = mCapAdmittance;
  model.parallelConductance = **mParallelCond;
  model.portAdmittance = Matrix::Zero(6, 6);
  model.portAdmittance.block<3, 3>(0, 0) =
      mSeriesAdmittance + mCapAdmittance + **mParallelCond / 2.;
  model.portAdmittance.block<3, 3>(0, 3) = -mSeriesAdmittance;
  model.portAdmittance.block<3, 3>(3, 0) = -mSeriesAdmittance;
  model.portAdmittance.block<3, 3>(3, 3) =
      mSeriesAdmittance + mCapAdmittance + **mParallelCond / 2.;
}

void EMT::Ph3::SSN::PiLine::mnaCompInitialize(Real, Real timeStep,
                                              Attribute<Matrix>::Ptr) {
  updateMatrixNodeIndices();
  mInductorAdmittance = (timeStep / 2.) * (**mSeriesInd).inverse();
  mSeriesSystemInverse =
      (Matrix::Identity(3, 3) + mInductorAdmittance * **mSeriesRes).inverse();
  mSeriesAdmittance = mSeriesSystemInverse * mInductorAdmittance;
  mCapAdmittance = (2. / timeStep) * ((**mParallelCap) / 2.);
}

void EMT::Ph3::SSN::PiLine::mnaCompApplySystemMatrixStamp(
    SparseMatrixRow &systemMatrix) {
  MNAStampUtils::stampConductanceMatrix(
      mSeriesAdmittance, systemMatrix, matrixNodeIndex(0), matrixNodeIndex(1),
      terminalNotGrounded(0), terminalNotGrounded(1), mSLog);
  const Matrix shunt = mCapAdmittance + ((**mParallelCond) / 2.);
  for (UInt terminal = 0; terminal < 2; ++terminal) {
    MNAStampUtils::stampConductanceMatrix(
        shunt, systemMatrix, matrixNodeIndex(terminal), 0,
        terminalNotGrounded(terminal), false, mSLog);
  }
}

void EMT::Ph3::SSN::PiLine::mnaCompPreStep(Real, Int) {
  mInductorHistory.noalias() =
      mInductorAdmittance * mStateValue->block<3, 1>(kSeriesInductorVoltage, 0);
  mInductorHistory += mStateValue->block<3, 1>(kSeriesCurrent, 0);
  mSeriesHistory.noalias() = mSeriesSystemInverse * mInductorHistory;
  mCapHistory0 = -mStateValue->block<3, 1>(kCapCurrent0, 0) -
                 mCapAdmittance * mStateValue->block<3, 1>(kCapVoltage0, 0);
  mCapHistory1 = -mStateValue->block<3, 1>(kCapCurrent1, 0) -
                 mCapAdmittance * mStateValue->block<3, 1>(kCapVoltage1, 0);
}

void EMT::Ph3::SSN::PiLine::mnaCompApplyRightSideVectorStamp(
    Matrix &rightVector) {
  if (terminalNotGrounded(0)) {
    for (UInt phase = 0; phase < 3; ++phase)
      Math::setVectorElement(rightVector, matrixNodeIndex(0, phase),
                             mSeriesHistory(phase, 0) - mCapHistory0(phase, 0));
  }
  if (terminalNotGrounded(1)) {
    for (UInt phase = 0; phase < 3; ++phase)
      Math::setVectorElement(rightVector, matrixNodeIndex(1, phase),
                             -mSeriesHistory(phase, 0) -
                                 mCapHistory1(phase, 0));
  }
}

void EMT::Ph3::SSN::PiLine::mnaAddCompactRightSideVectorStamp(
    Matrix &rightVector) {
  if (terminalNotGrounded(0)) {
    for (UInt phase = 0; phase < 3; ++phase)
      Math::addToVectorElement(rightVector, matrixNodeIndex(0, phase),
                               mSeriesHistory(phase, 0) -
                                   mCapHistory0(phase, 0));
  }
  if (terminalNotGrounded(1)) {
    for (UInt phase = 0; phase < 3; ++phase)
      Math::addToVectorElement(rightVector, matrixNodeIndex(1, phase),
                               -mSeriesHistory(phase, 0) -
                                   mCapHistory1(phase, 0));
  }
}

EMT::Ph3::SSN::PiLine::TerminalVoltage
EMT::Ph3::SSN::PiLine::terminalVoltage(const Matrix &leftVector,
                                       UInt terminal) {
  TerminalVoltage voltage = TerminalVoltage::Zero();
  if (terminalNotGrounded(terminal)) {
    for (UInt phase = 0; phase < 3; ++phase)
      voltage(phase, 0) = Math::realFromVectorElement(
          leftVector, matrixNodeIndex(terminal, phase));
  }
  return voltage;
}

void EMT::Ph3::SSN::PiLine::mnaCompUpdateVoltage(const Matrix &leftVector) {
  *mIntfVoltageValue =
      terminalVoltage(leftVector, 1) - terminalVoltage(leftVector, 0);
}

void EMT::Ph3::SSN::PiLine::mnaCompUpdateCurrent(const Matrix &) {
  *mIntfCurrentValue =
      mSeriesAdmittance * (*mIntfVoltageValue) + mSeriesHistory;
}

void EMT::Ph3::SSN::PiLine::mnaCompPostStep(
    Real, Int, Attribute<Matrix>::Ptr &leftVector) {
  // Reuse the terminal voltages for both the interface update and the
  // capacitor state update instead of extracting them twice.
  const TerminalVoltage v0 = terminalVoltage(**leftVector, 0);
  const TerminalVoltage v1 = terminalVoltage(**leftVector, 1);
  const TerminalVoltage interfaceVoltage = v1 - v0;
  *mIntfVoltageValue = interfaceVoltage;
  mIntfCurrentValue->noalias() = mSeriesAdmittance * interfaceVoltage;
  *mIntfCurrentValue += mSeriesHistory;
  const TerminalVoltage inductorVoltage =
      interfaceVoltage - **mSeriesRes * (*mIntfCurrentValue);
  const TerminalVoltage capCurrent0 = mCapAdmittance * v0 + mCapHistory0;
  const TerminalVoltage capCurrent1 = mCapAdmittance * v1 + mCapHistory1;

  mStateValue->block<3, 1>(kSeriesCurrent, 0) = *mIntfCurrentValue;
  mStateValue->block<3, 1>(kSeriesInductorVoltage, 0) = inductorVoltage;
  mStateValue->block<3, 1>(kCapVoltage0, 0) = v0;
  mStateValue->block<3, 1>(kCapCurrent0, 0) = capCurrent0;
  mStateValue->block<3, 1>(kCapVoltage1, 0) = v1;
  mStateValue->block<3, 1>(kCapCurrent1, 0) = capCurrent1;
}

void EMT::Ph3::SSN::PiLine::mnaCompAddPreStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes) {
  prevStepDependencies.push_back(mState);
  modifiedAttributes.push_back(mRightVector);
}

void EMT::Ph3::SSN::PiLine::mnaCompAddPostStepDependencies(
    AttributeBase::List &, AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes,
    Attribute<Matrix>::Ptr &leftVector) {
  attributeDependencies.push_back(leftVector);
  modifiedAttributes.push_back(mIntfVoltage);
  modifiedAttributes.push_back(mIntfCurrent);
  modifiedAttributes.push_back(mState);
}
