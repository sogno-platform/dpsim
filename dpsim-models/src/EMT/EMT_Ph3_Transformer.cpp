/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/EMT/EMT_Ph3_Transformer.h>
#include <dpsim-models/MNAStampUtils.h>

using namespace CPS;

EMT::Ph3::Transformer::Transformer(String uid, String name,
                                   Logger::Level logLevel,
                                   Bool withResistiveLosses)
    : Transformer(uid, name, true, logLevel, withResistiveLosses) {}

EMT::Ph3::Transformer::Transformer(String uid, String name,
                                   Bool useSubcomponents,
                                   Logger::Level logLevel,
                                   Bool withResistiveLosses)
    : Base::Ph3::Transformer(mAttributes),
      CompositePowerComp<Real>(uid, name, true, true, logLevel) {
  mUseSubcomponents = useSubcomponents;
  mDirectState = mAttributes->create<Matrix>("direct_state");
  **mDirectState = Matrix::Zero(18, 1);
  mPhaseType = PhaseType::ABC;
  mWithResistiveLosses = withResistiveLosses;
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
  auto copy = Transformer::make(name, mUseSubcomponents, mLogLevel,
                                mWithResistiveLosses);
  copy->setParameters(mNominalVoltagePrimary, mNominalVoltageSecondary,
                      mRatedPower, std::abs(**mRatio), std::arg(**mRatio),
                      mResistance, mInductance);
  copy->setTapRatio(tapRatio());
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
                     std::abs(effectiveRatio()), std::arg(effectiveRatio()));

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
  mRatioFromReference =
      (mReferenceTerminal == 0) ? effectiveRatio() : 1. / effectiveRatio();
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

  if (!mUseSubcomponents) {
    resolveWindingRoles();
    return;
  }

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
  if (!mUseSubcomponents) {
    resolveWindingRoles();
    const Real omega = 2. * PI * frequency;
    mSeriesInductanceHalf = mInductance / 2.;
    mSeriesResistanceHalf.setZero();
    if (mWithResistiveLosses)
      mSeriesResistanceHalf = mResistance / 2.;
    if (mRatedPower > 0.) {
      const Real bmag = std::sqrt(mNoLoadCurrent * mNoLoadCurrent -
                                  mNoLoadLoss * mNoLoadLoss) *
                        mRatedPower /
                        std::pow(nominalVoltageAt(mReferenceTerminal), 2);
      const Real rmag = std::pow(nominalVoltageAt(mReferenceTerminal), 2) /
                        (mNoLoadLoss * mRatedPower);
      mMagnetizingInductance =
          Math::singlePhaseParameterToThreePhase(1. / (omega * bmag));
      mMagnetizingResistance = Math::singlePhaseParameterToThreePhase(rmag);
      mDirectMagnetizingConductance = mMagnetizingResistance.inverse();
    } else {
      mMagnetizingInductance = Matrix::Zero(3, 3);
      mDirectMagnetizingConductance = Matrix::Zero(3, 3);
    }
    MatrixComp v0 = MatrixComp::Zero(3, 1), v1 = MatrixComp::Zero(3, 1);
    v0(0) = RMS3PH_TO_PEAK1PH * initialSingleVoltage(0);
    v1(0) = RMS3PH_TO_PEAK1PH * initialSingleVoltage(1);
    v0(1) = v0(0) * SHIFT_TO_PHASE_B;
    v0(2) = v0(0) * SHIFT_TO_PHASE_C;
    v1(1) = v1(0) * SHIFT_TO_PHASE_B;
    v1(2) = v1(0) * SHIFT_TO_PHASE_C;
    const MatrixComp vref = mReferenceTerminal == 0 ? v0 : v1;
    const MatrixComp vother = mReferenceTerminal == 0 ? v1 : v0;
    const MatrixComp vreferred = mRatioFromReference.real() * vother;
    const MatrixComp zhalf =
        mSeriesResistanceHalf.cast<Complex>() +
        Complex(0., omega) * mSeriesInductanceHalf.cast<Complex>();
    const MatrixComp yseries = zhalf.inverse();
    MatrixComp ymag = mDirectMagnetizingConductance.cast<Complex>();
    if (mRatedPower > 0.)
      ymag += Complex(0., -1. / omega) *
              mMagnetizingInductance.inverse().cast<Complex>();
    const MatrixComp midpointInv = (2. * yseries + ymag).inverse();
    const MatrixComp vmid = midpointInv * yseries * (vref + vreferred);
    const MatrixComp i0 = yseries * (vmid - vref);
    const MatrixComp i1 = yseries * (vreferred - vmid);
    const MatrixComp imag =
        (ymag - mDirectMagnetizingConductance.cast<Complex>()) * (-vmid);
    (**mDirectState).setZero();
    (**mDirectState).block<3, 1>(0, 0) =
        (Complex(0., omega) * mSeriesInductanceHalf.cast<Complex>() * i0)
            .real();
    (**mDirectState).block<3, 1>(3, 0) = i0.real();
    (**mDirectState).block<3, 1>(6, 0) =
        (Complex(0., omega) * mSeriesInductanceHalf.cast<Complex>() * i1)
            .real();
    (**mDirectState).block<3, 1>(9, 0) = i1.real();
    const Complex ratioNodeVoltage =
        initialSingleVoltage(nonReferenceTerminal()) * mRatioFromReference;
    mVirtualNodes[0]->setInitialVoltage(ratioNodeVoltage);
    mVirtualNodes[2]->setInitialVoltage(PEAK1PH_TO_RMS3PH * vmid(0));
    if (mWithResistiveLosses) {
      const Complex halfImpedance(mResistance(0, 0) / 2.,
                                  omega * mInductance(0, 0) / 2.);
      const Complex referenceCurrent = (vref(0) - vmid(0)) / halfImpedance;
      const Complex nonReferenceCurrent =
          (vmid(0) - vreferred(0)) / halfImpedance;
      mVirtualNodes[3]->setInitialVoltage(
          initialSingleVoltage(mReferenceTerminal) -
          PEAK1PH_TO_RMS3PH * referenceCurrent * mResistance(0, 0) / 2.);
      mVirtualNodes[4]->setInitialVoltage(
          PEAK1PH_TO_RMS3PH * vmid(0) -
          PEAK1PH_TO_RMS3PH * nonReferenceCurrent * mResistance(0, 0) / 2.);
    }
    if (mRatedPower > 0.) {
      (**mDirectState).block<3, 1>(12, 0) =
          (Complex(0., omega) * mMagnetizingInductance.cast<Complex>() * imag)
              .real();
      (**mDirectState).block<3, 1>(15, 0) = imag.real();
    }
    **mIntfVoltage = (v1 - v0).real();
    // Match the legacy parent trace at t=0. The direct companion states above
    // retain the initialized branch currents and are used by the first MNA
    // step; the legacy composite does not publish a terminal current until its
    // first post-step.
    **mIntfCurrent = Matrix::Zero(3, 1);
    return;
  }
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

  Complex halfImpedance =
      Complex(mResistance(0, 0) / 2., omega * mInductance(0, 0) / 2.);
  Complex magnetizingAdmittance = Complex(0, 0);
  if (mSubMagnetizingInductor)
    magnetizingAdmittance =
        1. / mMagnetizingResistance(0, 0) +
        1. / Complex(0, omega * mMagnetizingInductance(0, 0));

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
    mVirtualNodes[3]->setInitialVoltage(
        referenceVoltage - referenceCurrent * mResistance(0, 0) / 2.);
    mVirtualNodes[4]->setInitialVoltage(
        midpointVoltage - nonReferenceCurrent * mResistance(0, 0) / 2.);
  }

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
  if (!mUseSubcomponents) {
    updateMatrixNodeIndices();
    mDirectInductorAdmittance =
        (timeStep / 2.) * mSeriesInductanceHalf.inverse();
    mDirectMagnetizingInductorAdmittance.setZero();
    if (mRatedPower > 0.)
      mDirectMagnetizingInductorAdmittance =
          (timeStep / 2.) * mMagnetizingInductance.inverse();
    return;
  }
  SPDLOG_LOGGER_INFO(
      mSLog,
      "\nTerminal 0 connected to {:s} = sim node {:d}"
      "\nTerminal 1 connected to {:s} = sim node {:d}",
      mTerminals[0]->node()->name(), mTerminals[0]->node()->matrixNodeIndex(),
      mTerminals[1]->node()->name(), mTerminals[1]->node()->matrixNodeIndex());
}

void EMT::Ph3::Transformer::mnaCompApplySystemMatrixStamp(
    SparseMatrixRow &systemMatrix) {
  if (!mUseSubcomponents) {
    updateMatrixNodeIndices();
    auto stampIdealTransformer = [&]() {
      if (terminalNotGrounded(mReferenceTerminal))
        for (UInt phase = 0; phase < 3; ++phase) {
          const UInt row0 =
              mVirtualNodes[0]->matrixNodeIndex(static_cast<PhaseType>(phase));
          const UInt row1 =
              mVirtualNodes[1]->matrixNodeIndex(static_cast<PhaseType>(phase));
          Math::setMatrixElement(systemMatrix, row0, row1, -1.);
          Math::setMatrixElement(systemMatrix, row1, row0, 1.);
        }
      const UInt other = nonReferenceTerminal();
      if (terminalNotGrounded(other))
        for (UInt phase = 0; phase < 3; ++phase) {
          const UInt terminalIndex = matrixNodeIndex(other, phase);
          const UInt idealIndex =
              mVirtualNodes[1]->matrixNodeIndex(static_cast<PhaseType>(phase));
          Math::setMatrixElement(systemMatrix, terminalIndex, idealIndex,
                                 mRatioFromReference.real());
          Math::setMatrixElement(systemMatrix, idealIndex, terminalIndex,
                                 -mRatioFromReference.real());
        }
    };
    stampIdealTransformer();
    const UInt midpoint = mVirtualNodes[2]->matrixNodeIndex(PhaseType::A);
    const UInt ratioNode = mVirtualNodes[0]->matrixNodeIndex(PhaseType::A);
    const bool withLosses = mWithResistiveLosses;
    const UInt firstInductor =
        withLosses ? mVirtualNodes[3]->matrixNodeIndex(PhaseType::A)
                   : matrixNodeIndex(mReferenceTerminal);
    const UInt secondInductor =
        withLosses ? mVirtualNodes[4]->matrixNodeIndex(PhaseType::A) : midpoint;
    if (withLosses) {
      const Matrix halfResistanceConductance = (mResistance / 2.).inverse();
      MNAStampUtils::stampConductanceMatrix(
          halfResistanceConductance, systemMatrix,
          matrixNodeIndex(mReferenceTerminal), firstInductor,
          terminalNotGrounded(mReferenceTerminal), true, mSLog);
      MNAStampUtils::stampConductanceMatrix(halfResistanceConductance,
                                            systemMatrix, midpoint,
                                            secondInductor, true, true, mSLog);
    }
    MNAStampUtils::stampConductanceMatrix(mDirectInductorAdmittance,
                                          systemMatrix, firstInductor, midpoint,
                                          true, true, mSLog);
    MNAStampUtils::stampConductanceMatrix(mDirectInductorAdmittance,
                                          systemMatrix, secondInductor,
                                          ratioNode, true, true, mSLog);
    MNAStampUtils::stampConductanceMatrix(mDirectMagnetizingConductance,
                                          systemMatrix, midpoint, 0, true,
                                          false, mSLog);
    MNAStampUtils::stampConductanceMatrix(mDirectMagnetizingInductorAdmittance,
                                          systemMatrix, midpoint, 0, true,
                                          false, mSLog);
    return;
  }
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
                       Logger::complexToString(mRatioFromReference),
                       matrixNodeIndex(1, 0),
                       mVirtualNodes[1]->matrixNodeIndex(PhaseType::A));
    SPDLOG_LOGGER_INFO(mSLog, "Add {:s} to system at ({:d},{:d})",
                       Logger::complexToString(mRatioFromReference),
                       matrixNodeIndex(1, 1),
                       mVirtualNodes[1]->matrixNodeIndex(PhaseType::B));
    SPDLOG_LOGGER_INFO(mSLog, "Add {:s} to system at ({:d},{:d})",
                       Logger::complexToString(mRatioFromReference),
                       matrixNodeIndex(1, 2),
                       mVirtualNodes[1]->matrixNodeIndex(PhaseType::C));

    SPDLOG_LOGGER_INFO(mSLog, "Add {:s} to system at ({:d},{:d})",
                       Logger::complexToString(-mRatioFromReference),
                       mVirtualNodes[1]->matrixNodeIndex(PhaseType::A),
                       matrixNodeIndex(1, 0));
    SPDLOG_LOGGER_INFO(mSLog, "Add {:s} to system at ({:d},{:d})",
                       Logger::complexToString(-mRatioFromReference),
                       mVirtualNodes[1]->matrixNodeIndex(PhaseType::B),
                       matrixNodeIndex(1, 1));
    SPDLOG_LOGGER_INFO(mSLog, "Add {:s} to system at ({:d},{:d})",
                       Logger::complexToString(-mRatioFromReference),
                       mVirtualNodes[1]->matrixNodeIndex(PhaseType::C),
                       matrixNodeIndex(1, 2));
  }
}

void EMT::Ph3::Transformer::mnaParentAddPreStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes) {
  if (!mUseSubcomponents)
    prevStepDependencies.push_back(mDirectState);
  else {
    prevStepDependencies.push_back(mIntfCurrent);
    prevStepDependencies.push_back(mIntfVoltage);
  }
  modifiedAttributes.push_back(mRightVector);
}

void EMT::Ph3::Transformer::mnaParentPreStep(Real time, Int timeStepCount) {
  if (!mUseSubcomponents) {
    const auto &state = **mDirectState;
    mDirectHistory0 = mDirectInductorAdmittance * state.block<3, 1>(0, 0) +
                      state.block<3, 1>(3, 0);
    mDirectHistory1 = mDirectInductorAdmittance * state.block<3, 1>(6, 0) +
                      state.block<3, 1>(9, 0);
    mDirectHistoryMag =
        mDirectMagnetizingInductorAdmittance * state.block<3, 1>(12, 0) +
        state.block<3, 1>(15, 0);
  }
  mnaCompApplyRightSideVectorStamp(**mRightVector);
}

void EMT::Ph3::Transformer::mnaParentApplyRightSideVectorStamp(
    Matrix &rightVector) {
  if (mUseSubcomponents)
    return;
  const UInt midpoint = mVirtualNodes[2]->matrixNodeIndex(PhaseType::A);
  const UInt ratioNode = mVirtualNodes[0]->matrixNodeIndex(PhaseType::A);
  const UInt firstInductor =
      mWithResistiveLosses ? mVirtualNodes[3]->matrixNodeIndex(PhaseType::A)
                           : matrixNodeIndex(mReferenceTerminal);
  const UInt secondInductor =
      mWithResistiveLosses ? mVirtualNodes[4]->matrixNodeIndex(PhaseType::A)
                           : midpoint;
  for (UInt phase = 0; phase < 3; ++phase) {
    const UInt midpointIndex = midpoint + phase;
    const UInt ratioIndex = ratioNode + phase;
    const UInt firstIndex = firstInductor + phase;
    const UInt secondIndex = secondInductor + phase;
    Math::setVectorElement(rightVector, firstIndex, mDirectHistory0(phase));
    // Without explicit series-resistance nodes, the second inductor starts
    // at the midpoint too. Accumulate both branch-history contributions into
    // that one row; writing secondIndex separately would overwrite the first
    // inductor's midpoint contribution (secondIndex == midpointIndex).
    Math::setVectorElement(
        rightVector, midpointIndex,
        -mDirectHistory0(phase) +
            (mWithResistiveLosses ? 0. : mDirectHistory1(phase)) +
            mDirectHistoryMag(phase));
    if (mWithResistiveLosses)
      Math::setVectorElement(rightVector, secondIndex, mDirectHistory1(phase));
    Math::setVectorElement(rightVector, ratioIndex, -mDirectHistory1(phase));
  }
}

Bool EMT::Ph3::Transformer::mnaHasCompactRightSideVectorStamp() const {
  return !mUseSubcomponents ||
         CompositePowerComp<Real>::mnaHasCompactRightSideVectorStamp();
}

void EMT::Ph3::Transformer::mnaAddCompactRightSideVectorStamp(
    Matrix &rightVector) {
  if (mUseSubcomponents) {
    CompositePowerComp<Real>::mnaAddCompactRightSideVectorStamp(rightVector);
    return;
  }
  const UInt midpoint = mVirtualNodes[2]->matrixNodeIndex(PhaseType::A);
  const UInt ratioNode = mVirtualNodes[0]->matrixNodeIndex(PhaseType::A);
  const UInt firstInductor =
      mWithResistiveLosses ? mVirtualNodes[3]->matrixNodeIndex(PhaseType::A)
                           : matrixNodeIndex(mReferenceTerminal);
  const UInt secondInductor =
      mWithResistiveLosses ? mVirtualNodes[4]->matrixNodeIndex(PhaseType::A)
                           : midpoint;
  for (UInt phase = 0; phase < 3; ++phase) {
    const UInt midpointIndex = midpoint + phase;
    const UInt ratioIndex = ratioNode + phase;
    const UInt firstIndex = firstInductor + phase;
    const UInt secondIndex = secondInductor + phase;
    if (mWithResistiveLosses || terminalNotGrounded(mReferenceTerminal))
      Math::addToVectorElement(rightVector, firstIndex, mDirectHistory0(phase));
    Math::addToVectorElement(
        rightVector, midpointIndex,
        -mDirectHistory0(phase) +
            (mWithResistiveLosses ? 0. : mDirectHistory1(phase)) +
            mDirectHistoryMag(phase));
    if (mWithResistiveLosses)
      Math::addToVectorElement(rightVector, secondIndex,
                               mDirectHistory1(phase));
    Math::addToVectorElement(rightVector, ratioIndex, -mDirectHistory1(phase));
  }
}

void EMT::Ph3::Transformer::mnaParentAddPostStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes,
    Attribute<Matrix>::Ptr &leftVector) {
  attributeDependencies.push_back(leftVector);
  modifiedAttributes.push_back(mIntfVoltage);
  modifiedAttributes.push_back(mIntfCurrent);
  if (!mUseSubcomponents)
    modifiedAttributes.push_back(mDirectState);
}

void EMT::Ph3::Transformer::mnaParentPostStep(
    Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
  if (!mUseSubcomponents) {
    Eigen::Matrix<Real, 3, 1> v0 = Eigen::Matrix<Real, 3, 1>::Zero();
    Eigen::Matrix<Real, 3, 1> v1 = Eigen::Matrix<Real, 3, 1>::Zero();
    Eigen::Matrix<Real, 3, 1> virtualVoltage[5];
    for (auto &voltage : virtualVoltage)
      voltage.setZero();
    for (UInt phase = 0; phase < 3; ++phase) {
      if (terminalNotGrounded(0))
        v0(phase) = Math::realFromVectorElement(**leftVector,
                                                matrixNodeIndex(0, phase));
      if (terminalNotGrounded(1))
        v1(phase) = Math::realFromVectorElement(**leftVector,
                                                matrixNodeIndex(1, phase));
      for (UInt node = 0; node < mVirtualNodes.size(); ++node)
        virtualVoltage[node](phase) = Math::realFromVectorElement(
            **leftVector, mVirtualNodes[node]->matrixNodeIndex(
                              static_cast<PhaseType>(phase)));
    }
    const auto vmid = virtualVoltage[2];
    const auto firstEndpoint = mWithResistiveLosses
                                   ? virtualVoltage[3]
                                   : (mReferenceTerminal == 0 ? v0 : v1);
    const auto secondEndpoint = mWithResistiveLosses ? virtualVoltage[4] : vmid;
    const auto vratio = virtualVoltage[0];
    const auto vInductor0 = vmid - firstEndpoint;
    const auto vInductor1 = vratio - secondEndpoint;
    const auto i0 = mDirectInductorAdmittance * vInductor0 + mDirectHistory0;
    const auto i1 = mDirectInductorAdmittance * vInductor1 + mDirectHistory1;
    const auto imag =
        mDirectMagnetizingInductorAdmittance * (-vmid) + mDirectHistoryMag;
    (**mDirectState).block<3, 1>(0, 0) = vInductor0;
    (**mDirectState).block<3, 1>(3, 0) = i0;
    (**mDirectState).block<3, 1>(6, 0) = vInductor1;
    (**mDirectState).block<3, 1>(9, 0) = i1;
    (**mDirectState).block<3, 1>(12, 0) = -vmid;
    (**mDirectState).block<3, 1>(15, 0) = imag;
    **mIntfVoltage = v1 - v0;
    **mIntfCurrent = mOrientationSign * i0;
    return;
  }
  mnaCompUpdateVoltage(**leftVector);
  mnaCompUpdateCurrent(**leftVector);
}

void EMT::Ph3::Transformer::mnaCompUpdateCurrent(const Matrix &leftVector) {
  if (!mUseSubcomponents) {
    **mIntfCurrent = mOrientationSign * (**mDirectState).block<3, 1>(3, 0);
    return;
  }
  **mIntfCurrent = mOrientationSign * mSubInductor->intfCurrent();
}

void EMT::Ph3::Transformer::mnaCompUpdateVoltage(const Matrix &leftVector) {
  // v1 - v0
  (**mIntfVoltage).setZero();
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
