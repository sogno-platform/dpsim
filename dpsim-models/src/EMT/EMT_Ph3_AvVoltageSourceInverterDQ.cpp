/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/EMT/EMT_Ph3_AvVoltageSourceInverterDQ.h>

using namespace CPS;

EMT::Ph3::AvVoltageSourceInverterDQ::AvVoltageSourceInverterDQ(
    String uid, String name, Logger::Level logLevel, Bool withTrafo)
    : CompositePowerComp<Real>(uid, name, true, true, logLevel),
      mOmegaN(mAttributes->create<Real>("Omega_nom")),
      mPref(mAttributes->create<Real>("P_ref")),
      mQref(mAttributes->create<Real>("Q_ref")),
      mVcd(mAttributes->create<Real>("Vc_d", 0)),
      mVcq(mAttributes->create<Real>("Vc_q", 0)),
      mIrcd(mAttributes->create<Real>("Irc_d", 0)),
      mIrcq(mAttributes->create<Real>("Irc_q", 0)),
      mVsref(mAttributes->create<Matrix>("Vsref", Matrix::Zero(3, 1))),
      mVs(mAttributes->createDynamic<Matrix>("Vs")),
      mPllOutput(mAttributes->createDynamic<Matrix>("pll_output")),
      mPowerctrlInputs(mAttributes->createDynamic<Matrix>("powerctrl_inputs")),
      mPowerctrlOutputs(
          mAttributes->createDynamic<Matrix>("powerctrl_outputs")),
      mPowerctrlStates(mAttributes->createDynamic<Matrix>("powerctrl_states")) {
  mPhaseType = PhaseType::ABC;
  if (withTrafo) {
    setVirtualNodeNumber(4);
    mConnectionTransformer = EMT::Ph3::Transformer::make(
        **mName + "_trans", **mName + "_trans", mLogLevel);
    addMNASubComponent(mConnectionTransformer,
                       MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                       MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);
  } else {
    setVirtualNodeNumber(3);
  }
  mWithConnectionTransformer = withTrafo;
  setTerminalNumber(1);

  // Create electrical sub components
  mSubResistorF = EMT::Ph3::Resistor::make(**mName + "_resF", mLogLevel);
  mSubResistorC = EMT::Ph3::Resistor::make(**mName + "_resC", mLogLevel);
  mSubCapacitorF = EMT::Ph3::Capacitor::make(**mName + "_capF", mLogLevel);
  mSubInductorF = EMT::Ph3::Inductor::make(**mName + "_indF", mLogLevel);
  mSubCtrledVoltageSource =
      EMT::Ph3::VoltageSource::make(**mName + "_src", mLogLevel);
  addMNASubComponent(mSubResistorF, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, false);
  addMNASubComponent(mSubResistorC, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, false);
  addMNASubComponent(mSubCapacitorF, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);
  addMNASubComponent(mSubInductorF, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);

  // Pre-step of the subcontrolled voltage source is handled explicitly in mnaParentPreStep
  addMNASubComponent(mSubCtrledVoltageSource, MNA_SUBCOMP_TASK_ORDER::NO_TASK,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);

  SPDLOG_LOGGER_INFO(mSLog, "Electrical subcomponents: ");
  for (auto subcomp : mSubComponents)
    SPDLOG_LOGGER_INFO(mSLog, "- {}", subcomp->name());

  // Create control sub components
  mPLL = Signal::PLL::make(**mName + "_PLL", mLogLevel);
  mPowerControllerVSI = Signal::PowerControllerVSI::make(
      **mName + "_PowerControllerVSI", mLogLevel);

  // Sub voltage source
  mVs->setReference(mSubCtrledVoltageSource->mIntfVoltage);

  // PLL
  mPLL->mInputRef->setReference(mVcq);
  mPllOutput->setReference(mPLL->mOutputCurr);

  // Power controller
  // input references
  mPowerControllerVSI->mVc_d->setReference(mVcd);
  mPowerControllerVSI->mVc_q->setReference(mVcq);
  mPowerControllerVSI->mIrc_d->setReference(mIrcd);
  mPowerControllerVSI->mIrc_q->setReference(mIrcq);
  // input, state and output vector for logging
  mPowerctrlInputs->setReference(mPowerControllerVSI->mInputCurr);
  mPowerctrlStates->setReference(mPowerControllerVSI->mStateCurr);
  mPowerctrlOutputs->setReference(mPowerControllerVSI->mOutputCurr);
}

void EMT::Ph3::AvVoltageSourceInverterDQ::setParameters(Real sysOmega,
                                                        Real sysVoltNom,
                                                        Real Pref, Real Qref) {
  mParametersSet = true;

  SPDLOG_LOGGER_INFO(mSLog, "General Parameters:");
  SPDLOG_LOGGER_INFO(mSLog, "Nominal Voltage={} [V] Nominal Omega={} [1/s]",
                     sysVoltNom, sysOmega);
  SPDLOG_LOGGER_INFO(mSLog, "Active Power={} [W] Reactive Power={} [VAr]", Pref,
                     Qref);

  mPowerControllerVSI->setParameters(Pref, Qref);

  **mOmegaN = sysOmega;
  mVnom = sysVoltNom;
  **mPref = Pref;
  **mQref = Qref;
}

void EMT::Ph3::AvVoltageSourceInverterDQ::setTransformerParameters(
    Real nomVoltageEnd1, Real nomVoltageEnd2, Real ratedPower, Real ratioAbs,
    Real ratioPhase, Real resistance, Real inductance, Real omega) {

  Base::AvVoltageSourceInverterDQ::setTransformerParameters(
      nomVoltageEnd1, nomVoltageEnd2, ratedPower, ratioAbs, ratioPhase,
      resistance, inductance);

  SPDLOG_LOGGER_INFO(mSLog, "Connection Transformer Parameters:");
  SPDLOG_LOGGER_INFO(
      mSLog, "Nominal Voltage End 1={} [V] Nominal Voltage End 2={} [V]",
      mTransformerNominalVoltageEnd1, mTransformerNominalVoltageEnd2);
  SPDLOG_LOGGER_INFO(mSLog, "Rated Apparent Power = {} [VA]",
                     mTransformerRatedPower);
  SPDLOG_LOGGER_INFO(mSLog, "Resistance={} [Ohm] Inductance={} [H]",
                     mTransformerResistance, mTransformerInductance);
  SPDLOG_LOGGER_INFO(mSLog, "Tap Ratio={} [ ] Phase Shift={} [deg]",
                     mTransformerRatioAbs, mTransformerRatioPhase);

  if (mWithConnectionTransformer)
    // TODO: resistive losses neglected so far (mWithResistiveLosses=false)
    mConnectionTransformer->setParameters(
        mTransformerNominalVoltageEnd1, mTransformerNominalVoltageEnd2,
        mTransformerRatioAbs, mTransformerRatioPhase,
        CPS::Math::singlePhaseParameterToThreePhase(mTransformerResistance),
        CPS::Math::singlePhaseParameterToThreePhase(mTransformerInductance));
}

void EMT::Ph3::AvVoltageSourceInverterDQ::setControllerParameters(
    Real Kp_pll, Real Ki_pll, Real Kp_powerCtrl, Real Ki_powerCtrl,
    Real Kp_currCtrl, Real Ki_currCtrl, Real Omega_cutoff) {

  SPDLOG_LOGGER_INFO(mSLog, "Control Parameters:");
  SPDLOG_LOGGER_INFO(mSLog, "PLL: K_p = {}, K_i = {}, Omega_Nom = {}", Kp_pll,
                     Ki_pll, Omega_cutoff);
  SPDLOG_LOGGER_INFO(mSLog, "Power Loop: K_p = {}, K_i = {}", Kp_powerCtrl,
                     Ki_powerCtrl);
  SPDLOG_LOGGER_INFO(mSLog, "Current Loop: K_p = {}, K_i = {}", Kp_currCtrl,
                     Ki_currCtrl);
  SPDLOG_LOGGER_INFO(mSLog, "Cut-Off Frequency = {}", Omega_cutoff);

  // TODO: add and use Omega_nominal instead of Omega_cutoff
  mPLL->setParameters(Kp_pll, Ki_pll, Omega_cutoff);
  mPLL->composeStateSpaceMatrices();
  mPowerControllerVSI->setControllerParameters(
      Kp_powerCtrl, Ki_powerCtrl, Kp_currCtrl, Ki_currCtrl, Omega_cutoff);
}

void EMT::Ph3::AvVoltageSourceInverterDQ::setFilterParameters(Real Lf, Real Cf,
                                                              Real Rf,
                                                              Real Rc) {
  Base::AvVoltageSourceInverterDQ::setFilterParameters(Lf, Cf, Rf, Rc);

  SPDLOG_LOGGER_INFO(mSLog, "Filter Parameters:");
  SPDLOG_LOGGER_INFO(mSLog, "Inductance Lf={} [H] Capacitance Cf={} [F]", mLf,
                     mCf);
  SPDLOG_LOGGER_INFO(mSLog, "Resistance Rf={} [Ohm] Resistance Rc={} [Ohm]",
                     mRf, mRc);

  mSubResistorC->setParameters(
      CPS::Math::singlePhaseParameterToThreePhase(mRc));
  mSubResistorF->setParameters(
      CPS::Math::singlePhaseParameterToThreePhase(mRf));
  mSubInductorF->setParameters(
      CPS::Math::singlePhaseParameterToThreePhase(mLf));
  mSubCapacitorF->setParameters(
      CPS::Math::singlePhaseParameterToThreePhase(mCf));
}

void EMT::Ph3::AvVoltageSourceInverterDQ::setInitialStateValues(
    Real pInit, Real qInit, Real phi_dInit, Real phi_qInit, Real gamma_dInit,
    Real gamma_qInit) {

  SPDLOG_LOGGER_INFO(mSLog, "Initial State Value Parameters:");
  SPDLOG_LOGGER_INFO(mSLog, "PInit = {}, QInit = {}", pInit, qInit);
  SPDLOG_LOGGER_INFO(mSLog, "Phi_dInit = {}, Phi_qInit = {}", phi_dInit,
                     phi_qInit);
  SPDLOG_LOGGER_INFO(mSLog, "Gamma_dInit = {}, Gamma_qInit = {}", gamma_dInit,
                     gamma_qInit);

  mPowerControllerVSI->setInitialStateValues(pInit, qInit, phi_dInit, phi_qInit,
                                             gamma_dInit, gamma_qInit);
}

void EMT::Ph3::AvVoltageSourceInverterDQ::initializeFromNodesAndTerminals(
    Real frequency) {

  // use complex interface quantities for initialization calculations
  MatrixComp intfVoltageComplex = Matrix::Zero(3, 1);
  MatrixComp intfCurrentComplex = Matrix::Zero(3, 1);

  // derive complex threephase initialization from single phase initial values (only valid for balanced systems)
  intfVoltageComplex(0, 0) = RMS3PH_TO_PEAK1PH * initialSingleVoltage(0);
  intfVoltageComplex(1, 0) = intfVoltageComplex(0, 0) * SHIFT_TO_PHASE_B;
  intfVoltageComplex(2, 0) = intfVoltageComplex(0, 0) * SHIFT_TO_PHASE_C;
  intfCurrentComplex(0, 0) = -std::conj(2. / 3. * Complex(**mPref, **mQref) /
                                        intfVoltageComplex(0, 0));
  intfCurrentComplex(1, 0) = intfCurrentComplex(0, 0) * SHIFT_TO_PHASE_B;
  intfCurrentComplex(2, 0) = intfCurrentComplex(0, 0) * SHIFT_TO_PHASE_C;

  MatrixComp filterInterfaceInitialVoltage = MatrixComp::Zero(3, 1);
  MatrixComp filterInterfaceInitialCurrent = MatrixComp::Zero(3, 1);
  if (mWithConnectionTransformer) {
    // calculate quantities of low voltage side of transformer (being the interface quantities of the filter, calculations only valid for symmetrical systems)
    filterInterfaceInitialVoltage =
        (intfVoltageComplex -
         Complex(mTransformerResistance, mTransformerInductance * **mOmegaN) *
             intfCurrentComplex) /
        Complex(mTransformerRatioAbs, mTransformerRatioPhase);
    filterInterfaceInitialCurrent =
        intfCurrentComplex *
        Complex(mTransformerRatioAbs, mTransformerRatioPhase);

    // connect and init transformer
    mVirtualNodes[3]->setInitialVoltage(PEAK1PH_TO_RMS3PH *
                                        filterInterfaceInitialVoltage);
    mConnectionTransformer->connect({mTerminals[0]->node(), mVirtualNodes[3]});
  } else {
    // if no transformer used, filter interface equal to inverter interface
    filterInterfaceInitialVoltage = intfVoltageComplex;
    filterInterfaceInitialCurrent = intfCurrentComplex;
  }

  // derive initialization quantities of filter (calculations only valid for symmetrical systems)
  MatrixComp vcInit = filterInterfaceInitialVoltage -
                      filterInterfaceInitialCurrent * Complex(mRc, 0);
  MatrixComp icfInit = vcInit * Complex(0., 2. * PI * frequency * mCf);
  MatrixComp vfInit = vcInit - (filterInterfaceInitialCurrent - icfInit) *
                                   Complex(0., 2. * PI * frequency * mLf);
  MatrixComp vsInit =
      vfInit - (filterInterfaceInitialCurrent - icfInit) * Complex(mRf, 0);
  mVirtualNodes[0]->setInitialVoltage(PEAK1PH_TO_RMS3PH * vsInit);
  mVirtualNodes[1]->setInitialVoltage(PEAK1PH_TO_RMS3PH * vfInit);
  mVirtualNodes[2]->setInitialVoltage(PEAK1PH_TO_RMS3PH * vcInit);

  // save real interface quantities calculated from complex ones
  **mIntfVoltage = intfVoltageComplex.real();
  **mIntfCurrent = intfCurrentComplex.real();

  // Initialize controlled source
  mSubCtrledVoltageSource->setParameters(mVirtualNodes[0]->initialVoltage(),
                                         0.0);

  // Connect electrical subcomponents
  mSubCtrledVoltageSource->connect({SimNode::GND, mVirtualNodes[0]});
  mSubResistorF->connect({mVirtualNodes[0], mVirtualNodes[1]});
  mSubInductorF->connect({mVirtualNodes[1], mVirtualNodes[2]});
  mSubCapacitorF->connect({mVirtualNodes[2], SimNode::GND});
  if (mWithConnectionTransformer)
    mSubResistorC->connect({mVirtualNodes[2], mVirtualNodes[3]});
  else
    mSubResistorC->connect({mVirtualNodes[2], mTerminals[0]->node()});

  // Initialize electrical subcomponents
  for (auto subcomp : mSubComponents) {
    subcomp->initialize(mFrequencies);
    subcomp->initializeFromNodesAndTerminals(frequency);
  }

  // Initialize control subcomponents
  // current and voltage inputs to PLL and power controller
  Matrix vcdq, ircdq;
  Real theta = std::arg(mVirtualNodes[3]->initialSingleVoltage());
  vcdq =
      parkTransformPowerInvariant(theta, filterInterfaceInitialVoltage.real());
  ircdq = parkTransformPowerInvariant(
      theta, -1 * filterInterfaceInitialCurrent.real());

  **mVcd = vcdq(0, 0);
  **mVcq = vcdq(1, 0);
  **mIrcd = ircdq(0, 0);
  **mIrcq = ircdq(1, 0);

  // angle input
  Matrix matrixStateInit = Matrix::Zero(2, 1);
  Matrix matrixOutputInit = Matrix::Zero(2, 1);
  matrixStateInit(0, 0) = std::arg(mVirtualNodes[3]->initialSingleVoltage());
  matrixOutputInit(0, 0) = std::arg(mVirtualNodes[3]->initialSingleVoltage());
  mPLL->setInitialValues(**mVcq, matrixStateInit, matrixOutputInit);

  SPDLOG_LOGGER_INFO(
      mSLog,
      "\n--- Initialization from powerflow ---"
      "\nInterface voltage across: {:s}"
      "\nInterface current: {:s}"
      "\nTerminal 0 initial voltage: {:s}"
      "\nTerminal 0 connected to {:s} = sim node {:d}"
      "\nVirtual node 0 initial voltage: {:s}"
      "\nVirtual node 1 initial voltage: {:s}"
      "\nVirtual node 2 initial voltage: {:s}",
      Logger::phasorToString(intfVoltageComplex(0, 0)),
      Logger::phasorToString(intfCurrentComplex(0, 0)),
      Logger::phasorToString(initialSingleVoltage(0)),
      mTerminals[0]->node()->name(), mTerminals[0]->node()->matrixNodeIndex(),
      Logger::phasorToString(mVirtualNodes[0]->initialSingleVoltage()),
      Logger::phasorToString(mVirtualNodes[1]->initialSingleVoltage()),
      Logger::phasorToString(mVirtualNodes[2]->initialSingleVoltage()));
  if (mWithConnectionTransformer)
    SPDLOG_LOGGER_INFO(
        mSLog, "\nVirtual node 3 initial voltage: {:s}",
        Logger::phasorToString(mVirtualNodes[3]->initialSingleVoltage()));
  SPDLOG_LOGGER_INFO(mSLog, "\n--- Initialization from powerflow finished ---");
}

void EMT::Ph3::AvVoltageSourceInverterDQ::mnaParentInitialize(
    Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
  mTimeStep = timeStep;

  // initialize state space controller
  mPowerControllerVSI->initializeStateSpaceModel(omega, timeStep, leftVector);
  mPLL->setSimulationParameters(timeStep);

  // TODO: these are actually no MNA tasks
  mMnaTasks.push_back(std::make_shared<ControlPreStep>(*this));
  mMnaTasks.push_back(std::make_shared<ControlStep>(*this));
}

void EMT::Ph3::AvVoltageSourceInverterDQ::addControlPreStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes) {
  // add pre-step dependencies of subcomponents
  mPLL->signalAddPreStepDependencies(prevStepDependencies,
                                     attributeDependencies, modifiedAttributes);
  mPowerControllerVSI->signalAddPreStepDependencies(
      prevStepDependencies, attributeDependencies, modifiedAttributes);
}

void EMT::Ph3::AvVoltageSourceInverterDQ::controlPreStep(Real time,
                                                         Int timeStepCount) {
  // add pre-step of subcomponents
  mPLL->signalPreStep(time, timeStepCount);
  mPowerControllerVSI->signalPreStep(time, timeStepCount);
}

void EMT::Ph3::AvVoltageSourceInverterDQ::addControlStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes) {
  // add step dependencies of subcomponents
  mPLL->signalAddStepDependencies(prevStepDependencies, attributeDependencies,
                                  modifiedAttributes);
  mPowerControllerVSI->signalAddStepDependencies(
      prevStepDependencies, attributeDependencies, modifiedAttributes);
  // add step dependencies of component itself
  attributeDependencies.push_back(mIntfCurrent);
  attributeDependencies.push_back(mIntfVoltage);
  modifiedAttributes.push_back(mVsref);
}

Matrix EMT::Ph3::AvVoltageSourceInverterDQ::parkTransformPowerInvariant(
    Real theta, const Matrix &fabc) {
  // Calculates fdq = Tdq * fabc
  // Assumes that d-axis starts aligned with phase a
  Matrix Tdq = getParkTransformMatrixPowerInvariant(theta);
  Matrix dqvector = Tdq * fabc;
  return dqvector;
}

Matrix
EMT::Ph3::AvVoltageSourceInverterDQ::getParkTransformMatrixPowerInvariant(
    Real theta) {
  // Return park matrix for theta
  // Assumes that d-axis starts aligned with phase a
  Matrix Tdq = Matrix::Zero(2, 3);
  Real k = sqrt(2. / 3.);
  Tdq << k * cos(theta), k * cos(theta - 2. * M_PI / 3.),
      k * cos(theta + 2. * M_PI / 3.), -k * sin(theta),
      -k * sin(theta - 2. * M_PI / 3.), -k * sin(theta + 2. * M_PI / 3.);
  return Tdq;
}

Matrix EMT::Ph3::AvVoltageSourceInverterDQ::inverseParkTransformPowerInvariant(
    Real theta, const Matrix &fdq) {
  // Calculates fabc = Tabc * fdq
  // with d-axis starts aligned with phase a
  Matrix Tabc = getInverseParkTransformMatrixPowerInvariant(theta);
  Matrix fabc = Tabc * fdq;
  return fabc;
}

Matrix EMT::Ph3::AvVoltageSourceInverterDQ::
    getInverseParkTransformMatrixPowerInvariant(Real theta) {
  // Return inverse park matrix for theta
  /// with d-axis starts aligned with phase a
  Matrix Tabc = Matrix::Zero(3, 2);
  Real k = sqrt(2. / 3.);
  Tabc << k * cos(theta), -k * sin(theta), k * cos(theta - 2. * M_PI / 3.),
      -k * sin(theta - 2. * M_PI / 3.), k * cos(theta + 2. * M_PI / 3.),
      -k * sin(theta + 2. * M_PI / 3.);
  return Tabc;
}

void EMT::Ph3::AvVoltageSourceInverterDQ::controlStep(Real time,
                                                      Int timeStepCount) {
  // Transformation interface forward
  Matrix vcdq, ircdq;
  Real theta = mPLL->mOutputPrev->get()(0, 0);
  vcdq = parkTransformPowerInvariant(theta, **mVirtualNodes[3]->mVoltage);
  ircdq = parkTransformPowerInvariant(theta, -**mSubResistorC->mIntfCurrent);

  **mVcd = vcdq(0, 0);
  **mVcq = vcdq(1, 0);
  **mIrcd = ircdq(0, 0);
  **mIrcq = ircdq(1, 0);

  // add step of subcomponents
  mPLL->signalStep(time, timeStepCount);
  mPowerControllerVSI->signalStep(time, timeStepCount);

  // Transformation interface backward
  **mVsref = inverseParkTransformPowerInvariant(
      mPLL->mOutputPrev->get()(0, 0), mPowerControllerVSI->mOutputCurr->get());
  mThetaN = mThetaN + mTimeStep * **mOmegaN;
}

void EMT::Ph3::AvVoltageSourceInverterDQ::mnaParentAddPreStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes) {
  prevStepDependencies.push_back(mVsref);
  prevStepDependencies.push_back(mIntfCurrent);
  prevStepDependencies.push_back(mIntfVoltage);
  attributeDependencies.push_back(mPowerControllerVSI->mOutputPrev);
  attributeDependencies.push_back(mPLL->mOutputPrev);
  modifiedAttributes.push_back(mRightVector);
}

void EMT::Ph3::AvVoltageSourceInverterDQ::mnaParentPreStep(Real time,
                                                           Int timeStepCount) {
  // pre-step of subcomponents - controlled source
  if (mWithControl)
    mSubCtrledVoltageSource->mVoltageRef->set(PEAK1PH_TO_RMS3PH * **mVsref);

  std::dynamic_pointer_cast<MNAInterface>(mSubCtrledVoltageSource)
      ->mnaPreStep(time, timeStepCount);
  // pre-step of component itself
  mnaCompApplyRightSideVectorStamp(**mRightVector);
}

void EMT::Ph3::AvVoltageSourceInverterDQ::mnaParentAddPostStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes,
    Attribute<Matrix>::Ptr &leftVector) {
  attributeDependencies.push_back(leftVector);
  modifiedAttributes.push_back(mIntfVoltage);
  modifiedAttributes.push_back(mIntfCurrent);
}

void EMT::Ph3::AvVoltageSourceInverterDQ::mnaParentPostStep(
    Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
  mnaCompUpdateCurrent(**leftVector);
  mnaCompUpdateVoltage(**leftVector);
}

void EMT::Ph3::AvVoltageSourceInverterDQ::mnaCompUpdateCurrent(
    const Matrix &leftvector) {
  if (mWithConnectionTransformer)
    **mIntfCurrent = mConnectionTransformer->mIntfCurrent->get();
  else
    **mIntfCurrent = mSubResistorC->mIntfCurrent->get();
}

void EMT::Ph3::AvVoltageSourceInverterDQ::mnaCompUpdateVoltage(
    const Matrix &leftVector) {
  for (auto virtualNode : mVirtualNodes)
    virtualNode->mnaUpdateVoltage(leftVector);
  (**mIntfVoltage)(0, 0) =
      Math::realFromVectorElement(leftVector, matrixNodeIndex(0, 0));
  (**mIntfVoltage)(1, 0) =
      Math::realFromVectorElement(leftVector, matrixNodeIndex(0, 1));
  (**mIntfVoltage)(2, 0) =
      Math::realFromVectorElement(leftVector, matrixNodeIndex(0, 2));
}
