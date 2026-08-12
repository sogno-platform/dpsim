#include <cmath>
#include <stdexcept>

#include <dpsim-models/EMT/EMT_Ph3_GFL.h>

using namespace CPS;

EMT::Ph3::GFL::GFL(String uid, String name, Logger::Level logLevel)
    : CompositePowerComp<Real>(uid, name, true, true, logLevel),
      mVcd(mAttributes->create<Real>("vc_d", 0.0)),
      mVcq(mAttributes->create<Real>("vc_q", 0.0)),
      mIgridD(mAttributes->create<Real>("igrid_d", 0.0)),
      mIgridQ(mAttributes->create<Real>("igrid_q", 0.0)),
      mPInst(mAttributes->create<Real>("p_inst", 0.0)),
      mQInst(mAttributes->create<Real>("q_inst", 0.0)),
      mOmegaPLL(mAttributes->create<Real>("omega_pll", 0.0)),
      mVsref(mAttributes->create<Matrix>("vs_ref", Matrix::Zero(3, 1))),
      mVs(mAttributes->createDynamic<Matrix>("vs")),
      mPllOutput(mAttributes->createDynamic<Matrix>("pll_output")),
      mPowerctrlInputs(mAttributes->createDynamic<Matrix>("powerctrl_inputs")),
      mPowerctrlOutputs(
          mAttributes->createDynamic<Matrix>("powerctrl_outputs")),
      mPowerctrlStates(mAttributes->createDynamic<Matrix>("powerctrl_states")) {

  mPhaseType = PhaseType::ABC;

  // Only two internal electrical nodes are required:
  //
  //   virtual 0: controlled-source output / resistor input
  //   virtual 1: resistor output / inductor input
  //
  // The inductor output and capacitor are connected directly to the external
  // PCC terminal.
  setVirtualNodeNumber(2);
  setTerminalNumber(1);

  **mIntfVoltage = Matrix::Zero(3, 1);
  **mIntfCurrent = Matrix::Zero(3, 1);

  SPDLOG_LOGGER_INFO(mSLog, "Create {} {}", type(), name);

  // -------------------------------------------------------------------------
  // Electrical subcomponents
  // -------------------------------------------------------------------------
  mSubControlledVoltageSource =
      EMT::Ph3::VoltageSource::make(**mName + "_src", mLogLevel);

  mSubResistorF = EMT::Ph3::Resistor::make(**mName + "_resF", mLogLevel);

  mSubInductorF = EMT::Ph3::Inductor::make(**mName + "_indF", mLogLevel);

  mSubCapacitorF = EMT::Ph3::Capacitor::make(**mName + "_capF", mLogLevel);

  addMNASubComponent(mSubResistorF, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, false);

  addMNASubComponent(mSubInductorF, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);

  addMNASubComponent(mSubCapacitorF, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);

  // The controlled voltage source pre-step is handled explicitly by the
  // parent so its voltage reference can be updated immediately beforehand.
  addMNASubComponent(mSubControlledVoltageSource,
                     MNA_SUBCOMP_TASK_ORDER::NO_TASK,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);

  // -------------------------------------------------------------------------
  // Control subcomponents
  // -------------------------------------------------------------------------
  mPLL = Signal::PLL::make(**mName + "_PLL", mLogLevel);

  mPowerControllerVSI = Signal::PowerControllerVSI::make(
      **mName + "_PowerControllerVSI", mLogLevel);

  // Actual internal source voltage for logging.
  mVs->setReference(mSubControlledVoltageSource->mIntfVoltage);

  // PLL input is the PCC q-axis voltage.
  mPLL->mInputRef->setReference(mVcq);
  mPllOutput->setReference(mPLL->mOutputCurr);

  // Power controller measurements:
  //   Vc = PCC voltage
  //   Igrid = positive converter-to-grid current
  mPowerControllerVSI->mVc_d->setReference(mVcd);
  mPowerControllerVSI->mVc_q->setReference(mVcq);
  mPowerControllerVSI->mIrc_d->setReference(mIgridD);
  mPowerControllerVSI->mIrc_q->setReference(mIgridQ);

  mPowerctrlInputs->setReference(mPowerControllerVSI->mInputCurr);
  mPowerctrlStates->setReference(mPowerControllerVSI->mStateCurr);
  mPowerctrlOutputs->setReference(mPowerControllerVSI->mOutputCurr);
}

SimPowerComp<Real>::Ptr EMT::Ph3::GFL::clone(String name) {
  auto copy = GFL::make(name, mLogLevel);

  copy->setParameters(mOmegaN, mVnom, mPRef, mQRef);

  copy->setControllerParameters(mKpPLL, mKiPLL, mKpPowerCtrl, mKiPowerCtrl,
                                mKpCurrCtrl, mKiCurrCtrl, mOmegaCutoff);

  copy->setFilterParameters(mLf, mCf, mRf);

  if (mManualInitialStatesSet) {
    copy->setInitialStateValues(mPInitManual, mQInitManual, mPhiDInitManual,
                                mPhiQInitManual, mGammaDInitManual,
                                mGammaQInitManual);
  }

  copy->withControl(mWithControl);

  return copy;
}

void EMT::Ph3::GFL::setParameters(Real sysOmega, Real sysVoltNom, Real pRef,
                                  Real qRef) {

  if (sysOmega <= 0.0)
    throw std::invalid_argument(
        "GFL nominal angular frequency must be positive.");

  if (sysVoltNom <= 0.0)
    throw std::invalid_argument("GFL nominal voltage must be positive.");

  mOmegaN = sysOmega;
  mVnom = sysVoltNom;
  mPRef = pRef;
  mQRef = qRef;

  mPowerControllerVSI->setParameters(mPRef, mQRef);

  mParametersSet = true;

  SPDLOG_LOGGER_INFO(mSLog,
                     "GFL general parameters:"
                     "\n  V_nom = {} V"
                     "\n  omega_nom = {} rad/s"
                     "\n  P_ref = {} W"
                     "\n  Q_ref = {} var",
                     mVnom, mOmegaN, mPRef, mQRef);
}

void EMT::Ph3::GFL::setControllerParameters(Real kpPLL, Real kiPLL,
                                            Real kpPowerCtrl, Real kiPowerCtrl,
                                            Real kpCurrCtrl, Real kiCurrCtrl,
                                            Real omegaCutoff) {

  if (!mParametersSet)
    throw std::logic_error("GFL::setParameters() must be called before "
                           "setControllerParameters().");

  if (kiPLL == 0.0)
    throw std::invalid_argument("GFL PLL integral gain must be non-zero.");

  if (kiPowerCtrl == 0.0)
    throw std::invalid_argument(
        "GFL power-controller integral gain must be non-zero.");

  if (kiCurrCtrl == 0.0)
    throw std::invalid_argument(
        "GFL current-controller integral gain must be non-zero.");

  if (omegaCutoff <= 0.0)
    throw std::invalid_argument(
        "GFL power-filter cutoff frequency must be positive.");

  mKpPLL = kpPLL;
  mKiPLL = kiPLL;

  mKpPowerCtrl = kpPowerCtrl;
  mKiPowerCtrl = kiPowerCtrl;

  mKpCurrCtrl = kpCurrCtrl;
  mKiCurrCtrl = kiCurrCtrl;

  mOmegaCutoff = omegaCutoff;

  // Fix relative to the legacy class:
  // PLL nominal frequency is omega_nom, NOT the power-filter cutoff.
  mPLL->setParameters(mKpPLL, mKiPLL, mOmegaN);
  mPLL->composeStateSpaceMatrices();

  mPowerControllerVSI->setControllerParameters(
      mKpPowerCtrl, mKiPowerCtrl, mKpCurrCtrl, mKiCurrCtrl, mOmegaCutoff);

  mControllerParametersSet = true;
}

void EMT::Ph3::GFL::setFilterParameters(Real lf, Real cf, Real rf) {

  if (lf <= 0.0)
    throw std::invalid_argument("GFL filter inductance Lf must be positive.");

  if (cf <= 0.0)
    throw std::invalid_argument("GFL filter capacitance Cf must be positive.");

  if (rf < 0.0)
    throw std::invalid_argument(
        "GFL filter resistance Rf must be non-negative.");

  mLf = lf;
  mCf = cf;
  mRf = rf;

  mSubResistorF->setParameters(Math::singlePhaseParameterToThreePhase(mRf));

  mSubInductorF->setParameters(Math::singlePhaseParameterToThreePhase(mLf));

  mSubCapacitorF->setParameters(Math::singlePhaseParameterToThreePhase(mCf));

  mFilterParametersSet = true;

  SPDLOG_LOGGER_INFO(mSLog,
                     "GFL filter parameters:"
                     "\n  Lf = {} H"
                     "\n  Cf = {} F"
                     "\n  Rf = {} Ohm"
                     "\n  Rc = removed",
                     mLf, mCf, mRf);
}

void EMT::Ph3::GFL::setInitialStateValues(Real pInit, Real qInit, Real phiDInit,
                                          Real phiQInit, Real gammaDInit,
                                          Real gammaQInit) {

  mPInitManual = pInit;
  mQInitManual = qInit;
  mPhiDInitManual = phiDInit;
  mPhiQInitManual = phiQInit;
  mGammaDInitManual = gammaDInit;
  mGammaQInitManual = gammaQInit;

  mManualInitialStatesSet = true;

  mPowerControllerVSI->setInitialStateValues(pInit, qInit, phiDInit, phiQInit,
                                             gammaDInit, gammaQInit);
}

Matrix EMT::Ph3::GFL::getParkTransformMatrixPowerInvariant(Real theta) const {

  Matrix transform = Matrix::Zero(2, 3);

  const Real k = std::sqrt(2.0 / 3.0);

  transform << k * std::cos(theta), k * std::cos(theta - 2.0 * PI / 3.0),
      k * std::cos(theta + 2.0 * PI / 3.0),

      -k * std::sin(theta), -k * std::sin(theta - 2.0 * PI / 3.0),
      -k * std::sin(theta + 2.0 * PI / 3.0);

  return transform;
}

Matrix
EMT::Ph3::GFL::getInverseParkTransformMatrixPowerInvariant(Real theta) const {

  Matrix transform = Matrix::Zero(3, 2);

  const Real k = std::sqrt(2.0 / 3.0);

  transform << k * std::cos(theta), -k * std::sin(theta),

      k * std::cos(theta - 2.0 * PI / 3.0),
      -k * std::sin(theta - 2.0 * PI / 3.0),

      k * std::cos(theta + 2.0 * PI / 3.0),
      -k * std::sin(theta + 2.0 * PI / 3.0);

  return transform;
}

Matrix EMT::Ph3::GFL::parkTransformPowerInvariant(Real theta,
                                                  const Matrix &fabc) const {

  return getParkTransformMatrixPowerInvariant(theta) * fabc;
}

Matrix
EMT::Ph3::GFL::inverseParkTransformPowerInvariant(Real theta,
                                                  const Matrix &fdq) const {

  return getInverseParkTransformMatrixPowerInvariant(theta) * fdq;
}

void EMT::Ph3::GFL::updateMeasurementAttributes(Real theta) {

  // Correct voltage measurement:
  // capacitor voltage == PCC voltage because Rc has been removed and Cf is
  // connected directly at the external terminal.
  const Matrix vPccAbc = **mIntfVoltage;

  // Correct current measurement:
  // mIntfCurrent follows DPsim's component convention and points from the
  // external network into the converter. The GFL controller uses positive
  // converter-to-grid injection.
  const Matrix iGridAbc = -**mIntfCurrent;

  const Matrix vDq = parkTransformPowerInvariant(theta, vPccAbc);

  const Matrix iDq = parkTransformPowerInvariant(theta, iGridAbc);

  **mVcd = vDq(0, 0);
  **mVcq = vDq(1, 0);

  **mIgridD = iDq(0, 0);
  **mIgridQ = iDq(1, 0);
}

void EMT::Ph3::GFL::updatePowerAndFrequencyAttributes() {
  **mPInst = **mVcd * **mIgridD + **mVcq * **mIgridQ;

  **mQInst = -**mVcd * **mIgridQ + **mVcq * **mIgridD;

  // PLL output is [theta, phi_pll].
  const Real phiPLL = (**mPllOutput)(1, 0);

  **mOmegaPLL = mOmegaN + mKpPLL * **mVcq + mKiPLL * phiPLL;
}

void EMT::Ph3::GFL::initializeParentFromNodesAndTerminals(Real frequency) {

  if (!mParametersSet)
    throw std::logic_error(
        "GFL::setParameters() must be called before initialization.");

  if (!mFilterParametersSet)
    throw std::logic_error(
        "GFL::setFilterParameters() must be called before initialization.");

  if (!mControllerParametersSet)
    throw std::logic_error(
        "GFL::setControllerParameters() must be called before initialization.");

  const Real omega = 2.0 * PI * frequency;

  const Complex j(0.0, 1.0);

  // -------------------------------------------------------------------------
  // PCC voltage from power flow
  // -------------------------------------------------------------------------
  MatrixComp vPcc = MatrixComp::Zero(3, 1);

  vPcc(0, 0) = RMS3PH_TO_PEAK1PH * initialSingleVoltage(0);

  vPcc(1, 0) = vPcc(0, 0) * SHIFT_TO_PHASE_B;

  vPcc(2, 0) = vPcc(0, 0) * SHIFT_TO_PHASE_C;

  if (std::abs(vPcc(0, 0)) < 1e-12)
    throw std::runtime_error("GFL cannot initialize from zero PCC voltage.");

  // -------------------------------------------------------------------------
  // Positive converter-to-grid current from requested P/Q
  // -------------------------------------------------------------------------
  const Complex sRef(mPRef, mQRef);

  const Complex iGridA = std::conj((2.0 / 3.0) * sRef / vPcc(0, 0));

  MatrixComp iGrid = MatrixComp::Zero(3, 1);

  iGrid(0, 0) = iGridA;
  iGrid(1, 0) = iGridA * SHIFT_TO_PHASE_B;
  iGrid(2, 0) = iGridA * SHIFT_TO_PHASE_C;

  // -------------------------------------------------------------------------
  // Filter steady state
  //
  //   iF = iGrid + iCf
  //   vL_in = vPcc + j*w*Lf*iF
  //   vSource = vL_in + Rf*iF
  // -------------------------------------------------------------------------
  const MatrixComp iCf = j * omega * mCf * vPcc;

  const MatrixComp iF = iGrid + iCf;

  const MatrixComp vInductorInput = vPcc + j * omega * mLf * iF;

  const MatrixComp vSource = vInductorInput + mRf * iF;

  // Parent virtual nodes.
  mVirtualNodes[0]->setInitialVoltage(PEAK1PH_TO_RMS3PH * vSource);

  mVirtualNodes[1]->setInitialVoltage(PEAK1PH_TO_RMS3PH * vInductorInput);

  // Parent external interface.
  **mIntfVoltage = vPcc.real();

  // DPsim interface current points from grid into component.
  **mIntfCurrent = -iGrid.real();

  // -------------------------------------------------------------------------
  // Controlled source
  // -------------------------------------------------------------------------
  mSubControlledVoltageSource->setParameters(mVirtualNodes[0]->initialVoltage(),
                                             0.0);

  // Electrical topology:
  //
  //   GND -- source -- v0 -- Rf -- v1 -- Lf -- PCC
  //                                         |
  //                                         Cf
  //                                         |
  //                                        GND
  mSubControlledVoltageSource->connect({SimNode::GND, mVirtualNodes[0]});

  mSubResistorF->connect({mVirtualNodes[0], mVirtualNodes[1]});

  mSubInductorF->connect({mVirtualNodes[1], mTerminals[0]->node()});

  mSubCapacitorF->connect({mTerminals[0]->node(), SimNode::GND});

  // Do not initialize subcomponents manually here.
  // CompositePowerComp::initializeFromNodesAndTerminals() performs the
  // recursive subcomponent initialization after this parent hook returns.

  // -------------------------------------------------------------------------
  // Controller steady-state initialization
  // -------------------------------------------------------------------------
  const Real theta0 = std::arg(vPcc(0, 0));

  const Matrix vPccAbc0 = vPcc.real();

  const Matrix iGridAbc0 = iGrid.real();

  const Matrix vSourceAbc0 = vSource.real();

  const Matrix vDq0 = parkTransformPowerInvariant(theta0, vPccAbc0);

  const Matrix iDq0 = parkTransformPowerInvariant(theta0, iGridAbc0);

  const Matrix vSourceDq0 = parkTransformPowerInvariant(theta0, vSourceAbc0);

  **mVcd = vDq0(0, 0);

  **mVcq = vDq0(1, 0);

  **mIgridD = iDq0(0, 0);

  **mIgridQ = iDq0(1, 0);

  const Real pInit = **mVcd * **mIgridD + **mVcq * **mIgridQ;

  const Real qInit = -**mVcd * **mIgridQ + **mVcq * **mIgridD;

  // PLL state = [theta, phi_pll].
  Matrix pllStateInit = Matrix::Zero(2, 1);

  Matrix pllOutputInit = Matrix::Zero(2, 1);

  pllStateInit(0, 0) = theta0;

  pllStateInit(1, 0) = (omega - mOmegaN) / mKiPLL;

  pllOutputInit = pllStateInit;

  mPLL->setInitialValues(**mVcq, pllStateInit, pllOutputInit);

  if (!mManualInitialStatesSet) {
    // Choose controller integrators so that:
    //   i_ref_dq = i_grid_dq
    //   v_ref_dq = required steady-state source voltage.
    const Real phiDInit =
        (**mIgridD + mKpPowerCtrl * (pInit - mPRef)) / mKiPowerCtrl;

    const Real phiQInit =
        (**mIgridQ - mKpPowerCtrl * (qInit - mQRef)) / mKiPowerCtrl;

    const Real iRefD = mKpPowerCtrl * (mPRef - pInit) + mKiPowerCtrl * phiDInit;

    const Real iRefQ = mKpPowerCtrl * (qInit - mQRef) + mKiPowerCtrl * phiQInit;

    const Real gammaDInit =
        (vSourceDq0(0, 0) + mKpCurrCtrl * (**mIgridD - iRefD)) / mKiCurrCtrl;

    const Real gammaQInit =
        (vSourceDq0(1, 0) + mKpCurrCtrl * (**mIgridQ - iRefQ)) / mKiCurrCtrl;

    mPowerControllerVSI->setInitialStateValues(pInit, qInit, phiDInit, phiQInit,
                                               gammaDInit, gammaQInit);
  }

  // Critical startup fix:
  // initialize the abc controlled-source command before the very first
  // mnaParentPreStep() so it cannot overwrite the steady-state source with 0.
  **mVsref = vSourceAbc0;

  updatePowerAndFrequencyAttributes();

  SPDLOG_LOGGER_INFO(
      mSLog,
      "\n--- EMT_Ph3_GFL initialization ---"
      "\nPCC voltage abc: {:s}"
      "\nGrid injection abc: {:s}"
      "\nFilter current abc: {:s}"
      "\nSource voltage abc: {:s}"
      "\nP/Q init: [{:.6e}, {:.6e}]"
      "\nVc dq: [{:.6e}, {:.6e}]"
      "\nIgrid dq: [{:.6e}, {:.6e}]"
      "\n--- Initialization finished ---",
      Logger::matrixToString(vPccAbc0), Logger::matrixToString(iGridAbc0),
      Logger::matrixToString(iF.real()), Logger::matrixToString(vSourceAbc0),
      pInit, qInit, **mVcd, **mVcq, **mIgridD, **mIgridQ);
}

void EMT::Ph3::GFL::mnaParentInitialize(Real omega, Real timeStep,
                                        Attribute<Matrix>::Ptr leftVector) {

  mTimeStep = timeStep;

  // Initialize power controller first. Its initial output is the desired
  // source voltage in dq when the automatically computed states are used.
  mPowerControllerVSI->initializeStateSpaceModel(omega, timeStep, leftVector);

  // PowerControllerVSI::initializeStateSpaceModel() currently assigns its
  // internal cutoff member from the system omega argument. Restore the
  // explicitly configured controller cutoff so this GFL also works correctly
  // when omegaCutoff != omega_nom.
  mPowerControllerVSI->setControllerParameters(
      mKpPowerCtrl, mKiPowerCtrl, mKpCurrCtrl, mKiCurrCtrl, mOmegaCutoff);

  mPLL->setSimulationParameters(timeStep);

  // Keep the exact PF-derived source-voltage command that was calculated in
  // initializeParentFromNodesAndTerminals().  Reconstructing mVsref here from
  // controller states can introduce a first-step discontinuity even when the
  // electrical network is initialized exactly at its periodic operating point.
  updatePowerAndFrequencyAttributes();

  mMnaTasks.push_back(std::make_shared<ControlPreStep>(*this));

  mMnaTasks.push_back(std::make_shared<ControlStep>(*this));
}

void EMT::Ph3::GFL::addControlPreStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes) {

  mPLL->signalAddPreStepDependencies(prevStepDependencies,
                                     attributeDependencies, modifiedAttributes);

  mPowerControllerVSI->signalAddPreStepDependencies(
      prevStepDependencies, attributeDependencies, modifiedAttributes);
}

void EMT::Ph3::GFL::controlPreStep(Real time, Int timeStepCount) {

  mPLL->signalPreStep(time, timeStepCount);

  mPowerControllerVSI->signalPreStep(time, timeStepCount);
}

void EMT::Ph3::GFL::addControlStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes) {

  // Follow the proven AvVoltageSourceInverterDQ scheduler pattern.
  //
  // The measurement attributes mVcd/mVcq/mIgridD/mIgridQ are updated
  // internally inside controlStep() immediately before the PLL and power
  // controller are stepped. They must NOT be declared as scheduler outputs of
  // this same task, because the signal components consume them through dynamic
  // references and this creates a circular dependency.
  mPLL->signalAddStepDependencies(prevStepDependencies, attributeDependencies,
                                  modifiedAttributes);

  mPowerControllerVSI->signalAddStepDependencies(
      prevStepDependencies, attributeDependencies, modifiedAttributes);

  // The control calculation needs the solved PCC quantities from the network.
  attributeDependencies.push_back(mIntfCurrent);

  attributeDependencies.push_back(mIntfVoltage);

  // Only the externally consumed controller output is a scheduler-visible
  // modification of this parent task.
  modifiedAttributes.push_back(mVsref);
}

void EMT::Ph3::GFL::controlStep(Real time, Int timeStepCount) {

  // -----------------------------------------------------------------------
  // Time alignment of the PLL angle
  // -----------------------------------------------------------------------
  //
  // mIntfVoltage/mIntfCurrent represent the newly solved EMT sample, while
  // mPLL->mOutputPrev contains the PLL state from the previous sample.
  //
  // Using theta_prev directly therefore creates an artificial phase error of
  //
  //     Delta theta ~= omega * dt
  //
  // even in a perfectly synchronized 50-Hz steady state.  For dt=100 us this
  // is ~1.8 deg and, with the present PLL gains, produces the ~0.5-Hz startup
  // frequency jump seen in the benchmark.
  //
  // Predict theta to the measurement instant using the previous PLL frequency.
  Real thetaMeasurement = mPLL->mOutputPrev->get()(0, 0);

  if (timeStepCount > 0) {
    const Real phiPLLPrev = mPLL->mOutputPrev->get()(1, 0);

    const Real omegaPLLPrev = mOmegaN + mKpPLL * **mVcq + mKiPLL * phiPLLPrev;

    thetaMeasurement += mTimeStep * omegaPLLPrev;
  }

  updateMeasurementAttributes(thetaMeasurement);

  // Update PLL and cascaded P/Q-current controller.
  mPLL->signalStep(time, timeStepCount);

  mPowerControllerVSI->signalStep(time, timeStepCount);

  // -----------------------------------------------------------------------
  // Time alignment of the controlled-source command
  // -----------------------------------------------------------------------
  //
  // mVsref is a prevStep dependency of the MNA pre-step, hence the value
  // generated here is applied at the NEXT EMT sample.  Rotate the dq command
  // with a one-step-ahead angle so a constant dq voltage remains a correct
  // 50-Hz abc waveform rather than being delayed by omega*dt.
  const Real thetaPLL = (**mPllOutput)(0, 0);

  const Real phiPLL = (**mPllOutput)(1, 0);

  const Real omegaPLL = mOmegaN + mKpPLL * **mVcq + mKiPLL * phiPLL;

  const Real thetaCommand = thetaPLL + mTimeStep * omegaPLL;

  **mVsref =
      inverseParkTransformPowerInvariant(thetaCommand, **mPowerctrlOutputs);

  updatePowerAndFrequencyAttributes();
}

void EMT::Ph3::GFL::mnaParentAddPreStepDependencies(
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

void EMT::Ph3::GFL::mnaParentPreStep(Real time, Int timeStepCount) {

  if (mWithControl) {
    // VoltageSource::mVoltageRef uses RMS line-line-equivalent scaling,
    // whereas mVsref stores instantaneous phase-peak abc values.
    mSubControlledVoltageSource->mVoltageRef->set(PEAK1PH_TO_RMS3PH * **mVsref);
  }

  std::dynamic_pointer_cast<MNAInterface>(mSubControlledVoltageSource)
      ->mnaPreStep(time, timeStepCount);

  mnaCompApplyRightSideVectorStamp(**mRightVector);
}

void EMT::Ph3::GFL::mnaParentAddPostStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes,
    Attribute<Matrix>::Ptr &leftVector) {

  attributeDependencies.push_back(leftVector);

  modifiedAttributes.push_back(mIntfVoltage);

  modifiedAttributes.push_back(mIntfCurrent);
}

void EMT::Ph3::GFL::mnaParentPostStep(Real time, Int timeStepCount,
                                      Attribute<Matrix>::Ptr &leftVector) {

  // Subcomponent post-step tasks run before the parent, so the inductor current
  // has already been updated when this executes.
  mnaCompUpdateCurrent(**leftVector);

  mnaCompUpdateVoltage(**leftVector);
}

void EMT::Ph3::GFL::mnaCompUpdateCurrent(const Matrix &leftVector) {

  // The external PCC current is the NET current entering the complete
  // converter component, not the filter-inductor current alone.
  //
  // Connections:
  //
  //   inductor  : {internal node, PCC}
  //   capacitor : {PCC, GND}
  //
  // With DPsim's two-terminal convention:
  //
  //   i_ind = -i_f
  //   i_cap = -i_cf
  //
  // Hence the parent interface current is
  //
  //   i_intf = i_ind - i_cap
  //          = -i_f + i_cf
  //          = -i_grid
  //
  // so -mIntfCurrent is exactly the positive converter-to-grid current used
  // by the controller.
  **mIntfCurrent =
      mSubInductorF->mIntfCurrent->get() - mSubCapacitorF->mIntfCurrent->get();
}

void EMT::Ph3::GFL::mnaCompUpdateVoltage(const Matrix &leftVector) {

  for (auto virtualNode : mVirtualNodes)
    virtualNode->mnaUpdateVoltage(leftVector);

  **mIntfVoltage = Matrix::Zero(3, 1);

  if (terminalNotGrounded(0)) {
    (**mIntfVoltage)(0, 0) =
        Math::realFromVectorElement(leftVector, matrixNodeIndex(0, 0));

    (**mIntfVoltage)(1, 0) =
        Math::realFromVectorElement(leftVector, matrixNodeIndex(0, 1));

    (**mIntfVoltage)(2, 0) =
        Math::realFromVectorElement(leftVector, matrixNodeIndex(0, 2));
  }
}
