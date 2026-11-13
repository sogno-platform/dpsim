/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/DP/DP_Ph1_AvVoltageSourceInverterDQ.h>
#include <dpsim-models/MNAStampUtils.h>

#include <array>
#include <stdexcept>

using namespace CPS;

DP::Ph1::AvVoltageSourceInverterDQ::AvVoltageSourceInverterDQ(
    String uid, String name, Logger::Level logLevel, Bool withTrafo)
    : AvVoltageSourceInverterDQ(uid, name, true, logLevel, withTrafo) {}

DP::Ph1::AvVoltageSourceInverterDQ::AvVoltageSourceInverterDQ(
    String uid, String name, Bool useSubcomponents, Logger::Level logLevel,
    Bool withTrafo)
    : CompositePowerComp<Complex>(uid, name, true, true, logLevel),
      mOmegaN(mAttributes->create<Real>("Omega_nom")),
      mPref(mAttributes->create<Real>("P_ref")),
      mQref(mAttributes->create<Real>("Q_ref")),
      mVcd(mAttributes->create<Real>("Vc_d", 0)),
      mVcq(mAttributes->create<Real>("Vc_q", 0)),
      mIrcd(mAttributes->create<Real>("Irc_d", 0)),
      mIrcq(mAttributes->create<Real>("Irc_q", 0)),
      mVsref(mAttributes->create<MatrixComp>("Vsref", MatrixComp::Zero(1, 1))),
      mVs(mAttributes->createDynamic<MatrixComp>("Vs")),
      mDirectSourceVoltageAttr(mAttributes->create<MatrixComp>(
          "direct_source_voltage", MatrixComp::Zero(1, 1))),
      mPllOutput(mAttributes->createDynamic<Matrix>("pll_output")),
      mPowerctrlInputs(mAttributes->createDynamic<Matrix>("powerctrl_inputs")),
      mPowerctrlOutputs(
          mAttributes->createDynamic<Matrix>("powerctrl_outputs")),
      mPowerctrlStates(mAttributes->createDynamic<Matrix>("powerctrl_states")) {

  mUseSubcomponents = useSubcomponents;
  if (withTrafo) {
    setVirtualNodeNumber(4);
    mConnectionTransformer = DP::Ph1::Transformer::make(
        **mName + "_trans", **mName + "_trans", mLogLevel, false);
    addMNASubComponent(mConnectionTransformer,
                       MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                       MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);
  } else {
    setVirtualNodeNumber(mUseSubcomponents ? 3 : 4);
  }
  mWithConnectionTransformer = withTrafo;
  setTerminalNumber(1);

  SPDLOG_LOGGER_INFO(mSLog, "Create {} {}", type(), name);
  **mIntfVoltage = MatrixComp::Zero(1, 1);
  **mIntfCurrent = MatrixComp::Zero(1, 1);

  if (mUseSubcomponents) {
    // Create electrical sub components
    mSubResistorF = DP::Ph1::Resistor::make(**mName + "_resF", mLogLevel);
    mSubResistorC = DP::Ph1::Resistor::make(**mName + "_resC", mLogLevel);
    mSubCapacitorF = DP::Ph1::Capacitor::make(**mName + "_capF", mLogLevel);
    mSubInductorF = DP::Ph1::Inductor::make(**mName + "_indF", mLogLevel);
    addMNASubComponent(mSubResistorF,
                       MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                       MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, false);
    addMNASubComponent(mSubResistorC,
                       MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                       MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, false);
    addMNASubComponent(mSubCapacitorF,
                       MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                       MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);
    addMNASubComponent(mSubInductorF,
                       MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                       MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);
  }

  // Keep the controlled source as a component in transformer-enabled direct
  // mode. Its branch unknown is distinct from the transformer's low-voltage
  // node, while the R-L-C filter remains stamped directly by this parent.
  if (!mUseSubcomponents && mWithConnectionTransformer) {
    mSubCtrledVoltageSource =
        DP::Ph1::VoltageSource::make(**mName + "_src", mLogLevel);
    addMNASubComponent(mSubCtrledVoltageSource, MNA_SUBCOMP_TASK_ORDER::NO_TASK,
                       MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);
  } else if (mUseSubcomponents) {
    mSubCtrledVoltageSource =
        DP::Ph1::VoltageSource::make(**mName + "_src", mLogLevel);
    addMNASubComponent(mSubCtrledVoltageSource, MNA_SUBCOMP_TASK_ORDER::NO_TASK,
                       MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);
  }

  SPDLOG_LOGGER_INFO(mSLog, "Electrical subcomponents: ");
  for (auto subcomp : mSubComponents)
    SPDLOG_LOGGER_INFO(mSLog, "- {}", subcomp->name());

  // Create control sub components
  mPLL = Signal::PLL::make(**mName + "_PLL", mLogLevel);
  mPowerControllerVSI = Signal::PowerControllerVSI::make(
      **mName + "_PowerControllerVSI", mLogLevel);

  // Sub voltage source
  if (mUseSubcomponents || mWithConnectionTransformer)
    mVs->setReference(mSubCtrledVoltageSource->mIntfVoltage);
  else
    mVs->setReference(mDirectSourceVoltageAttr);

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

void DP::Ph1::AvVoltageSourceInverterDQ::setParameters(Real sysOmega,
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

void DP::Ph1::AvVoltageSourceInverterDQ::setTransformerParameters(
    Real nomVoltagePrimary, Real nomVoltageSecondary, Real ratedPower,
    Real ratioAbs, Real ratioPhase, Real resistance, Real inductance) {

  Base::AvVoltageSourceInverterDQ::setTransformerParameters(
      nomVoltagePrimary, nomVoltageSecondary, ratedPower, ratioAbs, ratioPhase,
      resistance, inductance);

  SPDLOG_LOGGER_INFO(mSLog, "Connection Transformer Parameters:");
  SPDLOG_LOGGER_INFO(
      mSLog, "Nominal Voltage Primary={} [V] Nominal Voltage Secondary={} [V]",
      mTransformerNominalVoltagePrimary, mTransformerNominalVoltageSecondary);
  SPDLOG_LOGGER_INFO(mSLog, "Rated Apparent Power = {} [VA]",
                     mTransformerRatedPower);
  SPDLOG_LOGGER_INFO(mSLog, "Resistance={} [Ohm] Inductance={} [H]",
                     mTransformerResistance, mTransformerInductance);
  SPDLOG_LOGGER_INFO(mSLog, "Tap Ratio={} [ ] Phase Shift={} [deg]",
                     mTransformerRatioAbs, mTransformerRatioPhase);

  if (mWithConnectionTransformer)
    // TODO: resistive losses neglected so far (mWithResistiveLosses=false)
    mConnectionTransformer->setParameters(
        mTransformerNominalVoltagePrimary, mTransformerNominalVoltageSecondary,
        mTransformerRatedPower, mTransformerRatioAbs, mTransformerRatioPhase,
        mTransformerResistance, mTransformerInductance);
}

void DP::Ph1::AvVoltageSourceInverterDQ::setControllerParameters(
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

void DP::Ph1::AvVoltageSourceInverterDQ::setFilterParameters(Real Lf, Real Cf,
                                                             Real Rf, Real Rc) {
  Base::AvVoltageSourceInverterDQ::setFilterParameters(Lf, Cf, Rf, Rc);

  SPDLOG_LOGGER_INFO(mSLog, "Filter Parameters:");
  SPDLOG_LOGGER_INFO(mSLog, "Inductance Lf={} [H] Capacitance Cf={} [F]", mLf,
                     mCf);
  SPDLOG_LOGGER_INFO(mSLog, "Resistance Rf={} [Ohm] Resistance Rc={} [Ohm]",
                     mRf, mRc);

  if (mUseSubcomponents) {
    mSubResistorC->setParameters(mRc);
    mSubResistorF->setParameters(mRf);
    mSubInductorF->setParameters(mLf);
    mSubCapacitorF->setParameters(mCf);
  }
}

void DP::Ph1::AvVoltageSourceInverterDQ::setInitialStateValues(
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

void DP::Ph1::AvVoltageSourceInverterDQ::initializeParentFromNodesAndTerminals(
    Real frequency) {

  // set initial interface quantities
  (**mIntfVoltage)(0, 0) = initialSingleVoltage(0);
  (**mIntfCurrent)(0, 0) =
      -std::conj(Complex(**mPref, **mQref) / (**mIntfVoltage)(0, 0));

  Complex filterInterfaceInitialVoltage;
  Complex filterInterfaceInitialCurrent;

  if (mWithConnectionTransformer) {
    // calculate quantities of low voltage side of transformer (being the interface quantities of the filter)
    filterInterfaceInitialVoltage =
        ((**mIntfVoltage)(0, 0) -
         Complex(mTransformerResistance, mTransformerInductance * **mOmegaN) *
             (**mIntfCurrent)(0, 0)) /
        Complex(mTransformerRatioAbs, mTransformerRatioPhase);
    filterInterfaceInitialCurrent =
        (**mIntfCurrent)(0, 0) *
        Complex(mTransformerRatioAbs, mTransformerRatioPhase);

    // connect transformer
    mVirtualNodes[3]->setInitialVoltage(filterInterfaceInitialVoltage);
    mConnectionTransformer->connect({mTerminals[0]->node(), mVirtualNodes[3]});
  } else {
    // if no transformer used, filter interface equal to inverter interface
    filterInterfaceInitialVoltage = (**mIntfVoltage)(0, 0);
    filterInterfaceInitialCurrent = (**mIntfCurrent)(0, 0);
  }

  // derive initialization quantities of filter
  Complex vcInit =
      filterInterfaceInitialVoltage - filterInterfaceInitialCurrent * mRc;
  Complex icfInit = vcInit * Complex(0., 2. * PI * frequency * mCf);
  Complex vfInit = vcInit - (filterInterfaceInitialCurrent - icfInit) *
                                Complex(0., 2. * PI * frequency * mLf);
  Complex vsInit =
      vfInit - (filterInterfaceInitialCurrent - icfInit) * Complex(mRf, 0);
  mVirtualNodes[0]->setInitialVoltage(vsInit);
  mVirtualNodes[1]->setInitialVoltage(vfInit);
  mVirtualNodes[2]->setInitialVoltage(vcInit);

  (**mVsref)(0, 0) = mVirtualNodes[0]->initialSingleVoltage();
  mDirectSourceValue = (**mVsref)(0, 0);
  (**mDirectSourceVoltageAttr)(0, 0) = mDirectSourceValue;
  if (mUseSubcomponents) {
    mSubCtrledVoltageSource->setParameters((**mVsref)(0, 0));
    mSubCtrledVoltageSource->connect({SimNode::GND, mVirtualNodes[0]});
    mSubResistorF->connect({mVirtualNodes[0], mVirtualNodes[1]});
    mSubInductorF->connect({mVirtualNodes[1], mVirtualNodes[2]});
    mSubCapacitorF->connect({mVirtualNodes[2], SimNode::GND});
    if (mWithConnectionTransformer)
      mSubResistorC->connect({mVirtualNodes[2], mVirtualNodes[3]});
    else
      mSubResistorC->connect({mVirtualNodes[2], mTerminals[0]->node()});

    for (auto subcomp : mSubComponents) {
      subcomp->initialize(mFrequencies);
      subcomp->initializeFromNodesAndTerminals(frequency);
    }
  } else {
    mDirectGridCurrent = filterInterfaceInitialCurrent;
    // DP inductor/capacitor component voltage is terminal-1 minus terminal-0.
    mDirectInductorVoltage = vcInit - vfInit;
    mDirectInductorCurrent = filterInterfaceInitialCurrent - icfInit;
    mDirectCapacitorVoltage = -vcInit;
    mDirectCapacitorCurrent = -icfInit;
    if (mWithConnectionTransformer) {
      mSubCtrledVoltageSource->setParameters((**mVsref)(0, 0));
      mSubCtrledVoltageSource->connect({SimNode::GND, mVirtualNodes[0]});
    }
  }

  // Initialize control subcomponents
  // current and voltage inputs to PLL and power controller
  Complex vcdq, ircdq;
  // The filter-side virtual node is node 3 only when the optional connection
  // transformer is present. Without it, use the filter interface quantities
  // derived above; indexing virtual node 3 in that configuration is invalid.
  const Complex pllVoltage = mWithConnectionTransformer
                                 ? mVirtualNodes[3]->initialSingleVoltage()
                                 : filterInterfaceInitialVoltage;
  vcdq = Math::rotatingFrame2to1(pllVoltage, std::arg(pllVoltage), 0);
  const Complex initialGridCurrent = mUseSubcomponents
                                         ? (**mSubResistorC->mIntfCurrent)(0, 0)
                                         : mDirectGridCurrent;
  ircdq = Math::rotatingFrame2to1(-1. * initialGridCurrent,
                                  std::arg(pllVoltage), 0);
  **mVcd = vcdq.real();
  **mVcq = vcdq.imag();
  **mIrcd = ircdq.real();
  **mIrcq = ircdq.imag();
  const Complex vsdq = Math::rotatingFrame2to1(vsInit, std::arg(pllVoltage), 0);
  mPowerControllerVSI->initializeSteadyState(**mPref, **mQref, **mIrcd, **mIrcq,
                                             vsdq.real(), vsdq.imag());
  // angle input
  Matrix matrixStateInit = Matrix::Zero(2, 1);
  matrixStateInit(0, 0) = std::arg(pllVoltage);
  mPLL->setInitialValues(**mVcq, matrixStateInit, Matrix::Zero(2, 1));

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
      Logger::phasorToString((**mIntfVoltage)(0, 0)),
      Logger::phasorToString((**mIntfCurrent)(0, 0)),
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

void DP::Ph1::AvVoltageSourceInverterDQ::mnaParentInitialize(
    Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
  mTimeStep = timeStep;

  if (!mUseSubcomponents) {
    updateMatrixNodeIndices();
    const Real b = timeStep * omega / 2.;
    const Real a = timeStep / (2. * mLf);
    const Real denominator = 1. + b * b;
    mDirectInductorConductance = Complex(a / denominator, -a * b / denominator);
    mDirectInductorPreviousCurrentFactor =
        Complex((1. - b * b) / denominator, -2. * b / denominator);
    mDirectCapacitorConductance = Complex(2. * mCf / timeStep, omega * mCf);
    mDirectCapacitorPreviousVoltageFactor =
        Complex(2. * mCf / timeStep, -omega * mCf);
    mDirectInductorHistory =
        mDirectInductorConductance * mDirectInductorVoltage +
        mDirectInductorPreviousCurrentFactor * mDirectInductorCurrent;
    mDirectCapacitorHistory =
        -mDirectCapacitorCurrent -
        mDirectCapacitorPreviousVoltageFactor * mDirectCapacitorVoltage;

    if (mWithConnectionTransformer) {
      const UInt harmonicOffset =
          static_cast<UInt>(leftVector->get().rows()) / mNumFreqs;
      const UInt complexOffset = harmonicOffset / 2;
      const UInt vectorSize = static_cast<UInt>(leftVector->get().rows());
      auto appendComponentRows = [&](SimPowerComp<Complex> &component) {
        std::vector<UInt> baseIndices;
        for (UInt terminal = 0; terminal < component.terminalNumberConnected();
             ++terminal) {
          if (component.node(terminal)->isGround())
            continue;
          const auto indices = component.matrixNodeIndices(terminal);
          baseIndices.insert(baseIndices.end(), indices.begin(), indices.end());
        }
        for (UInt virtualNode = 0; virtualNode < component.virtualNodesNumber();
             ++virtualNode) {
          const auto indices = component.virtualMatrixNodeIndices(virtualNode);
          baseIndices.insert(baseIndices.end(), indices.begin(), indices.end());
        }
        for (UInt freq = 0; freq < mNumFreqs; ++freq) {
          for (UInt index : baseIndices) {
            const UInt realIndex = index + freq * harmonicOffset;
            const UInt imagIndex = realIndex + complexOffset;
            if (realIndex < vectorSize)
              mDirectChildRightVectorIndices.push_back(realIndex);
            if (imagIndex < vectorSize)
              mDirectChildRightVectorIndices.push_back(imagIndex);
          }
        }
      };
      appendComponentRows(*mConnectionTransformer);
      appendComponentRows(*mSubCtrledVoltageSource);
      std::sort(mDirectChildRightVectorIndices.begin(),
                mDirectChildRightVectorIndices.end());
      mDirectChildRightVectorIndices.erase(
          std::unique(mDirectChildRightVectorIndices.begin(),
                      mDirectChildRightVectorIndices.end()),
          mDirectChildRightVectorIndices.end());
    }
  }

  // initialize state space controller
  mPowerControllerVSI->initializeStateSpaceModel(omega, timeStep, leftVector);
  mPLL->setSimulationParameters(timeStep);

  // TODO: these are actually no MNA tasks
  mMnaTasks.push_back(std::make_shared<ControlPreStep>(*this));
  mMnaTasks.push_back(std::make_shared<ControlStep>(*this));
}

void DP::Ph1::AvVoltageSourceInverterDQ::addControlPreStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes) {
  // add pre-step dependencies of subcomponents
  mPLL->signalAddPreStepDependencies(prevStepDependencies,
                                     attributeDependencies, modifiedAttributes);
  mPowerControllerVSI->signalAddPreStepDependencies(
      prevStepDependencies, attributeDependencies, modifiedAttributes);
}

void DP::Ph1::AvVoltageSourceInverterDQ::controlPreStep(Real time,
                                                        Int timeStepCount) {
  // add pre-step of subcomponents
  mPLL->signalPreStep(time, timeStepCount);
  mPowerControllerVSI->signalPreStep(time, timeStepCount);
}

void DP::Ph1::AvVoltageSourceInverterDQ::addControlStepDependencies(
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

void DP::Ph1::AvVoltageSourceInverterDQ::controlStep(Real time,
                                                     Int timeStepCount) {
  // Transformation interface forward
  const Complex pllVoltage = mWithConnectionTransformer
                                 ? mVirtualNodes[3]->singleVoltage()
                                 : (**mIntfVoltage)(0, 0);
  Complex vcdq, ircdq;
  vcdq = Math::rotatingFrame2to1(
      pllVoltage, mPLL->attributeTyped<Matrix>("output_prev")->get()(0, 0),
      mThetaN);
  const Complex gridCurrent = mUseSubcomponents
                                  ? (**mSubResistorC->mIntfCurrent)(0, 0)
                                  : mDirectGridCurrent;
  ircdq = Math::rotatingFrame2to1(
      -1. * gridCurrent,
      mPLL->attributeTyped<Matrix>("output_prev")->get()(0, 0), mThetaN);
  **mVcd = vcdq.real();
  **mVcq = vcdq.imag();
  **mIrcd = ircdq.real();
  **mIrcq = ircdq.imag();

  // add step of subcomponents
  mPLL->signalStep(time, timeStepCount);
  mPowerControllerVSI->signalStep(time, timeStepCount);

  // Transformation interface backward
  (**mVsref)(0, 0) = Math::rotatingFrame2to1(
      Complex(mPowerControllerVSI->attributeTyped<Matrix>("output_curr")
                  ->get()(0, 0),
              mPowerControllerVSI->attributeTyped<Matrix>("output_curr")
                  ->get()(1, 0)),
      mThetaN, mPLL->attributeTyped<Matrix>("output_prev")->get()(0, 0));

  // Update nominal system angle
  mThetaN = mThetaN + mTimeStep * **mOmegaN;
}

void DP::Ph1::AvVoltageSourceInverterDQ::mnaParentApplySystemMatrixStamp(
    SparseMatrixRow &systemMatrix) {
  if (mUseSubcomponents) {
    return CompositePowerComp<Complex>::mnaParentApplySystemMatrixStamp(
        systemMatrix);
  }

  const Int sourceNode = mVirtualNodes[0]->matrixNodeIndex();
  const Int resistorNode = mVirtualNodes[1]->matrixNodeIndex();
  const Int capacitorNode = mVirtualNodes[2]->matrixNodeIndex();
  const Int sourceBranch = mVirtualNodes[3]->matrixNodeIndex();
  const Int terminal = matrixNodeIndex(0);
  const Int filterOutput = mWithConnectionTransformer
                               ? mVirtualNodes[3]->matrixNodeIndex()
                               : terminal;
  const Complex rfConductance(1. / mRf, 0.);
  const Complex rcConductance(1. / mRc, 0.);

  MNAStampUtils::stampAdmittance(rfConductance, systemMatrix, sourceNode,
                                 resistorNode, true, true, mSLog);
  MNAStampUtils::stampAdmittance(mDirectInductorConductance, systemMatrix,
                                 resistorNode, capacitorNode, true, true,
                                 mSLog);
  MNAStampUtils::stampAdmittance(mDirectCapacitorConductance, systemMatrix,
                                 capacitorNode, 0, true, false, mSLog);
  MNAStampUtils::stampAdmittance(
      rcConductance, systemMatrix, capacitorNode, filterOutput, true,
      mWithConnectionTransformer || terminalNotGrounded(0), mSLog);

  if (!mWithConnectionTransformer) {
    Math::setMatrixElement(systemMatrix, sourceBranch, sourceNode,
                           Complex(1., 0.));
    Math::setMatrixElement(systemMatrix, sourceNode, sourceBranch,
                           Complex(1., 0.));
  }
}

void DP::Ph1::AvVoltageSourceInverterDQ::mnaParentApplyRightSideVectorStamp(
    Matrix &rightVector) {
  if (mUseSubcomponents)
    return CompositePowerComp<Complex>::mnaParentApplyRightSideVectorStamp(
        rightVector);

  const Int sourceNode = mVirtualNodes[0]->matrixNodeIndex();
  const Int resistorNode = mVirtualNodes[1]->matrixNodeIndex();
  const Int capacitorNode = mVirtualNodes[2]->matrixNodeIndex();
  const Complex inductorEquivalent =
      mDirectInductorConductance * mDirectInductorVoltage +
      mDirectInductorPreviousCurrentFactor * mDirectInductorCurrent;
  const Complex capacitorEquivalent =
      -mDirectCapacitorCurrent -
      mDirectCapacitorPreviousVoltageFactor * mDirectCapacitorVoltage;
  if (!mWithConnectionTransformer)
    Math::setVectorElement(rightVector, mVirtualNodes[3]->matrixNodeIndex(),
                           mDirectSourceValue, mNumFreqs);
  Math::setVectorElement(rightVector, resistorNode, inductorEquivalent,
                         mNumFreqs);
  Math::setVectorElement(rightVector, capacitorNode,
                         capacitorEquivalent - inductorEquivalent, mNumFreqs);
}

void DP::Ph1::AvVoltageSourceInverterDQ::mnaAddCompactRightSideVectorStamp(
    Matrix &rightVector) {
  CompositePowerComp<Complex>::mnaAddCompactRightSideVectorStamp(rightVector);
  if (mUseSubcomponents || !mWithConnectionTransformer)
    return;

  // The direct filter realization keeps only its external voltage source and
  // connection transformer as MNA children. Unlike the legacy pre-step path,
  // the direct parent does not fold those child histories into its own vector.
  const Matrix &transformerStamp = **mConnectionTransformer->mRightVector;
  const Matrix &sourceStamp = **mSubCtrledVoltageSource->mRightVector;
  for (UInt index : mDirectChildRightVectorIndices) {
    rightVector(index, 0) += transformerStamp(index, 0);
    rightVector(index, 0) += sourceStamp(index, 0);
  }
}

void DP::Ph1::AvVoltageSourceInverterDQ::mnaParentAddPreStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes) {
  // add pre-step dependencies of component itself
  prevStepDependencies.push_back(mVsref);
  prevStepDependencies.push_back(mIntfCurrent);
  prevStepDependencies.push_back(mIntfVoltage);
  attributeDependencies.push_back(
      mPowerControllerVSI->attributeTyped<Matrix>("output_prev"));
  attributeDependencies.push_back(mPLL->attributeTyped<Matrix>("output_prev"));
  modifiedAttributes.push_back(mRightVector);
}

void DP::Ph1::AvVoltageSourceInverterDQ::mnaParentPreStep(Real time,
                                                          Int timeStepCount) {
  if (mUseSubcomponents) {
    if (mWithControl)
      **mSubCtrledVoltageSource->mVoltageRef = (**mVsref)(0, 0);
    std::dynamic_pointer_cast<MNAInterface>(mSubCtrledVoltageSource)
        ->mnaPreStep(time, timeStepCount);
  }
  // pre-step of component itself
  if (mUseSubcomponents)
    mnaCompApplyRightSideVectorStamp(**mRightVector);
  else {
    if (mWithConnectionTransformer) {
      if (mWithControl)
        **mSubCtrledVoltageSource->mVoltageRef = (**mVsref)(0, 0);
      mSubCtrledVoltageSource->mnaPreStep(time, timeStepCount);
    }
    if (mWithControl)
      mDirectSourceValue = (**mVsref)(0, 0);
    (**mDirectSourceVoltageAttr)(0, 0) = mDirectSourceValue;
    mnaParentApplyRightSideVectorStamp(**mRightVector);
  }
}

void DP::Ph1::AvVoltageSourceInverterDQ::mnaParentAddPostStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes,
    Attribute<Matrix>::Ptr &leftVector) {
  // add post-step dependencies of component itself
  attributeDependencies.push_back(leftVector);
  modifiedAttributes.push_back(mIntfVoltage);
  modifiedAttributes.push_back(mIntfCurrent);
}

void DP::Ph1::AvVoltageSourceInverterDQ::mnaParentPostStep(
    Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
  // post-step of component itself
  mnaCompUpdateCurrent(**leftVector);
  mnaCompUpdateVoltage(**leftVector);
}

void DP::Ph1::AvVoltageSourceInverterDQ::mnaCompUpdateCurrent(
    const Matrix &leftvector) {
  if (!mUseSubcomponents) {
    const Complex sourceVoltage = Math::complexFromVectorElement(
        leftvector, mVirtualNodes[0]->matrixNodeIndex());
    const Complex resistorVoltage = Math::complexFromVectorElement(
        leftvector, mVirtualNodes[1]->matrixNodeIndex());
    const Complex capacitorVoltage = Math::complexFromVectorElement(
        leftvector, mVirtualNodes[2]->matrixNodeIndex());
    const Complex terminalVoltage =
        terminalNotGrounded(0)
            ? Math::complexFromVectorElement(leftvector, matrixNodeIndex(0))
            : Complex(0., 0.);
    const Complex inductorVoltage = capacitorVoltage - resistorVoltage;
    const Complex inductorEquivalent =
        mDirectInductorConductance * mDirectInductorVoltage +
        mDirectInductorPreviousCurrentFactor * mDirectInductorCurrent;
    const Complex capacitorEquivalent =
        -mDirectCapacitorCurrent -
        mDirectCapacitorPreviousVoltageFactor * mDirectCapacitorVoltage;
    mDirectInductorCurrent =
        mDirectInductorConductance * inductorVoltage + inductorEquivalent;
    mDirectInductorVoltage = inductorVoltage;
    const Complex capacitorComponentVoltage = -capacitorVoltage;
    mDirectCapacitorCurrent =
        mDirectCapacitorConductance * capacitorComponentVoltage +
        capacitorEquivalent;
    mDirectCapacitorVoltage = capacitorComponentVoltage;
    const Complex filterOutputVoltage =
        mWithConnectionTransformer
            ? Math::complexFromVectorElement(
                  leftvector, mVirtualNodes[3]->matrixNodeIndex())
            : terminalVoltage;
    mDirectGridCurrent = (filterOutputVoltage - capacitorVoltage) / mRc;
    (**mIntfVoltage)(0, 0) = terminalVoltage;
    (**mIntfCurrent)(0, 0) =
        mWithConnectionTransformer
            ? (**mConnectionTransformer->mIntfCurrent)(0, 0)
            : mDirectGridCurrent;
    return;
  }
  if (mWithConnectionTransformer)
    **mIntfCurrent = **mConnectionTransformer->mIntfCurrent;
  else
    **mIntfCurrent = **mSubResistorC->mIntfCurrent;
}

void DP::Ph1::AvVoltageSourceInverterDQ::mnaCompUpdateVoltage(
    const Matrix &leftVector) {
  if (!mUseSubcomponents) {
    for (auto virtualNode : mVirtualNodes)
      virtualNode->mnaUpdateVoltage(leftVector);
    (**mIntfVoltage)(0, 0) =
        terminalNotGrounded(0)
            ? Math::complexFromVectorElement(leftVector, matrixNodeIndex(0))
            : Complex(0., 0.);
    return;
  }
  for (auto virtualNode : mVirtualNodes)
    virtualNode->mnaUpdateVoltage(leftVector);
  (**mIntfVoltage)(0, 0) =
      Math::complexFromVectorElement(leftVector, matrixNodeIndex(0));
}
