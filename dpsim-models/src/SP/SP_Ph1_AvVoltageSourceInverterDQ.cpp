/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/SP/SP_Ph1_AvVoltageSourceInverterDQ.h>

using namespace CPS;

SP::Ph1::AvVoltageSourceInverterDQ::AvVoltageSourceInverterDQ(String uid, String name, Logger::Level logLevel, Bool withTrafo) :
	CompositePowerComp<Complex>(uid, name, true, true, logLevel),
	mOmegaN(mAttributes->create<Real>("Omega_nom")),
	mVnom(mAttributes->create<Real>("vnom")),
	mPref(mAttributes->create<Real>("P_ref")),
	mQref(mAttributes->create<Real>("Q_ref")),
	mVcd(mAttributes->create<Real>("Vc_d", 0)),
	mVcq(mAttributes->create<Real>("Vc_q", 0)),
	mIrcd(mAttributes->create<Real>("Irc_d", 0)),
	mIrcq(mAttributes->create<Real>("Irc_q", 0)),
	mVsref(mAttributes->create<MatrixComp>("Vsref", MatrixComp::Zero(1,1))),
	mVs(mAttributes->createDynamic<MatrixComp>("Vs")),
	mPllOutput(mAttributes->createDynamic<Matrix>("pll_output")),
	mPowerctrlInputs(mAttributes->createDynamic<Matrix>("powerctrl_inputs")),
	mPowerctrlOutputs(mAttributes->createDynamic<Matrix>("powerctrl_outputs")),
	mPowerctrlStates(mAttributes->createDynamic<Matrix>("powerctrl_states")) {
	if (withTrafo) {
		setVirtualNodeNumber(4);
		mConnectionTransformer = SP::Ph1::Transformer::make(**mName + "_trans", **mName + "_trans", mLogLevel);
		addMNASubComponent(mConnectionTransformer, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);
	} else {
		setVirtualNodeNumber(3);
	}
	mWithConnectionTransformer = withTrafo;
	setTerminalNumber(1);

	SPDLOG_LOGGER_INFO(mSLog, "Create {} {}", type(), name);
	**mIntfVoltage = MatrixComp::Zero(1, 1);
	**mIntfCurrent = MatrixComp::Zero(1, 1);

	// Create electrical sub components
	mSubResistorF = SP::Ph1::Resistor::make(**mName + "_resF", mLogLevel);
	mSubResistorC = SP::Ph1::Resistor::make(**mName + "_resC", mLogLevel);
	mSubCapacitorF = SP::Ph1::Capacitor::make(**mName + "_capF", mLogLevel);
	mSubInductorF = SP::Ph1::Inductor::make(**mName + "_indF", mLogLevel);
	mSubCtrledVoltageSource = SP::Ph1::VoltageSource::make(**mName + "_src", mLogLevel);
	addMNASubComponent(mSubResistorF, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, false);
	addMNASubComponent(mSubResistorC, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, false);
	addMNASubComponent(mSubCapacitorF, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);
	addMNASubComponent(mSubInductorF, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);

	// Pre-step of the subcontrolled voltage source is handled explicitly in mnaParentPreStep
	addMNASubComponent(mSubCtrledVoltageSource, MNA_SUBCOMP_TASK_ORDER::NO_TASK, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);

	SPDLOG_LOGGER_INFO(mSLog, "Electrical subcomponents: ");
	for (auto subcomp: mSubComponents)
		SPDLOG_LOGGER_INFO(mSLog, "- {}", subcomp->name());

	// Create control sub components
	mPLL = Signal::PLL::make(**mName + "_PLL", mLogLevel);
	mPowerControllerVSI = Signal::PowerControllerVSI::make(**mName + "_PowerControllerVSI", mLogLevel);

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

void SP::Ph1::AvVoltageSourceInverterDQ::setParameters(Real sysOmega, Real sysVoltNom, Real Pref, Real Qref) {
	mParametersSet = true;

	SPDLOG_LOGGER_INFO(mSLog, "General Parameters:");
	SPDLOG_LOGGER_INFO(mSLog, "Nominal Voltage={} [V] Nominal Omega={} [1/s]", sysVoltNom, sysOmega);
	SPDLOG_LOGGER_INFO(mSLog, "Active Power={} [W] Reactive Power={} [VAr]", Pref, Qref);

	mPowerControllerVSI->setParameters(Pref, Qref);

	**mOmegaN = sysOmega;
	**mVnom = sysVoltNom;
	**mPref = Pref;
	**mQref = Qref;
}

void SP::Ph1::AvVoltageSourceInverterDQ::setTransformerParameters(Real nomVoltageEnd1, Real nomVoltageEnd2, Real ratedPower,
	Real ratioAbs, Real ratioPhase, Real resistance, Real inductance) {

	Base::AvVoltageSourceInverterDQ::setTransformerParameters(nomVoltageEnd1, nomVoltageEnd2, ratedPower,
		ratioAbs, ratioPhase, resistance, inductance);

	SPDLOG_LOGGER_INFO(mSLog, "Connection Transformer Parameters:");
	SPDLOG_LOGGER_INFO(mSLog, "Nominal Voltage End 1={} [V] Nominal Voltage End 2={} [V]", mTransformerNominalVoltageEnd1, mTransformerNominalVoltageEnd2);
	SPDLOG_LOGGER_INFO(mSLog, "Rated Apparent Power = {} [VA]", mTransformerRatedPower);
	SPDLOG_LOGGER_INFO(mSLog, "Resistance={} [Ohm] Inductance={} [H]", mTransformerResistance, mTransformerInductance);
    SPDLOG_LOGGER_INFO(mSLog, "Tap Ratio={} [ ] Phase Shift={} [deg]", mTransformerRatioAbs, mTransformerRatioPhase);

	if (mWithConnectionTransformer)
		mConnectionTransformer->setParameters(mTransformerNominalVoltageEnd1, mTransformerNominalVoltageEnd2, mTransformerRatioAbs, mTransformerRatioPhase, mTransformerResistance, mTransformerInductance);
}

void SP::Ph1::AvVoltageSourceInverterDQ::setControllerParameters(Real Kp_pll, Real Ki_pll,
	Real Kp_powerCtrl, Real Ki_powerCtrl, Real Kp_currCtrl, Real Ki_currCtrl, Real Omega_cutoff) {

	SPDLOG_LOGGER_INFO(mSLog, "Control Parameters:");
	SPDLOG_LOGGER_INFO(mSLog, "PLL: K_p = {}, K_i = {}, Omega_Nom = {}", Kp_pll, Ki_pll, Omega_cutoff);
	SPDLOG_LOGGER_INFO(mSLog, "Power Loop: K_p = {}, K_i = {}", Kp_powerCtrl, Ki_powerCtrl);
	SPDLOG_LOGGER_INFO(mSLog, "Current Loop: K_p = {}, K_i = {}", Kp_currCtrl, Ki_currCtrl);
	SPDLOG_LOGGER_INFO(mSLog, "Cut-Off Frequency = {}", Omega_cutoff);

	// TODO: add and use Omega_nominal instead of Omega_cutoff
	mPLL->setParameters(Kp_pll, Ki_pll, Omega_cutoff);
	mPLL->composeStateSpaceMatrices();
	mPowerControllerVSI->setControllerParameters(Kp_powerCtrl, Ki_powerCtrl, Kp_currCtrl, Ki_currCtrl, Omega_cutoff);
}

void SP::Ph1::AvVoltageSourceInverterDQ::setFilterParameters(Real Lf, Real Cf, Real Rf, Real Rc) {
	Base::AvVoltageSourceInverterDQ::setFilterParameters(Lf, Cf, Rf, Rc);

	SPDLOG_LOGGER_INFO(mSLog, "Filter Parameters:");
	SPDLOG_LOGGER_INFO(mSLog, "Inductance Lf={} [H] Capacitance Cf={} [F]", mLf, mCf);
	SPDLOG_LOGGER_INFO(mSLog, "Resistance Rf={} [Ohm] Resistance Rc={} [Ohm]", mRf, mRc);

	mSubResistorC->setParameters(mRc);
	mSubResistorF->setParameters(mRf);
	mSubInductorF->setParameters(mLf);
	mSubCapacitorF->setParameters(mCf);
}

void SP::Ph1::AvVoltageSourceInverterDQ::setInitialStateValues(Real pInit, Real qInit,
	Real phi_dInit, Real phi_qInit, Real gamma_dInit, Real gamma_qInit) {

	SPDLOG_LOGGER_INFO(mSLog, "Initial State Value Parameters:");
	SPDLOG_LOGGER_INFO(mSLog, "PInit = {}, QInit = {}", pInit, qInit);
	SPDLOG_LOGGER_INFO(mSLog, "Phi_dInit = {}, Phi_qInit = {}", phi_dInit, phi_qInit);
	SPDLOG_LOGGER_INFO(mSLog, "Gamma_dInit = {}, Gamma_qInit = {}", gamma_dInit, gamma_qInit);

	mPowerControllerVSI->setInitialStateValues(pInit, qInit, phi_dInit, phi_qInit, gamma_dInit, gamma_qInit);
}

void SP::Ph1::AvVoltageSourceInverterDQ::initializeFromNodesAndTerminals(Real frequency) {

	// set initial interface quantities
	(**mIntfVoltage)(0, 0) = initialSingleVoltage(0);
	(**mIntfCurrent)(0, 0) = - std::conj(Complex(**mPref, **mQref) / (**mIntfVoltage)(0,0));

	Complex filterInterfaceInitialVoltage;
	Complex filterInterfaceInitialCurrent;

	if (mWithConnectionTransformer) {
		// calculate quantities of low voltage side of transformer (being the interface quantities of the filter)
		filterInterfaceInitialVoltage = ((**mIntfVoltage)(0, 0) - Complex(mTransformerResistance, mTransformerInductance * **mOmegaN)* (**mIntfCurrent)(0, 0)) / Complex(mTransformerRatioAbs, mTransformerRatioPhase);
		filterInterfaceInitialCurrent = (**mIntfCurrent)(0, 0) * Complex(mTransformerRatioAbs, mTransformerRatioPhase);

		// connect transformer
		mVirtualNodes[3]->setInitialVoltage(filterInterfaceInitialVoltage);
		mConnectionTransformer->connect({ mTerminals[0]->node(), mVirtualNodes[3] });
	} else {
		// if no transformer used, filter interface equal to inverter interface
		filterInterfaceInitialVoltage = (**mIntfVoltage)(0, 0);
		filterInterfaceInitialCurrent = (**mIntfCurrent)(0, 0);
	}

	// derive initialization quantities of filter
	Complex vcInit = filterInterfaceInitialVoltage - filterInterfaceInitialCurrent * mRc;
	Complex icfInit = vcInit * Complex(0., 2. * PI * frequency * mCf);
	Complex vfInit = vcInit - (filterInterfaceInitialCurrent - icfInit) * Complex(0., 2. * PI * frequency * mLf);
	Complex vsInit = vfInit - (filterInterfaceInitialCurrent - icfInit) * Complex(mRf, 0);
	mVirtualNodes[0]->setInitialVoltage(vsInit);
	mVirtualNodes[1]->setInitialVoltage(vfInit);
	mVirtualNodes[2]->setInitialVoltage(vcInit);

	// Set parameters electrical subcomponents
	(**mVsref)(0,0) = mVirtualNodes[0]->initialSingleVoltage();
	mSubCtrledVoltageSource->setParameters((**mVsref)(0,0));

	// Connect electrical subcomponents
	mSubCtrledVoltageSource->connect({ SimNode::GND, mVirtualNodes[0] });
	mSubResistorF->connect({ mVirtualNodes[0], mVirtualNodes[1] });
	mSubInductorF->connect({ mVirtualNodes[1], mVirtualNodes[2] });
	mSubCapacitorF->connect({ mVirtualNodes[2], SimNode::GND });
	if (mWithConnectionTransformer)
		mSubResistorC->connect({ mVirtualNodes[2],  mVirtualNodes[3]});
	else
		mSubResistorC->connect({ mVirtualNodes[2],  mTerminals[0]->node()});

	// Initialize electrical subcomponents
	for (auto subcomp: mSubComponents) {
		subcomp->initialize(mFrequencies);
		subcomp->initializeFromNodesAndTerminals(frequency);
	}

	// Initialize control subcomponents
	// current and voltage inputs to PLL and power controller
	Complex vcdq, ircdq;
	vcdq = Math::rotatingFrame2to1(mVirtualNodes[3]->initialSingleVoltage(), std::arg(mVirtualNodes[3]->initialSingleVoltage()), 0);
	ircdq = Math::rotatingFrame2to1(-1. * mSubResistorC->mIntfCurrent->get()(0, 0), std::arg(mVirtualNodes[3]->initialSingleVoltage()), 0);
	**mVcd = vcdq.real();
	**mVcq = vcdq.imag();
	**mIrcd = ircdq.real();
	**mIrcq = ircdq.imag();
	// angle input
	Matrix matrixStateInit = Matrix::Zero(2,1);
	matrixStateInit(0,0) = std::arg(mVirtualNodes[3]->initialSingleVoltage());
	mPLL->setInitialValues(**mVcq, matrixStateInit, Matrix::Zero(2,1));

	SPDLOG_LOGGER_INFO(mSLog,
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
			SPDLOG_LOGGER_INFO(mSLog, "\nVirtual node 3 initial voltage: {:s}", Logger::phasorToString(mVirtualNodes[3]->initialSingleVoltage()));
		SPDLOG_LOGGER_INFO(mSLog, "\n--- Initialization from powerflow finished ---");
}

void SP::Ph1::AvVoltageSourceInverterDQ::mnaParentInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
	mTimeStep = timeStep;

	// initialize state space controller
	mPowerControllerVSI->initializeStateSpaceModel(omega, timeStep, leftVector);
	mPLL->setSimulationParameters(timeStep);

	// TODO: these are actually no MNA tasks
	mMnaTasks.push_back(std::make_shared<ControlPreStep>(*this));
	mMnaTasks.push_back(std::make_shared<ControlStep>(*this));
}

void SP::Ph1::AvVoltageSourceInverterDQ::addControlPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) const {
	// add pre-step dependencies of subcomponents
	mPLL->signalAddPreStepDependencies(prevStepDependencies, attributeDependencies, modifiedAttributes);
	mPowerControllerVSI->signalAddPreStepDependencies(prevStepDependencies, attributeDependencies, modifiedAttributes);
}

void SP::Ph1::AvVoltageSourceInverterDQ::controlPreStep(Real time, Int timeStepCount) {
	// add pre-step of subcomponents
	mPLL->signalPreStep(time, timeStepCount);
	mPowerControllerVSI->signalPreStep(time, timeStepCount);
}

void SP::Ph1::AvVoltageSourceInverterDQ::addControlStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) const {
	// add step dependencies of subcomponents
	mPLL->signalAddStepDependencies(prevStepDependencies, attributeDependencies, modifiedAttributes);
	mPowerControllerVSI->signalAddStepDependencies(prevStepDependencies, attributeDependencies, modifiedAttributes);
	// add step dependencies of component itself
	attributeDependencies.push_back(mIntfCurrent);
	attributeDependencies.push_back(mIntfVoltage);
	modifiedAttributes.push_back(mVsref);
}

void SP::Ph1::AvVoltageSourceInverterDQ::controlStep(Real time, Int timeStepCount) {
	// Transformation interface forward
	Complex vcdq, ircdq;
	vcdq = Math::rotatingFrame2to1(mVirtualNodes[3]->singleVoltage(), mPLL->attributeTyped<Matrix>("output_prev")->get()(0, 0), mThetaN);
	ircdq = Math::rotatingFrame2to1(-1. * mSubResistorC->attributeTyped<MatrixComp>("i_intf")->get()(0, 0), mPLL->attributeTyped<Matrix>("output_prev")->get()(0, 0), mThetaN);
	**mVcd = vcdq.real();
	**mVcq = vcdq.imag();
	**mIrcd = ircdq.real();
	**mIrcq = ircdq.imag();

	// add step of subcomponents
	mPLL->signalStep(time, timeStepCount);
	mPowerControllerVSI->signalStep(time, timeStepCount);

	// Transformation interface backward
	(**mVsref)(0,0) = Math::rotatingFrame2to1(Complex(mPowerControllerVSI->attributeTyped<Matrix>("output_curr")->get()(0, 0), mPowerControllerVSI->attributeTyped<Matrix>("output_curr")->get()(1, 0)), mThetaN, mPLL->attributeTyped<Matrix>("output_prev")->get()(0, 0));

	// Update nominal system angle
	mThetaN = mThetaN + mTimeStep * **mOmegaN;
}

void SP::Ph1::AvVoltageSourceInverterDQ::mnaParentAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) {
	prevStepDependencies.push_back(mVsref);
	prevStepDependencies.push_back(mIntfCurrent);
	prevStepDependencies.push_back(mIntfVoltage);
	attributeDependencies.push_back(mPowerControllerVSI->attributeTyped<Matrix>("output_prev"));
	attributeDependencies.push_back(mPLL->attributeTyped<Matrix>("output_prev"));
	modifiedAttributes.push_back(mRightVector);
}

void SP::Ph1::AvVoltageSourceInverterDQ::mnaParentPreStep(Real time, Int timeStepCount) {
	// pre-steo of subcomponents - controlled source
	if (mWithControl)
		mSubCtrledVoltageSource->mVoltageRef->set((**mVsref)(0,0));

	std::dynamic_pointer_cast<MNAInterface>(mSubCtrledVoltageSource)->mnaPreStep(time, timeStepCount);
	// pre-step of component itself
	mnaCompApplyRightSideVectorStamp(**mRightVector);
}

void SP::Ph1::AvVoltageSourceInverterDQ::mnaParentAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector) {
	attributeDependencies.push_back(leftVector);
	modifiedAttributes.push_back(mIntfVoltage);
	modifiedAttributes.push_back(mIntfCurrent);
}

void SP::Ph1::AvVoltageSourceInverterDQ::mnaParentPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
	mnaCompUpdateCurrent(**leftVector);
	mnaCompUpdateVoltage(**leftVector);
}

void SP::Ph1::AvVoltageSourceInverterDQ::mnaCompUpdateCurrent(const Matrix& leftvector) {
	if (mWithConnectionTransformer)
		**mIntfCurrent = mConnectionTransformer->attributeTyped<MatrixComp>("i_intf")->get();
	else
		**mIntfCurrent = mSubResistorC->attributeTyped<MatrixComp>("i_intf")->get();
}

void SP::Ph1::AvVoltageSourceInverterDQ::mnaCompUpdateVoltage(const Matrix& leftVector) {
	for (auto virtualNode : mVirtualNodes)
		virtualNode->mnaUpdateVoltage(leftVector);
	(**mIntfVoltage)(0,0) = Math::complexFromVectorElement(leftVector, matrixNodeIndex(0));
}
