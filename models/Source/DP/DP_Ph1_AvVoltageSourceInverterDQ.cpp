/* Copyright 2017-2020 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <cps/DP/DP_Ph1_AvVoltageSourceInverterDQ.h>

using namespace CPS;


DP::Ph1::AvVoltageSourceInverterDQ::AvVoltageSourceInverterDQ(String uid, String name, Logger::Level logLevel, Bool withTrafo) :
	SimPowerComp<Complex>(uid, name, logLevel) {
	if (withTrafo) {
		setVirtualNodeNumber(5);
		mConnectionTransformer = DP::Ph1::Transformer::make(mName + "_trans", Logger::Level::debug);
		mSubComponents.push_back(mConnectionTransformer);
	} else {
		setVirtualNodeNumber(4);
	}
	mWithConnectionTransformer = withTrafo;
	setTerminalNumber(1);

	mSLog->info("Create {} {}", type(), name);
	mIntfVoltage = MatrixComp::Zero(1, 1);
	mIntfCurrent = MatrixComp::Zero(1, 1);

	// Create electrical sub components
	mSubResistorF = DP::Ph1::Resistor::make(mName + "_resF", mLogLevel);
	mSubResistorC = DP::Ph1::Resistor::make(mName + "_resC", mLogLevel);
	mSubCapacitorF = DP::Ph1::Capacitor::make(mName + "_capF", mLogLevel);
	mSubInductorF = DP::Ph1::Inductor::make(mName + "_indF", mLogLevel);
	mSubCtrledVoltageSource = DP::Ph1::VoltageSource::make(mName + "_src", mLogLevel);

	// Create control sub components
	mPLL = Signal::PLL::make("pv_PLL", mLogLevel);
	mPowerControllerVSI = Signal::PowerControllerVSI::make("pv_PowerControllerVSI", mLogLevel);

	// additional input variables
	addAttribute<Matrix>("Vcdq", &mVcdq, Flags::read | Flags::write);
	addAttribute<Matrix>("Ircdq", &mIrcdq, Flags::read | Flags::write);

	// additional output variables
	addAttribute<Matrix>("Vsdq", &mVsdq, Flags::read | Flags::write);
	addAttribute<MatrixComp>("Vsref", &mVsref, Flags::read | Flags::write);

	// state variables
	addAttribute<Real>("p", Flags::read | Flags::write);
	addAttribute<Real>("q", Flags::read | Flags::write);
	addAttribute<Real>("phid", Flags::read | Flags::write);
	addAttribute<Real>("phiq", Flags::read | Flags::write);
	addAttribute<Real>("gammad", Flags::read | Flags::write);
	addAttribute<Real>("gammaq", Flags::read | Flags::write);

	// input variables
	addAttribute<Real>("Omega_nom", &mOmegaN, Flags::read | Flags::write);
	addAttribute<Real>("P_ref", &mPref, Flags::read | Flags::write);
	addAttribute<Real>("Q_ref", &mQref, Flags::read | Flags::write);

	// PLL
	addAttribute<Matrix>("pll_output", Flags::read | Flags::write);

	// interfacing variabes
	addAttribute<Real>("Vc_d", &mVcdq(0, 0), Flags::read | Flags::write);
	addAttribute<Real>("Vc_q", &mVcdq(1, 0), Flags::read | Flags::write);
	addAttribute<Real>("Irc_d", &mIrcdq(0, 0), Flags::read | Flags::write);
	addAttribute<Real>("Irc_q", &mIrcdq(1, 0), Flags::read | Flags::write);
	addAttribute<Real>("Vs_d", &mVsdq(0, 0), Flags::read | Flags::write);
	addAttribute<Real>("Vs_q", &mVsdq(1, 0), Flags::read | Flags::write);

	// Relate component attributes with subcomponents attributes	
	// PLL
	mPLL->setAttributeRef("input_ref", attribute<Real>("Vc_q"));
	setAttributeRef("pll_output", mPLL->attribute<Matrix>("output_curr"));

	// Power controller
	mPowerControllerVSI->setAttributeRef("Vc_d", attribute<Real>("Vc_d"));
	mPowerControllerVSI->setAttributeRef("Vc_q", attribute<Real>("Vc_q"));
	mPowerControllerVSI->setAttributeRef("Irc_d", attribute<Real>("Irc_d"));
	mPowerControllerVSI->setAttributeRef("Irc_q", attribute<Real>("Irc_q"));
	mPowerControllerVSI->setAttributeRef("Vs_d", attribute<Real>("Vs_d"));
	mPowerControllerVSI->setAttributeRef("Vs_q", attribute<Real>("Vs_q"));
	setAttributeRef("p", mPowerControllerVSI->attribute<Real>("p"));
	setAttributeRef("q", mPowerControllerVSI->attribute<Real>("q"));
	setAttributeRef("phid", mPowerControllerVSI->attribute<Real>("phid"));
	setAttributeRef("phiq", mPowerControllerVSI->attribute<Real>("phiq"));
	setAttributeRef("gammad", mPowerControllerVSI->attribute<Real>("gammad"));
	setAttributeRef("gammaq", mPowerControllerVSI->attribute<Real>("gammaq"));
}

void DP::Ph1::AvVoltageSourceInverterDQ::setParameters(Real sysOmega, Real sysVoltNom, Real Pref, Real Qref) {
	parametersSet = true;

	mSLog->info("General Parameters:");
	mSLog->info("Nominal Voltage={} [V] Nominal Omega={} [1/s]", sysVoltNom, sysOmega);
	mSLog->info("Active Power={} [W] Reactive Power={} [VAr]", Pref, Qref);

	mPowerControllerVSI->setParameters(Pref, Qref);

	mOmegaN = sysOmega;
	mVnom = sysVoltNom;
	mPref = Pref;
	mQref = Qref;
}

void DP::Ph1::AvVoltageSourceInverterDQ::setTransformerParameters(Real nomVoltageEnd1, Real nomVoltageEnd2,
	Real ratedPower, Real ratioAbs,	Real ratioPhase, Real resistance, Real inductance, Real omega) {

	Base::AvVoltageSourceInverterDQ::setTransformerParameters(nomVoltageEnd1, nomVoltageEnd2,
		ratedPower, ratioAbs, ratioPhase, resistance, inductance, omega);

	mSLog->info("Connection Transformer Parameters:");
	mSLog->info("Resistance={} [Ohm] Inductance={} [H]", mTransformerResistance, mTransformerInductance);
    mSLog->info("Tap Ratio={} [ ] Phase Shift={} [deg]", mTransformerRatioAbs, mTransformerRatioPhase);

	if (mWithConnectionTransformer)
		mConnectionTransformer->setParameters(mTransformerRatioAbs, mTransformerRatioPhase, mTransformerResistance, mTransformerInductance);
}

void DP::Ph1::AvVoltageSourceInverterDQ::setControllerParameters(Real Kp_pll, Real Ki_pll,
	Real Kp_powerCtrl, Real Ki_powerCtrl, Real Kp_currCtrl, Real Ki_currCtrl, Real Omega_cutoff) {

	mSLog->info("Control Parameters:");
	mSLog->info("PLL: K_i = {}, K_p = {}, Omega_Nom = {}", Kp_pll, Ki_pll, Omega_cutoff);
	mSLog->info("Power Loop: K_i = {}, K_p = {}", Kp_powerCtrl, Ki_powerCtrl);
	mSLog->info("Current Loop: K_i = {}, K_p = {}", Kp_currCtrl, Ki_currCtrl);
	mSLog->info("Cut-Off Frequency = {}", Omega_cutoff);

	// TODO: add and use Omega_nominal instead of Omega_cutoff
	mPLL->setParameters(Kp_pll, Ki_pll, Omega_cutoff);
	mPLL->composeStateSpaceMatrices();
	mPowerControllerVSI->setControllerParameters(Kp_powerCtrl, Ki_powerCtrl, Kp_currCtrl, Ki_currCtrl, Omega_cutoff);
}

void DP::Ph1::AvVoltageSourceInverterDQ::setFilterParameters(Real Lf, Real Cf, Real Rf, Real Rc) {
	Base::AvVoltageSourceInverterDQ::setFilterParameters(Lf, Cf, Rf, Rc);

	mSLog->info("Filter Parameters:");
	mSLog->info("Inductance Lf={} [H] Capacitance Cf={} [F]", mLf, mCf);
	mSLog->info("Resistance Rf={} [H] Resistance Rc={} [F]", mRf, mRc);

	mSubResistorC->setParameters(mRc);
	mSubResistorF->setParameters(mRf);
	mSubInductorF->setParameters(mLf);
	mSubCapacitorF->setParameters(mCf);
}

void DP::Ph1::AvVoltageSourceInverterDQ::setInitialStateValues(Real pInit, Real qInit,
	Real phi_dInit, Real phi_qInit, Real gamma_dInit, Real gamma_qInit) {

	mSLog->info("Initial State Value Parameters:");
	mSLog->info("PInit = {}, QInit = {}", pInit, qInit);
	mSLog->info("Phi_dInit = {}, Phi_qInit = {}", phi_dInit, phi_qInit);
	mSLog->info("Gamma_dInit = {}, Gamma_qInit = {}", gamma_dInit, gamma_qInit);

	mPowerControllerVSI->setInitialStateValues(pInit, qInit, phi_dInit, phi_qInit, gamma_dInit, gamma_qInit);
}

void DP::Ph1::AvVoltageSourceInverterDQ::initialize(Matrix frequencies) {
	SimPowerComp<Complex>::initialize(frequencies);
}


Complex DP::Ph1::AvVoltageSourceInverterDQ::rotatingFrame2to1(Complex f2, Real theta1, Real theta2) {
	Real delta = theta2 - theta1;
	Real f1_real = f2.real() * cos(delta) - f2.imag() * sin(delta);
	Real f1_imag = f2.real() * sin(delta) + f2.imag() * cos(delta);
	return Complex(f1_real, f1_imag);
}

void DP::Ph1::AvVoltageSourceInverterDQ::initializeFromPowerflow(Real frequency) {
	checkForUnconnectedTerminals();

	// set initial interface quantities
	mIntfVoltage(0, 0) = initialSingleVoltage(0);
	mIntfCurrent(0, 0) = - std::conj(Complex(mPref, mQref) / mIntfVoltage(0,0));

	Complex filterInterfaceInitialVoltage;
	Complex filterInterfaceInitialCurrent;

	if (mWithConnectionTransformer) {
		// calculate quantities of low voltage side of transformer (being the interface quantities of the filter)
		filterInterfaceInitialVoltage = (mIntfVoltage(0, 0) - Complex(mTransformerResistance, mTransformerInductance*mOmegaN)*mIntfCurrent(0, 0)) / Complex(mTransformerRatioAbs, mTransformerRatioPhase);
		filterInterfaceInitialCurrent = mIntfCurrent(0, 0) * Complex(mTransformerRatioAbs, mTransformerRatioPhase);

		// connect and init transformer
		mVirtualNodes[4]->setInitialVoltage(filterInterfaceInitialVoltage);
		mConnectionTransformer->connect({ mTerminals[0]->node(), mVirtualNodes[4] });
		mConnectionTransformer->initialize(mFrequencies);
		mConnectionTransformer->initializeFromPowerflow(frequency);
	} else {
		// if no transformer used, filter interface equal to inverter interface
		filterInterfaceInitialVoltage = mIntfVoltage(0, 0);
		filterInterfaceInitialCurrent = mIntfCurrent(0, 0);
	}

	// derive initialization quantities of filter
	Complex vcInit = filterInterfaceInitialVoltage - filterInterfaceInitialCurrent * mRc;
	Complex icfInit = vcInit * Complex(0., 2. * PI * frequency * mCf);
	Complex vfInit = vcInit - (filterInterfaceInitialCurrent - icfInit) * Complex(0., 2. * PI * frequency * mLf);
	Complex vsInit = vfInit - (filterInterfaceInitialCurrent - icfInit) * Complex(mRf, 0);
	mVirtualNodes[1]->setInitialVoltage(vsInit);
	mVirtualNodes[2]->setInitialVoltage(vfInit);
	mVirtualNodes[3]->setInitialVoltage(vcInit);

	// Set parameters electrical subcomponents
	mVsref(0,0) = mVirtualNodes[1]->initialSingleVoltage();
	mSubCtrledVoltageSource->setParameters(mVsref(0,0));

	// Connect electrical subcomponents
	mSubCtrledVoltageSource->connect({ SimNode::GND, mVirtualNodes[1] });
	mSubCtrledVoltageSource->setVirtualNodeAt(mVirtualNodes[0], 0);
	mSubResistorF->connect({ mVirtualNodes[1], mVirtualNodes[2] });
	mSubInductorF->connect({ mVirtualNodes[2], mVirtualNodes[3] });
	mSubCapacitorF->connect({ mVirtualNodes[3], SimNode::GND });
	if (mWithConnectionTransformer)
		mSubResistorC->connect({ mVirtualNodes[3],  mVirtualNodes[4]});
	else
		mSubResistorC->connect({ mVirtualNodes[3],  mTerminals[0]->node()});

	// Initialize electrical subcomponents
	mSubCtrledVoltageSource->initialize(mFrequencies);
	mSubResistorF->initialize(mFrequencies);
	mSubInductorF->initialize(mFrequencies);
	mSubCapacitorF->initialize(mFrequencies);
	mSubResistorC->initialize(mFrequencies);

	mSubCtrledVoltageSource->initializeFromPowerflow(frequency);
	mSubResistorF->initializeFromPowerflow(frequency);
	mSubInductorF->initializeFromPowerflow(frequency);
	mSubCapacitorF->initializeFromPowerflow(frequency);
	mSubResistorC->initializeFromPowerflow(frequency);

	// Initialize control subcomponents
	// current and voltage inputs to PLL and power controller
	Complex vcdq, ircdq;
	vcdq = rotatingFrame2to1(mVirtualNodes[4]->initialSingleVoltage(), std::arg(mVirtualNodes[4]->initialSingleVoltage()), 0);
	ircdq = rotatingFrame2to1(-1. * mSubResistorC->attribute<MatrixComp>("i_intf")->get()(0, 0), std::arg(mVirtualNodes[4]->initialSingleVoltage()), 0);
	mVcdq(0, 0) = vcdq.real();
	mVcdq(1, 0) = vcdq.imag();
	mIrcdq(0, 0) = ircdq.real();
	mIrcdq(1, 0) = ircdq.imag();
	// angle input 
	Matrix matrixStateInit = Matrix::Zero(2,1);
	matrixStateInit(0,0) = std::arg(mVirtualNodes[4]->initialSingleVoltage());
	mPLL->setInitialValues(mVcdq(1, 0), matrixStateInit, Matrix::Zero(2,1));

	mSLog->info(
		"\n--- Initialization from powerflow ---"
		"\nVoltage across: {:s}"
		"\nCurrent: {:s}"
		"\nTerminal 0 voltage: {:s}"
		"\nTerminal 0 connected to {:s} = sim node {:d}"
		"\n--- Initialization from powerflow finished ---",
		Logger::phasorToString(mIntfVoltage(0, 0)),
		Logger::phasorToString(mIntfCurrent(0, 0)),
		Logger::phasorToString(initialSingleVoltage(0)),
		mTerminals[0]->node()->name(), mTerminals[0]->node()->matrixNodeIndex());
}

void DP::Ph1::AvVoltageSourceInverterDQ::mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
	MNAInterface::mnaInitialize(omega, timeStep);
	updateMatrixNodeIndices();
	mTimeStep = timeStep;

	// initialize subcomponents
	mSubResistorF->mnaInitialize(omega, timeStep, leftVector);
	mSubInductorF->mnaInitialize(omega, timeStep, leftVector);
	mSubCapacitorF->mnaInitialize(omega, timeStep, leftVector);
	mSubResistorC->mnaInitialize(omega, timeStep, leftVector);
	mSubCtrledVoltageSource->mnaInitialize(omega, timeStep, leftVector);
	if (mWithConnectionTransformer)
		mConnectionTransformer->mnaInitialize(omega, timeStep, leftVector);

	// initialize state space controller
	mPowerControllerVSI->initializeStateSpaceModel(omega, timeStep, leftVector);
	mPLL->setSimulationParameters(timeStep);

	// collect right side vectors of subcomponents
	mRightVectorStamps.push_back(&mSubCapacitorF->attribute<Matrix>("right_vector")->get());
	mRightVectorStamps.push_back(&mSubInductorF->attribute<Matrix>("right_vector")->get());
	mRightVectorStamps.push_back(&mSubCtrledVoltageSource->attribute<Matrix>("right_vector")->get());
	if (mWithConnectionTransformer)
		mRightVectorStamps.push_back(&mConnectionTransformer->attribute<Matrix>("right_vector")->get());

	// collect tasks
	mMnaTasks.push_back(std::make_shared<MnaPreStep>(*this));
	mMnaTasks.push_back(std::make_shared<MnaPostStep>(*this, leftVector));

	// TODO: these are actually no MNA tasks
	mMnaTasks.push_back(std::make_shared<ControlPreStep>(*this));
	mMnaTasks.push_back(std::make_shared<ControlStep>(*this));

	mRightVector = Matrix::Zero(leftVector->get().rows(), 1);
}


void DP::Ph1::AvVoltageSourceInverterDQ::mnaApplySystemMatrixStamp(Matrix& systemMatrix) {
	mSubCtrledVoltageSource->mnaApplySystemMatrixStamp(systemMatrix);
	mSubResistorF->mnaApplySystemMatrixStamp(systemMatrix);
	mSubInductorF->mnaApplySystemMatrixStamp(systemMatrix);
	mSubCapacitorF->mnaApplySystemMatrixStamp(systemMatrix);
	mSubResistorC->mnaApplySystemMatrixStamp(systemMatrix);
	if (mWithConnectionTransformer)
		mConnectionTransformer->mnaApplySystemMatrixStamp(systemMatrix);
}

void DP::Ph1::AvVoltageSourceInverterDQ::mnaApplyRightSideVectorStamp(Matrix& rightVector) {
	rightVector.setZero();
	for (auto stamp : mRightVectorStamps)
		rightVector += *stamp;
}

void DP::Ph1::AvVoltageSourceInverterDQ::addControlPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) {
	// add pre-step dependencies of subcomponents
	mPLL->signalAddPreStepDependencies(prevStepDependencies, attributeDependencies, modifiedAttributes);
	mPowerControllerVSI->signalAddPreStepDependencies(prevStepDependencies, attributeDependencies, modifiedAttributes);
}

void DP::Ph1::AvVoltageSourceInverterDQ::controlPreStep(Real time, Int timeStepCount) {	
	// add pre-step of subcomponents
	mPLL->signalPreStep(time, timeStepCount);
	mPowerControllerVSI->signalPreStep(time, timeStepCount);
}

void DP::Ph1::AvVoltageSourceInverterDQ::addControlStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) {
	// add step dependencies of subcomponents
	mPLL->signalAddStepDependencies(prevStepDependencies, attributeDependencies, modifiedAttributes);
	mPowerControllerVSI->signalAddStepDependencies(prevStepDependencies, attributeDependencies, modifiedAttributes);
	// add step dependencies of component itself
	attributeDependencies.push_back(attribute("i_intf"));
	attributeDependencies.push_back(attribute("v_intf"));
	modifiedAttributes.push_back(attribute("Vsref"));
}

void DP::Ph1::AvVoltageSourceInverterDQ::controlStep(Real time, Int timeStepCount) {
	// Transformation interface forward
	Complex vcdq, ircdq;
	vcdq = rotatingFrame2to1(mVirtualNodes[4]->singleVoltage(), mPLL->attribute<Matrix>("output_prev")->get()(0, 0), mThetaN);
	ircdq = rotatingFrame2to1(-1. * mSubResistorC->attribute<MatrixComp>("i_intf")->get()(0, 0), mPLL->attribute<Matrix>("output_prev")->get()(0, 0), mThetaN);
	mVcdq(0, 0) = vcdq.real();
	mVcdq(1, 0) = vcdq.imag();
	mIrcdq(0, 0) = ircdq.real();
	mIrcdq(1, 0) = ircdq.imag();

	// add step of subcomponents
	mPLL->signalStep(time, timeStepCount);
	mPowerControllerVSI->signalStep(time, timeStepCount);
	
	// Transformation interface backward
	mVsref(0,0) = rotatingFrame2to1(Complex(mPowerControllerVSI->attribute<Matrix>("output_curr")->get()(0, 0), mPowerControllerVSI->attribute<Matrix>("output_curr")->get()(1, 0)), mThetaN, mPLL->attribute<Matrix>("output_prev")->get()(0, 0));

	// Update nominal system angle
	mThetaN = mThetaN + mTimeStep * mOmegaN;
}

void DP::Ph1::AvVoltageSourceInverterDQ::mnaAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) {
	// add pre-step dependencies of subcomponents
	mSubCtrledVoltageSource->mnaAddPreStepDependencies(prevStepDependencies, attributeDependencies, modifiedAttributes);
	mSubInductorF->mnaAddPreStepDependencies(prevStepDependencies, attributeDependencies, modifiedAttributes);
	mSubCapacitorF->mnaAddPreStepDependencies(prevStepDependencies, attributeDependencies, modifiedAttributes);
	// add pre-step dependencies of component itself	
	prevStepDependencies.push_back(attribute("Vsref"));
	prevStepDependencies.push_back(attribute("i_intf"));
	prevStepDependencies.push_back(attribute("v_intf"));
	attributeDependencies.push_back(mPowerControllerVSI->attribute<Matrix>("output_prev"));
	attributeDependencies.push_back(mPLL->attribute<Matrix>("output_prev"));
	modifiedAttributes.push_back(attribute("right_vector"));
}

void DP::Ph1::AvVoltageSourceInverterDQ::mnaPreStep(Real time, Int timeStepCount) {	
	// pre-steo of subcomponents - controlled source
	mSubCtrledVoltageSource->setParameters(mVsref(0,0));
	// pre-step of subcomponents - others
	if (mWithConnectionTransformer)
		mConnectionTransformer->mnaPreStep(time, timeStepCount);	
	mSubCtrledVoltageSource->mnaPreStep(time, timeStepCount);
	mSubInductorF->mnaPreStep(time, timeStepCount);
	mSubCapacitorF->mnaPreStep(time, timeStepCount);
	// pre-step of component itself
	mnaApplyRightSideVectorStamp(mRightVector);
}

void DP::Ph1::AvVoltageSourceInverterDQ::mnaAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector) {
	// add post-step dependencies of subcomponents
	if (mWithConnectionTransformer)
		mConnectionTransformer->mnaAddPostStepDependencies(prevStepDependencies, attributeDependencies, modifiedAttributes, leftVector);
	mSubCtrledVoltageSource->mnaAddPostStepDependencies(prevStepDependencies, attributeDependencies, modifiedAttributes, leftVector);
	mSubResistorF->mnaAddPostStepDependencies(prevStepDependencies, attributeDependencies, modifiedAttributes, leftVector);
	mSubInductorF->mnaAddPostStepDependencies(prevStepDependencies, attributeDependencies, modifiedAttributes, leftVector);
	mSubCapacitorF->mnaAddPostStepDependencies(prevStepDependencies, attributeDependencies, modifiedAttributes, leftVector);
	mSubResistorC->mnaAddPostStepDependencies(prevStepDependencies, attributeDependencies, modifiedAttributes, leftVector);
	// add post-step dependencies of component itself
	attributeDependencies.push_back(leftVector);
	modifiedAttributes.push_back(attribute("v_intf"));
	modifiedAttributes.push_back(attribute("i_intf"));
}

void DP::Ph1::AvVoltageSourceInverterDQ::mnaPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
	// post-step of subcomponents
	if (mWithConnectionTransformer)
		mConnectionTransformer->mnaPostStep(time, timeStepCount, leftVector);
	mSubCtrledVoltageSource->mnaPostStep(time, timeStepCount, leftVector);
	mSubResistorF->mnaPostStep(time, timeStepCount, leftVector);
	mSubInductorF->mnaPostStep(time, timeStepCount, leftVector);
	mSubCapacitorF->mnaPostStep(time, timeStepCount, leftVector);
	mSubResistorC->mnaPostStep(time, timeStepCount, leftVector);
	// post-step of component itself
	mnaUpdateCurrent(*leftVector);
	mnaUpdateVoltage(*leftVector);
}

void DP::Ph1::AvVoltageSourceInverterDQ::mnaUpdateCurrent(const Matrix& leftvector) {
	if (mWithConnectionTransformer)
		mIntfCurrent = mConnectionTransformer->attribute<MatrixComp>("i_intf")->get();
	else
		mIntfCurrent = mSubResistorC->attribute<MatrixComp>("i_intf")->get();
}

void DP::Ph1::AvVoltageSourceInverterDQ::mnaUpdateVoltage(const Matrix& leftVector) {
	mVirtualNodes[4]->mnaUpdateVoltage(leftVector);
	mIntfVoltage(0, 0) = 0;
	if (terminalNotGrounded(1))
		mIntfVoltage(0,0) = Math::complexFromVectorElement(leftVector, matrixNodeIndex(1));
	if (terminalNotGrounded(0))
		mIntfVoltage(0,0) = mIntfVoltage(0,0) - Math::complexFromVectorElement(leftVector, matrixNodeIndex(0));
}
