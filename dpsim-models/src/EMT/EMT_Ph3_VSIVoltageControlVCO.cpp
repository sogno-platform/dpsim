/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/EMT/EMT_Ph3_VSIVoltageControlVCO.h>

using namespace CPS;


EMT::Ph3::VSIVoltageControlVCO::VSIVoltageControlVCO(String uid, String name, Logger::Level logLevel, Bool withTrafo) :
	CompositePowerComp<Real>(uid, name, true, true, logLevel),
	mOmegaN(Attribute<Real>::create("Omega_nom", mAttributes)),
	mVdRef(Attribute<Real>::create("VdRef", mAttributes)),
	mVqRef(Attribute<Real>::create("VqRef", mAttributes)),
	mVcd(Attribute<Real>::create("Vc_d", mAttributes, 0)),
	mVcq(Attribute<Real>::create("Vc_q", mAttributes, 0)),
	mIrcd(Attribute<Real>::create("Irc_d", mAttributes, 0)),
	mIrcq(Attribute<Real>::create("Irc_q", mAttributes, 0)),
	mElecActivePower(Attribute<Real>::create("P_elec", mAttributes, 0)),
	mElecPassivePower(Attribute<Real>::create("Q_elec", mAttributes, 0)),
	mVsref(Attribute<Matrix>::create("Vsref", mAttributes, Matrix::Zero(3,1))),
	mVs(Attribute<Matrix>::createDynamic("Vs", mAttributes)),
	mVCOOutput(Attribute<Real>::createDynamic("VCO_output", mAttributes)),
	mVoltagectrlInputs(Attribute<Matrix>::createDynamic("voltagectrl_inputs", mAttributes)),
	mVoltagectrlOutputs(Attribute<Matrix>::createDynamic("voltagectrl_outputs", mAttributes)),
	mVoltagectrlStates(Attribute<Matrix>::createDynamic("voltagectrl_states", mAttributes))  {
	
	mPhaseType = PhaseType::ABC;

	//if transformer is connected --> setup
	if (withTrafo) {
		setVirtualNodeNumber(4);
		mConnectionTransformer = EMT::Ph3::Transformer::make(**mName + "_trans", **mName + "_trans", mLogLevel, false);
		addMNASubComponent(mConnectionTransformer, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);
	} else {
		setVirtualNodeNumber(3);
	}
	mWithConnectionTransformer = withTrafo;
	setTerminalNumber(1);

	mSLog->info("Create {} {}", type(), name);
	**mIntfVoltage = Matrix::Zero(3, 1);
	**mIntfCurrent = Matrix::Zero(3, 1);

	// Create electrical sub components
	mSubResistorF = EMT::Ph3::Resistor::make(**mName + "_resF", mLogLevel);
	mSubResistorC = EMT::Ph3::Resistor::make(**mName + "_resC", mLogLevel);
	mSubCapacitorF = EMT::Ph3::Capacitor::make(**mName + "_capF", mLogLevel);
	mSubInductorF = EMT::Ph3::Inductor::make(**mName + "_indF", mLogLevel);
	mSubCtrledVoltageSource = EMT::Ph3::VoltageSource::make(**mName + "_src", mLogLevel);
	addMNASubComponent(mSubResistorF, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, false);
	addMNASubComponent(mSubResistorC, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, false);
	addMNASubComponent(mSubCapacitorF, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);
	addMNASubComponent(mSubInductorF, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);

	// Pre-step of the subcontrolled voltage source is handled explicitly in mnaParentPreStep
	addMNASubComponent(mSubCtrledVoltageSource, MNA_SUBCOMP_TASK_ORDER::NO_TASK, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);

	//Log subcomponents
	mSLog->info("Electrical subcomponents: ");
	for (auto subcomp: mSubComponents)
		mSLog->info("- {}", subcomp->name());

	// Create control sub components
	mVCO = Signal::VCO::make(**mName + "_VCO", mLogLevel);
	mVoltageControllerVSI = Signal::VoltageControllerVSI::make(**mName + "_VoltageControllerVSI", mLogLevel);

	// Sub voltage source
	mVs->setReference(mSubCtrledVoltageSource->mIntfVoltage);

	// VCO
	mVCO->mInputRef->setReference(mOmegaN);
	mVCOOutput->setReference(mVCO->mOutputCurr);

	// Voltage controller
	// input references
	mVoltageControllerVSI->mVc_d->setReference(mVcd);
	mVoltageControllerVSI->mVc_q->setReference(mVcq);
	mVoltageControllerVSI->mIrc_d->setReference(mIrcd);
	mVoltageControllerVSI->mIrc_q->setReference(mIrcq);

	// input, state and output vector for logging
	mVoltagectrlInputs->setReference(mVoltageControllerVSI->mInputCurr);
	mVoltagectrlStates->setReference(mVoltageControllerVSI->mStateCurr);
	mVoltagectrlOutputs->setReference(mVoltageControllerVSI->mOutputCurr);
}

//setter goal voltage and frequency
void EMT::Ph3::VSIVoltageControlVCO::setParameters(Real sysOmega, Real VdRef, Real VqRef) {
	mParametersSet = true;

	mSLog->info("General Parameters:");
	mSLog->info("Nominal Omega={} [1/s]", sysOmega);
	mSLog->info("VdRef={} [V] VqRef={} [V]", VdRef, VqRef);

	mVoltageControllerVSI->setParameters(VdRef, VqRef);

	**mOmegaN = sysOmega;
	**mVdRef = VdRef;
	**mVqRef = VqRef;
}

//setter for transformer if used
void EMT::Ph3::VSIVoltageControlVCO::setTransformerParameters(Real nomVoltageEnd1, Real nomVoltageEnd2, Real ratedPower, 
	Real ratioAbs,	Real ratioPhase, Real resistance, Real inductance, Real omega) {
	
	Base::AvVoltageSourceInverterDQ::setTransformerParameters(nomVoltageEnd1, nomVoltageEnd2, ratedPower,
	ratioAbs, ratioPhase, resistance, inductance);

	mSLog->info("Connection Transformer Parameters:");
	mSLog->info("Nominal Voltage End 1={} [V] Nominal Voltage End 2={} [V]", mTransformerNominalVoltageEnd1, mTransformerNominalVoltageEnd2);
	mSLog->info("Rated Apparent Power = {} [VA]", mTransformerRatedPower);
	mSLog->info("Resistance={} [Ohm] Inductance={} [H]", mTransformerResistance, mTransformerInductance);
    mSLog->info("Tap Ratio={} [ ] Phase Shift={} [deg]", mTransformerRatioAbs, mTransformerRatioPhase);

	if (mWithConnectionTransformer)
		// TODO: resistive losses neglected so far (mWithResistiveLosses=false)
		mConnectionTransformer->setParameters(mTransformerNominalVoltageEnd1, mTransformerNominalVoltageEnd2, mTransformerRatedPower, mTransformerRatioAbs, mTransformerRatioPhase, CPS::Math::singlePhaseParameterToThreePhase(mTransformerResistance), CPS::Math::singlePhaseParameterToThreePhase(mTransformerInductance));
}

void EMT::Ph3::VSIVoltageControlVCO::setControllerParameters(Real Kp_voltageCtrl, Real Ki_voltageCtrl, Real Kp_currCtrl, Real Ki_currCtrl, Real Omega_nominal) {

	mSLog->info("Control Parameters:");
	mSLog->info("VCO: Omega_Nom = {}", Omega_nominal);
	mSLog->info("Voltage Loop: K_p = {}, K_i = {}", Kp_voltageCtrl, Ki_voltageCtrl);
	mSLog->info("Current Loop: K_p = {}, K_i = {}", Kp_currCtrl, Ki_currCtrl);

	// TODO: add and use Omega_nominal instead of Omega_cutoff
	mVCO->setParameters(Omega_nominal);
	mVoltageControllerVSI->setControllerParameters(Kp_voltageCtrl, Ki_voltageCtrl, Kp_currCtrl, Ki_currCtrl, Omega_nominal);
}

void EMT::Ph3::VSIVoltageControlVCO::setFilterParameters(Real Lf, Real Cf, Real Rf, Real Rc) {
	Base::AvVoltageSourceInverterDQ::setFilterParameters(Lf, Cf, Rf, Rc);

	mSLog->info("Filter Parameters:");
	mSLog->info("Inductance Lf={} [H] Capacitance Cf={} [F]", mLf, mCf);
	mSLog->info("Resistance Rf={} [H] Resistance Rc={} [F]", mRf, mRc);

	mSubResistorC->setParameters(CPS::Math::singlePhaseParameterToThreePhase(mRc));
	mSubResistorF->setParameters(CPS::Math::singlePhaseParameterToThreePhase(mRf));
	mSubInductorF->setParameters(CPS::Math::singlePhaseParameterToThreePhase(mLf));
	mSubCapacitorF->setParameters(CPS::Math::singlePhaseParameterToThreePhase(mCf));
}

//Overload function of Voltage Controller --> set start values of states
void EMT::Ph3::VSIVoltageControlVCO::setInitialStateValues(Real phi_dInit, Real phi_qInit, Real gamma_dInit, Real gamma_qInit) {

	mSLog->info("Initial State Value Parameters:");
	mSLog->info("Phi_dInit = {}, Phi_qInit = {}", phi_dInit, phi_qInit);
	mSLog->info("Gamma_dInit = {}, Gamma_qInit = {}", gamma_dInit, gamma_qInit);

	mVoltageControllerVSI->setInitialStateValues(phi_dInit, phi_qInit, gamma_dInit, gamma_qInit);
}

void EMT::Ph3::VSIVoltageControlVCO::initializeFromNodesAndTerminals(Real frequency) {

	// use complex interface quantities for initialization calculations
	MatrixComp intfVoltageComplex = Matrix::Zero(3, 1);
	MatrixComp intfCurrentComplex = Matrix::Zero(3, 1);
	// terminal powers in consumer system -> convert to generator system
	Real activePower = terminal(0)->singlePower().real();;
	Real reactivePower = terminal(0)->singlePower().imag();

	// derive complex threephase initialization from single phase initial values (only valid for balanced systems)
	intfVoltageComplex(0, 0) = RMS3PH_TO_PEAK1PH * initialSingleVoltage(0);
	intfVoltageComplex(1, 0) = intfVoltageComplex(0, 0) * SHIFT_TO_PHASE_B;
	intfVoltageComplex(2, 0) = intfVoltageComplex(0, 0) * SHIFT_TO_PHASE_C;
	intfCurrentComplex(0, 0) = -std::conj(2./3.*Complex(activePower, reactivePower) / intfVoltageComplex(0, 0));
	intfCurrentComplex(1, 0) = intfCurrentComplex(0, 0) * SHIFT_TO_PHASE_B;
	intfCurrentComplex(2, 0) = intfCurrentComplex(0, 0) * SHIFT_TO_PHASE_C;

	MatrixComp filterInterfaceInitialVoltage = MatrixComp::Zero(3, 1);
	MatrixComp filterInterfaceInitialCurrent = MatrixComp::Zero(3, 1);
	if (mWithConnectionTransformer) {
		// calculate quantities of low voltage side of transformer (being the interface quantities of the filter, calculations only valid for symmetrical systems)
		filterInterfaceInitialVoltage = (intfVoltageComplex - Complex(mTransformerResistance, mTransformerInductance * **mOmegaN)*intfCurrentComplex) / Complex(mTransformerRatioAbs, mTransformerRatioPhase);
		filterInterfaceInitialCurrent = intfCurrentComplex * Complex(mTransformerRatioAbs, mTransformerRatioPhase);

		// connect and init transformer
		mVirtualNodes[3]->setInitialVoltage(PEAK1PH_TO_RMS3PH * filterInterfaceInitialVoltage);
		mConnectionTransformer->connect({ mTerminals[0]->node(), mVirtualNodes[3] });
	} else {
		// if no transformer used, filter interface equal to inverter interface
		filterInterfaceInitialVoltage = intfVoltageComplex;
		filterInterfaceInitialCurrent = intfCurrentComplex;
	}

	// derive initialization quantities of filter (calculations only valid for symmetrical systems)
	MatrixComp vcInit = filterInterfaceInitialVoltage - filterInterfaceInitialCurrent * Complex(mRc, 0);
	MatrixComp icfInit = vcInit * Complex(0., 2. * PI * frequency * mCf);
	MatrixComp vfInit = vcInit - (filterInterfaceInitialCurrent - icfInit) * Complex(0., 2. * PI * frequency * mLf);
	MatrixComp vsInit = vfInit - (filterInterfaceInitialCurrent - icfInit) * Complex(mRf, 0);
	mVirtualNodes[0]->setInitialVoltage(PEAK1PH_TO_RMS3PH * vsInit);
	mVirtualNodes[1]->setInitialVoltage(PEAK1PH_TO_RMS3PH * vfInit);
	mVirtualNodes[2]->setInitialVoltage(PEAK1PH_TO_RMS3PH * vcInit);

	// save real interface quantities calculated from complex ones
	**mIntfVoltage = intfVoltageComplex.real();
	**mIntfCurrent = intfCurrentComplex.real();
	**mElecActivePower = ( 3./2. * intfVoltageComplex(0,0) *  std::conj( - intfCurrentComplex(0,0)) ).real();
	**mElecPassivePower = ( 3./2. * intfVoltageComplex(0,0) *  std::conj( - intfCurrentComplex(0,0)) ).imag();

	// Initialize controlled source
	mSubCtrledVoltageSource->setParameters(mVirtualNodes[0]->initialVoltage(), 0.0);

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

	
	if(mWithConnectionTransformer)
	{
		// Initialize control subcomponents
		// current and voltage inputs to VCO and voltage controller
		Matrix vcdq, ircdq;
		Real theta = std::arg(mVirtualNodes[3]->initialSingleVoltage());
		vcdq = parkTransformPowerInvariant(theta, filterInterfaceInitialVoltage.real());
		ircdq = parkTransformPowerInvariant(theta, -1 * filterInterfaceInitialCurrent.real());

		**mVcd = vcdq(0, 0);
		**mVcq = vcdq(1, 0);
		**mIrcd = ircdq(0, 0);
		**mIrcq = ircdq(1, 0);

		// VCO start settings
		mVCO->setInitialValues(**mOmegaN, theta, theta);
	} 
	else
	{
		// Initialize control subcomponents
		// current and voltage inputs to VCO and voltage controller
		Matrix vcdq, ircdq;
		Real theta = std::arg(mVirtualNodes[2]->initialSingleVoltage());
		vcdq = parkTransformPowerInvariant(theta, filterInterfaceInitialVoltage.real());
		ircdq = parkTransformPowerInvariant(theta, -1 * filterInterfaceInitialCurrent.real());

		**mVcd = vcdq(0, 0);
		**mVcq = vcdq(1, 0);
		**mIrcd = ircdq(0, 0);
		**mIrcq = ircdq(1, 0);

	    // VCO start settings
		mVCO->setInitialValues(**mOmegaN, theta, theta);
	}
	mSLog->info(
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
			mSLog->info("\nVirtual node 3 initial voltage: {:s}", Logger::phasorToString(mVirtualNodes[3]->initialSingleVoltage()));
		mSLog->info("\n--- Initialization from powerflow finished ---");
}

void EMT::Ph3::VSIVoltageControlVCO::mnaParentInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
	mTimeStep = timeStep;

	// initialize state space controller
	mVoltageControllerVSI->initializeStateSpaceModel(omega, timeStep, leftVector);
	mVCO->setSimulationParameters(timeStep);

	// TODO: these are actually no MNA tasks
	mMnaTasks.push_back(std::make_shared<ControlPreStep>(*this));
	mMnaTasks.push_back(std::make_shared<ControlStep>(*this));
}

void EMT::Ph3::VSIVoltageControlVCO::addControlPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) {
	// add pre-step dependencies of subcomponents
	mVCO->signalAddPreStepDependencies(prevStepDependencies, attributeDependencies, modifiedAttributes);
	mVoltageControllerVSI->signalAddPreStepDependencies(prevStepDependencies, attributeDependencies, modifiedAttributes);
}

void EMT::Ph3::VSIVoltageControlVCO::controlPreStep(Real time, Int timeStepCount) {
	// add pre-step of subcomponents
	mVCO->signalPreStep(time, timeStepCount);
	mVoltageControllerVSI->signalPreStep(time, timeStepCount);
}

void EMT::Ph3::VSIVoltageControlVCO::addControlStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) {
	// add step dependencies of subcomponents
	mVCO->signalAddStepDependencies(prevStepDependencies, attributeDependencies, modifiedAttributes);
	mVoltageControllerVSI->signalAddStepDependencies(prevStepDependencies, attributeDependencies, modifiedAttributes);
	// add step dependencies of component itself
	attributeDependencies.push_back(mIntfCurrent);
	attributeDependencies.push_back(mIntfVoltage);
	modifiedAttributes.push_back(mVsref);
}

Matrix EMT::Ph3::VSIVoltageControlVCO::parkTransformPowerInvariant(Real theta, const Matrix &fabc) {
	// Calculates fdq = Tdq * fabc
	// Assumes that d-axis starts aligned with phase a
	Matrix Tdq = getParkTransformMatrixPowerInvariant(theta);
	Matrix dqvector = Tdq * fabc;
	return dqvector;
}

Matrix EMT::Ph3::VSIVoltageControlVCO::getParkTransformMatrixPowerInvariant(Real theta) {
	// Return park matrix for theta
	// Assumes that d-axis starts aligned with phase a
	Matrix Tdq = Matrix::Zero(2, 3);
	Real k = sqrt(2. / 3.);
	Tdq <<
		k * cos(theta), k * cos(theta - 2. * M_PI / 3.), k * cos(theta + 2. * M_PI / 3.),
		-k * sin(theta), -k * sin(theta - 2. * M_PI / 3.), -k * sin(theta + 2. * M_PI / 3.);
	return Tdq;
}

Matrix EMT::Ph3::VSIVoltageControlVCO::inverseParkTransformPowerInvariant(Real theta, const Matrix &fdq) {
	// Calculates fabc = Tabc * fdq
	// with d-axis starts aligned with phase a
	Matrix Tabc = getInverseParkTransformMatrixPowerInvariant(theta);
	Matrix fabc = Tabc * fdq;
	return fabc;
}


Matrix EMT::Ph3::VSIVoltageControlVCO::getInverseParkTransformMatrixPowerInvariant(Real theta) {
	// Return inverse park matrix for theta
	/// with d-axis starts aligned with phase a
	Matrix Tabc = Matrix::Zero(3, 2);
	Real k = sqrt(2. / 3.);
	Tabc <<
		k * cos(theta), - k * sin(theta),
		k * cos(theta - 2. * M_PI / 3.), - k * sin(theta - 2. * M_PI / 3.),
		k * cos(theta + 2. * M_PI / 3.), - k * sin(theta + 2. * M_PI / 3.);
	return Tabc;
}

void EMT::Ph3::VSIVoltageControlVCO::controlStep(Real time, Int timeStepCount) {
	// Transformation interface forward
	Matrix vcdq, ircdq;
	Real theta = mVCO->mOutputPrev->get();
	vcdq = parkTransformPowerInvariant(theta, **mVirtualNodes[2]->mVoltage);
	ircdq = parkTransformPowerInvariant(theta, - **mSubResistorC->mIntfCurrent);
	Matrix intfVoltageDQ = parkTransformPowerInvariant(mThetaN, **mIntfVoltage);
	Matrix intfCurrentDQ = parkTransformPowerInvariant(mThetaN, **mIntfCurrent);
	**mElecActivePower = - 1. * (intfVoltageDQ(0, 0)*intfCurrentDQ(0, 0) + intfVoltageDQ(1, 0)*intfCurrentDQ(1, 0));
	**mElecPassivePower = - 1. * (intfVoltageDQ(1, 0)*intfCurrentDQ(0, 0) - intfVoltageDQ(0, 0)*intfCurrentDQ(1, 0));
	//vector of voltages
	**mVcd = vcdq(0, 0);
	**mVcq = vcdq(1, 0);

	//vector of currents
	**mIrcd = ircdq(0, 0);
	**mIrcq = ircdq(1, 0);

	// add step of subcomponents
	mVCO->signalStep(time, timeStepCount);
	mVoltageControllerVSI->signalStep(time, timeStepCount);

	// Transformation interface backward
	**mVsref = inverseParkTransformPowerInvariant(mVCO->mOutputPrev->get(), mVoltageControllerVSI->mOutputCurr->get());

	// Update nominal system angle
	//mThetaN = mThetaN + mTimeStep * **mOmegaN;
    mThetaN = std::fmod(mThetaN + mTimeStep * **mOmegaN,2*PI);
}

void EMT::Ph3::VSIVoltageControlVCO::mnaParentAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) {
	prevStepDependencies.push_back(mVsref);
	prevStepDependencies.push_back(mIntfCurrent);
	prevStepDependencies.push_back(mIntfVoltage);
	attributeDependencies.push_back(mVoltageControllerVSI->mOutputPrev);
	attributeDependencies.push_back(mVCO->mOutputPrev);
	modifiedAttributes.push_back(mRightVector);
}

void EMT::Ph3::VSIVoltageControlVCO::mnaParentPreStep(Real time, Int timeStepCount) {
	// pre-step of subcomponents - controlled source
	if (mWithControl)
		mSubCtrledVoltageSource->mVoltageRef->set(PEAK1PH_TO_RMS3PH * **mVsref);

	std::dynamic_pointer_cast<MNAInterface>(mSubCtrledVoltageSource)->mnaPreStep(time, timeStepCount);
	// pre-step of component itself
	mnaApplyRightSideVectorStamp(**mRightVector);
}

void EMT::Ph3::VSIVoltageControlVCO::mnaParentAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector) {
	attributeDependencies.push_back(leftVector);
	modifiedAttributes.push_back(mIntfVoltage);
	modifiedAttributes.push_back(mIntfCurrent);
}

void EMT::Ph3::VSIVoltageControlVCO::mnaParentPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
	mnaUpdateCurrent(**leftVector);
	mnaUpdateVoltage(**leftVector);
}

//Current update
void EMT::Ph3::VSIVoltageControlVCO::mnaUpdateCurrent(const Matrix& leftvector) {
	if (mWithConnectionTransformer)
		**mIntfCurrent = mConnectionTransformer->mIntfCurrent->get();
	else
		**mIntfCurrent = mSubResistorC->mIntfCurrent->get();
}

//Voltage update
void EMT::Ph3::VSIVoltageControlVCO::mnaUpdateVoltage(const Matrix& leftVector) {
	for (auto virtualNode : mVirtualNodes)
		virtualNode->mnaUpdateVoltage(leftVector);
	(**mIntfVoltage)(0,0) = Math::realFromVectorElement(leftVector, matrixNodeIndex(0,0));
	(**mIntfVoltage)(1,0) = Math::realFromVectorElement(leftVector, matrixNodeIndex(0,1));
	(**mIntfVoltage)(2,0) = Math::realFromVectorElement(leftVector, matrixNodeIndex(0,2));
}
