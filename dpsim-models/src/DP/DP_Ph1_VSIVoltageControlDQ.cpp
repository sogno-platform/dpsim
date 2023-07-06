/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/DP/DP_Ph1_VSIVoltageControlDQ.h>

using namespace CPS;


DP::Ph1::VSIVoltageControlDQ::VSIVoltageControlDQ(String uid, String name, Logger::Level logLevel, Bool withTrafo) :
	CompositePowerComp<Complex>(uid, name, true, true, logLevel),
	mOmegaN(Attribute<Real>::create("Omega_nom", mAttributes)),
	mVdRef(Attribute<Real>::create("VdRef", mAttributes)),
	mVqRef(Attribute<Real>::create("VqRef", mAttributes)),
	mVcd(Attribute<Real>::create("Vc_d", mAttributes, 0)),
	mVcq(Attribute<Real>::create("Vc_q", mAttributes, 0)),
	mIrcd(Attribute<Real>::create("Irc_d", mAttributes, 0)),
	mIrcq(Attribute<Real>::create("Irc_q", mAttributes, 0)),
	mElecActivePower(Attribute<Real>::create("P_elec", mAttributes, 0)),
	mElecPassivePower(Attribute<Real>::create("Q_elec", mAttributes, 0)),
	mVsref(Attribute<MatrixComp>::create("Vsref", mAttributes, MatrixComp::Zero(1,1))),
	mVs(Attribute<MatrixComp>::createDynamic("Vs", mAttributes)),
	mPllOutput(Attribute<Matrix>::createDynamic("pll_output", mAttributes)),
	mVoltagectrlInputs(Attribute<Matrix>::createDynamic("voltagectrl_inputs", mAttributes)),
	mVoltagectrlOutputs(Attribute<Matrix>::createDynamic("voltagectrl_outputs", mAttributes)),
	mVoltagectrlStates(Attribute<Matrix>::createDynamic("voltagectrl_states", mAttributes))  {

	//if transformer is connected --> setup
	if (withTrafo) {
		setVirtualNodeNumber(4);
		mConnectionTransformer = DP::Ph1::Transformer::make(**mName + "_trans", **mName + "_trans", mLogLevel, false);
		addMNASubComponent(mConnectionTransformer, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);
	} else {
		setVirtualNodeNumber(3);
	}
	mWithConnectionTransformer = withTrafo;
	setTerminalNumber(1);

	mSLog->info("Create {} {}", type(), name);
	**mIntfVoltage = MatrixComp::Zero(1, 1);
	**mIntfCurrent = MatrixComp::Zero(1, 1);
	// Create electrical sub components
	mSubResistorF = DP::Ph1::Resistor::make(**mName + "_resF", mLogLevel);
	mSubResistorC = DP::Ph1::Resistor::make(**mName + "_resC", mLogLevel);
	mSubCapacitorF = DP::Ph1::Capacitor::make(**mName + "_capF", mLogLevel);
	mSubInductorF = DP::Ph1::Inductor::make(**mName + "_indF", mLogLevel);
	mSubCtrledVoltageSource = DP::Ph1::VoltageSource::make(**mName + "_src", mLogLevel);
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
	mPLL = Signal::PLL::make(**mName + "_PLL", mLogLevel);
	mVoltageControllerVSI = Signal::VoltageControllerVSI::make(**mName + "_VoltageControllerVSI", mLogLevel);

	// Sub voltage source
	mVs->setReference(mSubCtrledVoltageSource->mIntfVoltage);

	// PLL
	mPLL->mInputRef->setReference(mVcq);
	mPllOutput->setReference(mPLL->mOutputCurr);

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
void DP::Ph1::VSIVoltageControlDQ::setParameters(Real sysOmega, Real VdRef, Real VqRef) {
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
void DP::Ph1::VSIVoltageControlDQ::setTransformerParameters(Real nomVoltageEnd1, Real nomVoltageEnd2, Real ratedPower, 
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
		mConnectionTransformer->setParameters(mTransformerNominalVoltageEnd1, mTransformerNominalVoltageEnd2, mTransformerRatedPower, mTransformerRatioAbs, mTransformerRatioPhase, mTransformerResistance, mTransformerInductance);
}

void DP::Ph1::VSIVoltageControlDQ::setControllerParameters(Real Kp_voltageCtrl, Real Ki_voltageCtrl, Real Kp_currCtrl, Real Ki_currCtrl, Real Kp_pll, Real Ki_pll, Real Omega_cutoff) {

	mSLog->info("Control Parameters:");
	mSLog->info("PLL: K_p = {}, K_i = {}, Omega_Nom = {}", Kp_pll, Ki_pll, Omega_cutoff);
	mSLog->info("Voltage Loop: K_p = {}, K_i = {}", Kp_voltageCtrl, Ki_voltageCtrl);
	mSLog->info("Current Loop: K_p = {}, K_i = {}", Kp_currCtrl, Ki_currCtrl);
	mSLog->info("Cut-Off Frequency = {}", Omega_cutoff);

	// TODO: add and use Omega_nominal instead of Omega_cutoff
	mPLL->setParameters(Kp_pll, Ki_pll, Omega_cutoff);
	mPLL->composeStateSpaceMatrices();
	mVoltageControllerVSI->setControllerParameters(Kp_voltageCtrl, Ki_voltageCtrl, Kp_currCtrl, Ki_currCtrl, Kp_pll, Ki_pll, Omega_cutoff);
}

void DP::Ph1::VSIVoltageControlDQ::setFilterParameters(Real Lf, Real Cf, Real Rf, Real Rc) {
	Base::AvVoltageSourceInverterDQ::setFilterParameters(Lf, Cf, Rf, Rc);

	mSLog->info("Filter Parameters:");
	mSLog->info("Inductance Lf={} [H] Capacitance Cf={} [F]", mLf, mCf);
	mSLog->info("Resistance Rf={} [H] Resistance Rc={} [F]", mRf, mRc);

	mSubResistorC->setParameters(mRc);
	mSubResistorF->setParameters(mRf);
	mSubInductorF->setParameters(mLf);
	mSubCapacitorF->setParameters(mCf);
}

//Overload function of Voltage Controller --> set start values of states
void DP::Ph1::VSIVoltageControlDQ::setInitialStateValues(Real phi_dInit, Real phi_qInit, Real gamma_dInit, Real gamma_qInit) {

	mSLog->info("Initial State Value Parameters:");
	mSLog->info("Phi_dInit = {}, Phi_qInit = {}", phi_dInit, phi_qInit);
	mSLog->info("Gamma_dInit = {}, Gamma_qInit = {}", gamma_dInit, gamma_qInit);

	mVoltageControllerVSI->setInitialStateValues(phi_dInit, phi_qInit, gamma_dInit, gamma_qInit);
}

void DP::Ph1::VSIVoltageControlDQ::initializeFromNodesAndTerminals(Real frequency) {
	// MatrixComp intfVoltageComplex = Matrix::Zero(1, 1);
	// MatrixComp intfCurrentComplex = Matrix::Zero(1, 1);
	// terminal powers in consumer system -> convert to generator system
	Real activePower = terminal(0)->singlePower().real();;
	Real reactivePower = terminal(0)->singlePower().imag();

	// set initial interface quantities
	(**mIntfVoltage)(0, 0) = initialSingleVoltage(0);
	(**mIntfCurrent)(0, 0) = -std::conj(Complex(activePower, reactivePower) / (**mIntfVoltage)(0,0));

	Complex filterInterfaceInitialVoltage;
	Complex filterInterfaceInitialCurrent;

	if (mWithConnectionTransformer) {
		// calculate quantities of low voltage side of transformer (being the interface quantities of the filter)
		filterInterfaceInitialVoltage = ((**mIntfVoltage)(0, 0) - Complex(mTransformerResistance, mTransformerInductance * **mOmegaN) * (**mIntfCurrent)(0, 0)) / Complex(mTransformerRatioAbs, mTransformerRatioPhase);
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
	
	if(mWithConnectionTransformer)
	{
		// Initialize control subcomponents
		Complex vcdq, ircdq;
		Real theta = std::arg(mVirtualNodes[3]->initialSingleVoltage());
		vcdq = Math::rotatingFrame2to1(mVirtualNodes[3]->initialSingleVoltage(), std::arg(mVirtualNodes[3]->initialSingleVoltage()), 0);
		ircdq = Math::rotatingFrame2to1(-1. * (**mSubResistorC->mIntfCurrent)(0, 0), std::arg(mVirtualNodes[3]->initialSingleVoltage()), 0);

		**mVcd = vcdq.real();
		**mVcq = vcdq.imag();
		**mIrcd = ircdq.real();
		**mIrcq = ircdq.imag();

		// angle input
		Matrix matrixStateInit = Matrix::Zero(2,1);
		Matrix matrixOutputInit = Matrix::Zero(2,1);
		matrixStateInit(0,0) = std::arg(mVirtualNodes[3]->initialSingleVoltage());
		matrixOutputInit(0,0) = std::arg(mVirtualNodes[3]->initialSingleVoltage());
		mPLL->setInitialValues(**mVcq, matrixStateInit, matrixOutputInit);
	}
	else{
				// Initialize control subcomponents

		Complex vcdq, ircdq;
		Real theta = std::arg(mVirtualNodes[2]->initialSingleVoltage());
		vcdq = Math::rotatingFrame2to1(mVirtualNodes[2]->initialSingleVoltage(), std::arg(mVirtualNodes[2]->initialSingleVoltage()), 0);
		ircdq = Math::rotatingFrame2to1(-1. * (**mSubResistorC->mIntfCurrent)(0, 0), std::arg(mVirtualNodes[2]->initialSingleVoltage()), 0);

		**mVcd = vcdq.real();
		**mVcq = vcdq.imag();
		**mIrcd = ircdq.real();
		**mIrcq = ircdq.imag();

		// angle input
		Matrix matrixStateInit = Matrix::Zero(2,1);
		Matrix matrixOutputInit = Matrix::Zero(2,1);
		matrixStateInit(0,0) = std::arg(mVirtualNodes[2]->initialSingleVoltage());
		matrixOutputInit(0,0) = std::arg(mVirtualNodes[2]->initialSingleVoltage());
		mPLL->setInitialValues(**mVcq, matrixStateInit, matrixOutputInit                           );
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
		Logger::phasorToString((**mIntfVoltage)(0, 0)),
		Logger::phasorToString((**mIntfCurrent)(0, 0)),
		Logger::phasorToString(initialSingleVoltage(0)),
		mTerminals[0]->node()->name(), mTerminals[0]->node()->matrixNodeIndex(),
		Logger::phasorToString(mVirtualNodes[0]->initialSingleVoltage()),
		Logger::phasorToString(mVirtualNodes[1]->initialSingleVoltage()),
		Logger::phasorToString(mVirtualNodes[2]->initialSingleVoltage()));
		if (mWithConnectionTransformer)
			mSLog->info("\nVirtual node 3 initial voltage: {:s}", Logger::phasorToString(mVirtualNodes[3]->initialSingleVoltage()));
		mSLog->info("\n--- Initialization from powerflow finished ---");
}

void DP::Ph1::VSIVoltageControlDQ::mnaParentInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
	mTimeStep = timeStep;

	// initialize state space controller
	mVoltageControllerVSI->initializeStateSpaceModel(omega, timeStep, leftVector);
	mPLL->setSimulationParameters(timeStep);

	// TODO: these are actually no MNA tasks
	mMnaTasks.push_back(std::make_shared<ControlPreStep>(*this));
	mMnaTasks.push_back(std::make_shared<ControlStep>(*this));
}

void DP::Ph1::VSIVoltageControlDQ::addControlPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) {
	// add pre-step dependencies of subcomponents
	mPLL->signalAddPreStepDependencies(prevStepDependencies, attributeDependencies, modifiedAttributes);
	mVoltageControllerVSI->signalAddPreStepDependencies(prevStepDependencies, attributeDependencies, modifiedAttributes);
}

void DP::Ph1::VSIVoltageControlDQ::controlPreStep(Real time, Int timeStepCount) {
	// add pre-step of subcomponents
	mPLL->signalPreStep(time, timeStepCount);
	mVoltageControllerVSI->signalPreStep(time, timeStepCount);
}

void DP::Ph1::VSIVoltageControlDQ::addControlStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) {
	// add step dependencies of subcomponents
	mPLL->signalAddStepDependencies(prevStepDependencies, attributeDependencies, modifiedAttributes);
	mVoltageControllerVSI->signalAddStepDependencies(prevStepDependencies, attributeDependencies, modifiedAttributes);
	// add step dependencies of component itself
	attributeDependencies.push_back(mIntfCurrent);
	attributeDependencies.push_back(mIntfVoltage);
	modifiedAttributes.push_back(mVsref);
}

void DP::Ph1::VSIVoltageControlDQ::controlStep(Real time, Int timeStepCount) {
	// Transformation interface forward
	Complex vcdq, ircdq;
	Real theta = mPLL->mOutputPrev->get()(0, 0);
	if(mWithConnectionTransformer)
	{
	vcdq = Math::rotatingFrame2to1(mVirtualNodes[3]->singleVoltage(), mPLL->attributeTyped<Matrix>("output_prev")->get()(0, 0), mThetaN);
	ircdq = Math::rotatingFrame2to1(-1. * (**mSubResistorC->mIntfCurrent)(0, 0), mPLL->attributeTyped<Matrix>("output_prev")->get()(0, 0), mThetaN);
	}
	else{
		vcdq = Math::rotatingFrame2to1(mVirtualNodes[2]->singleVoltage(), mPLL->attributeTyped<Matrix>("output_prev")->get()(0, 0), mThetaN);
		ircdq = Math::rotatingFrame2to1(-1. * (**mSubResistorC->mIntfCurrent)(0, 0), mPLL->attributeTyped<Matrix>("output_prev")->get()(0, 0), mThetaN);
	
	}

	**mVcd = vcdq.real();
	**mVcq = vcdq.imag();
	**mIrcd = ircdq.real();
	**mIrcq = ircdq.imag();

	// add step of subcomponents
	mPLL->signalStep(time, timeStepCount);
	mVoltageControllerVSI->signalStep(time, timeStepCount);

	// Transformation interface backward
	(**mVsref)(0,0) = Math::rotatingFrame2to1(Complex(mPLL->mOutputCurr->get()(0, 0),mPLL->mOutputCurr->get()(1, 0)), mThetaN, mPLL->mOutputPrev->get()(0, 0));

	// Update nominal system angle
	mThetaN = mThetaN + mTimeStep * **mOmegaN;
}

void DP::Ph1::VSIVoltageControlDQ::mnaParentAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) {
	prevStepDependencies.push_back(mVsref);
	prevStepDependencies.push_back(mIntfCurrent);
	prevStepDependencies.push_back(mIntfVoltage);
	attributeDependencies.push_back(mVoltageControllerVSI->mOutputPrev);
	attributeDependencies.push_back(mPLL->mOutputPrev);
	modifiedAttributes.push_back(mRightVector);
}

void DP::Ph1::VSIVoltageControlDQ::mnaParentPreStep(Real time, Int timeStepCount) {
	// pre-step of subcomponents - controlled source
	if (mWithControl)
		**mSubCtrledVoltageSource->mVoltageRef = (**mVsref)(0,0);

	std::dynamic_pointer_cast<MNAInterface>(mSubCtrledVoltageSource)->mnaPreStep(time, timeStepCount);
	// pre-step of component itself
	mnaApplyRightSideVectorStamp(**mRightVector);
}

void DP::Ph1::VSIVoltageControlDQ::mnaParentAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector) {
	attributeDependencies.push_back(leftVector);
	modifiedAttributes.push_back(mIntfVoltage);
	modifiedAttributes.push_back(mIntfCurrent);
}

void DP::Ph1::VSIVoltageControlDQ::mnaParentPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
	mnaUpdateCurrent(**leftVector);
	mnaUpdateVoltage(**leftVector);
}

//Current update
void DP::Ph1::VSIVoltageControlDQ::mnaUpdateCurrent(const Matrix& leftvector) {
	if (mWithConnectionTransformer)
		**mIntfCurrent = mConnectionTransformer->mIntfCurrent->get();
	else
		**mIntfCurrent = mSubResistorC->mIntfCurrent->get();
}

//Voltage update
void DP::Ph1::VSIVoltageControlDQ::mnaUpdateVoltage(const Matrix& leftVector) {
	for (auto virtualNode : mVirtualNodes)
		virtualNode->mnaUpdateVoltage(leftVector);
	(**mIntfVoltage)(0,0) = Math::complexFromVectorElement(leftVector, matrixNodeIndex(0));
}
