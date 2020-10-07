/* Copyright 2017-2020 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <cps/EMT/EMT_Ph3_AvVoltageSourceInverterDQ.h>

using namespace CPS;

EMT::Ph3::AvVoltageSourceInverterDQ::AvVoltageSourceInverterDQ(String uid, String name, Logger::Level logLevel, Bool withTrafo) :
	SimPowerComp<Real>(uid,name,logLevel) {
	mPhaseType = PhaseType::ABC;
	if (withTrafo) {
		setVirtualNodeNumber(5);
		mConnectionTransformer = EMT::Ph3::Transformer::make(mName + "_trans", Logger::Level::debug);
		mSubComponents.push_back(mConnectionTransformer);
	} else {
		setVirtualNodeNumber(4);
	}
	mWithConnectionTransformer = withTrafo;
	setTerminalNumber(1);

	mSLog->info("Create {} {}", this->type(), name);
	mIntfVoltage = Matrix::Zero(3, 1);
	mIntfCurrent = Matrix::Zero(3, 1);

	// additional input variables
	addAttribute<Matrix>("Vcdq", &mVcdq, Flags::read | Flags::write);
	addAttribute<Matrix>("Ircdq", &mIrcdq, Flags::read | Flags::write);

	// additional output variables
	addAttribute<Matrix>("Vsdq", &mVsdq, Flags::read | Flags::write);

	// additional variables for logging
	addAttribute<Real>("omega", &mOmegaInst, Flags::read | Flags::write);
	addAttribute<Real>("freq", &mFreqInst, Flags::read | Flags::write);
	addAttribute<Bool>("ctrl_on", &mCtrlOn, Flags::read | Flags::write);

	// state variables
	addAttribute<Real>("theta", &mThetaPLL, Flags::read | Flags::write);
	addAttribute<Real>("phipll", &mPhiPLL, Flags::read | Flags::write);
	addAttribute<Real>("p", &mP, Flags::read | Flags::write);
	addAttribute<Real>("q", &mQ, Flags::read | Flags::write);
	addAttribute<Real>("phid", &mPhi_d, Flags::read | Flags::write);
	addAttribute<Real>("phiq", &mPhi_q, Flags::read | Flags::write);
	addAttribute<Real>("gammad", &mGamma_d, Flags::read | Flags::write);
	addAttribute<Real>("gammaq", &mGamma_q, Flags::read | Flags::write);

	// input variables
	addAttribute<Real>("Omega_nom", &mOmegaN, Flags::read | Flags::write);
	addAttribute<Real>("P_ref", &mPref, Flags::read | Flags::write);
	addAttribute<Real>("Q_ref", &mQref, Flags::read | Flags::write);
}

void EMT::Ph3::AvVoltageSourceInverterDQ::setParameters(Real sysOmega, Real sysVoltNom, Real Pref, Real Qref) {
	Base::AvVoltageSourceInverterDQWithStateSpace::setParameters(sysOmega, sysVoltNom, Pref, Qref);
	mParametersSet = true;

	mSLog->info("General Parameters:");
	mSLog->info("Nominal Voltage={} [V] Nominal Omega={} [1/s]", mVnom, mOmegaN);
	mSLog->info("Active Power={} [W] Reactive Power={} [VAr]", mPref, mQref);
}

void EMT::Ph3::AvVoltageSourceInverterDQ::setTransformerParameters(Real nomVoltageEnd1, Real nomVoltageEnd2,
	Real ratedPower, Real ratioAbs,	Real ratioPhase, Real resistance, Real inductance, Real omega) {

	Base::AvVoltageSourceInverterDQWithStateSpace::setTransformerParameters(nomVoltageEnd1, nomVoltageEnd2,
		ratedPower, ratioAbs, ratioPhase, resistance, inductance, omega);

	mSLog->info("Connection Transformer Parameters:");
	mSLog->info("Resistance={} [Ohm] Inductance={} [H]", mTransformerResistance, mTransformerInductance);
    mSLog->info("Tap Ratio={} [ ] Phase Shift={} [deg]", mTransformerRatioAbs, mTransformerRatioPhase);
}

void EMT::Ph3::AvVoltageSourceInverterDQ::setControllerParameters(Real Kp_pll, Real Ki_pll,
	Real Kp_powerCtrl, Real Ki_powerCtrl, Real Kp_currCtrl, Real Ki_currCtrl, Real Omega_cutoff) {

	Base::AvVoltageSourceInverterDQWithStateSpace::setControllerParameters(Kp_pll, Ki_pll,
		Kp_powerCtrl, Ki_powerCtrl, Kp_currCtrl, Ki_currCtrl, Omega_cutoff);

	mSLog->info("Control Parameters:");
	mSLog->info("PLL: K_i = {}, K_p = {}", mKpPLL, mKiPLL);
	mSLog->info("Power Loop: K_i = {}, K_p = {}", mKpPowerCtrld, mKiPowerCtrld);
	mSLog->info("Current Loop: K_i = {}, K_p = {}", mKpCurrCtrld, mKiCurrCtrld);
	mSLog->info("Cut-Off Frequency = {}", mOmegaCutoff);
}

void EMT::Ph3::AvVoltageSourceInverterDQ::setFilterParameters(Real Lf, Real Cf, Real Rf, Real Rc) {

	Base::AvVoltageSourceInverterDQWithStateSpace::setFilterParameters(Lf, Cf, Rf, Rc);

	mSLog->info("Filter Parameters:");
	mSLog->info("Inductance Lf={} [H] Capacitance Cf={} [F]", mLf, mCf);
	mSLog->info("Resistance Rf={} [H] Resistance Rc={} [F]", mRf, mRc);
}

void EMT::Ph3::AvVoltageSourceInverterDQ::setInitialStateValues(Real thetaPLLInit, Real phiPLLInit, Real pInit, Real qInit,
	Real phi_dInit, Real phi_qInit, Real gamma_dInit, Real gamma_qInit) {

	Base::AvVoltageSourceInverterDQWithStateSpace::setInitialStateValues(thetaPLLInit, phiPLLInit, pInit, qInit,
		phi_dInit, phi_qInit, gamma_dInit, gamma_qInit);

	mSLog->info("Initial State Value Parameters:");
	mSLog->info("ThetaPLLInit = {}, PhiPLLInit = {}", mThetaPLLInit, mPhiPLLInit);
	mSLog->info("PInit = {}, QInit = {}", mPInit, mQInit);
	mSLog->info("Phi_dInit = {}, Phi_qInit = {}", mPhi_dInit, mPhi_qInit);
	mSLog->info("Gamma_dInit = {}, Gamma_qInit = {}", mGamma_dInit, mGamma_qInit);
}

SimPowerComp<Real>::Ptr EMT::Ph3::AvVoltageSourceInverterDQ::clone(String name) {
	auto copy = EMT::Ph3::AvVoltageSourceInverterDQ::make(name, mLogLevel);
	copy->setParameters(mOmegaN, mVnom, mPref, mQref);
	return copy;
}

void EMT::Ph3::AvVoltageSourceInverterDQ::updateInputStateSpaceModel(const Matrix& leftVector, Real time) {

	mVcabc(0, 0) = Math::realFromVectorElement(leftVector, mSubCapacitorF->matrixNodeIndex(0, 0));
	mVcabc(1, 0) = Math::realFromVectorElement(leftVector, mSubCapacitorF->matrixNodeIndex(0, 1));
	mVcabc(2, 0) = Math::realFromVectorElement(leftVector, mSubCapacitorF->matrixNodeIndex(0, 2));
	mVcdq = parkTransformPowerInvariant(mThetaPLL, mVcabc(0, 0), mVcabc(1, 0), mVcabc(2, 0));

	mIrcabc = -1 * mSubResistorC->attribute<Matrix>("i_intf")->get();
	mIrcdq = parkTransformPowerInvariant(mThetaPLL, mIrcabc(0, 0), mIrcabc(1, 0), mIrcabc(2, 0));

	mIntfVoltage(0, 0) = Math::realFromVectorElement(leftVector, mTerminals[0]->matrixNodeIndices()[0]);
	mIntfVoltage(1, 0) = Math::realFromVectorElement(leftVector, mTerminals[0]->matrixNodeIndices()[1]);
	mIntfVoltage(2, 0) = Math::realFromVectorElement(leftVector, mTerminals[0]->matrixNodeIndices()[2]);

	updateBMatrixStateSpaceModel();
}


void EMT::Ph3::AvVoltageSourceInverterDQ::addAggregatedGenProfile(std::vector<Real>* genProfile, Real customerNumber) {
	std::transform(genProfile->begin(), genProfile->end(), genProfile->begin(),
		std::bind1st(std::multiplies<Real>(), customerNumber));
	mGenProfile = genProfile;
}


void EMT::Ph3::AvVoltageSourceInverterDQ::initializeStateSpaceModel(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
	mTimeStep = timeStep;
	mOmegaN = omega;
	mOmegaCutoff = omega;

	// get current and voltage inputs to state space model
	// done here to ensure quantites are already initialized by initializeFromPowerFlow
	mIrcabc = -1 * mSubResistorC->attribute<Matrix>("i_intf")->get();
	mIrcdq = parkTransformPowerInvariant(mThetaPLL, mIrcabc(0, 0), mIrcabc(1, 0), mIrcabc(2, 0));

	MatrixComp initVcComplex = MatrixComp::Zero(3,1);
	initVcComplex(0, 0) = RMS3PH_TO_PEAK1PH * mVirtualNodes[4]->initialSingleVoltage();
	initVcComplex(1, 0) = initVcComplex(0, 0) * SHIFT_TO_PHASE_B;
	initVcComplex(2, 0) = initVcComplex(0, 0) * SHIFT_TO_PHASE_C;
	mVcabc = initVcComplex.real();
	mVcdq = parkTransformPowerInvariant(mThetaPLL, mVcabc(0, 0), mVcabc(1, 0), mVcabc(2, 0));

	updateBMatrixStateSpaceModel();

	// initialization of input
	mU << mOmegaN, mPref, mQref, mVcdq(0, 0), mVcdq(1, 0), mIrcdq(0, 0), mIrcdq(1, 0);
	mSLog->info("Initialization of input: \n" + Logger::matrixToString(mU));

	// initialization of states
	mThetaPLL = mThetaPLLInit;
	mPhiPLL = mPhiPLLInit;
	mP = mPInit;
	mQ = mQInit;
	mPhi_d = mPhi_dInit;
	mPhi_q = mPhi_qInit;
	mGamma_d = mGamma_dInit;
	mGamma_q = mGamma_qInit;
	mStates << mThetaPLL, mPhiPLL, mP, mQ, mPhi_d, mPhi_q, mGamma_d, mGamma_q;
	mSLog->info("Initialization of states: \n" + Logger::matrixToString(mStates));

	// initialization of output
	mVsdq = mC * mStates + mD * mU;
	mSLog->info("Initialization of output (in dq frame): \n" + Logger::matrixToString(mVsdq));
}


void EMT::Ph3::AvVoltageSourceInverterDQ::updatePowerGeneration() {
	if (mCurrentPower != mGenProfile->end()) {
		mPref = *mCurrentPower;
		++mCurrentPower;
	}
}


void EMT::Ph3::AvVoltageSourceInverterDQ::step(Real time, Int timeStepCount) {
	Matrix newStates = Matrix::Zero(8, 1);
	Matrix newU = Matrix::Zero(7, 1);
	if (mBehaviour == Behaviour::Simulation && mGenProfile) {
		if (timeStepCount % Int(1 / mTimeStep) == 0)
			updatePowerGeneration();
	}

	newU << mOmegaN, mPref, mQref, mVcdq(0, 0), mVcdq(1, 0), mIrcdq(0, 0), mIrcdq(1, 0);
	newStates = Math::StateSpaceTrapezoidal(mStates, mA, mB, mTimeStep, newU, mU);

	if (mCtrlOn) {
		// update states
		mThetaPLL = newStates(0, 0);
		mPhiPLL = newStates(1, 0);
		mP = newStates(2, 0);
		mQ = newStates(3, 0);
		mPhi_d = newStates(4, 0);
		mPhi_q = newStates(5, 0);
		mGamma_d = newStates(6, 0);
		mGamma_q = newStates(7, 0);

		// update measurements ( for additional loggers)
		mOmegaInst = (newStates(0, 0) - mStates(0,0))/mTimeStep;
		mFreqInst = mOmegaInst / 2 / PI;

		mStates = newStates;
		mU = newU;

		// new output
		mVsdq = mC * mStates + mD * mU;
	}
	else {
		mThetaPLL = newStates(0, 0);
		// update measurements ( for additional loggers)
		mOmegaInst = (time > 0) ? mThetaPLL / time : 0;
		mFreqInst = mOmegaInst / 2 / PI;
		mPhiPLL = newStates(1, 0);
		mP = newStates(2, 0);
		mQ = newStates(3, 0);
		mStates(0, 0) = newStates(0, 0);
		mStates(1, 0) = newStates(1, 0);
		mU = newU;

		Complex vIntfInit = RMS3PH_TO_PEAK1PH * mVirtualNodes[1]->initialSingleVoltage();
		mVsabc = inverseParkTransformPowerInvariant(mOmegaN * time + Math::phase(vIntfInit), vIntfInit.real(), vIntfInit.imag());
	}
}

void EMT::Ph3::AvVoltageSourceInverterDQ::updateBMatrixStateSpaceModel() {
	mB.coeffRef(2, 3) = mOmegaCutoff * mIrcdq(0, 0);
	mB.coeffRef(2, 4) = mOmegaCutoff * mIrcdq(1, 0);
	mB.coeffRef(3, 3) = -mOmegaCutoff * mIrcdq(1, 0);
	mB.coeffRef(3, 4) = mOmegaCutoff * mIrcdq(0, 0);
}



Matrix EMT::Ph3::AvVoltageSourceInverterDQ::parkTransformPowerInvariant(Real theta, Real fa, Real fb, Real fc) {

	// Calculates fdq = Tdq * fabc
	// Assumes that d-axis starts aligned with phase a

	Matrix fabc = Matrix::Zero(3, 1);
	fabc << fa, fb, fc;
	Matrix Tdq = getParkTransformMatrixPowerInvariant(theta);
	Matrix dqvector = Tdq * fabc;
	return dqvector;
}


Matrix EMT::Ph3::AvVoltageSourceInverterDQ::getParkTransformMatrixPowerInvariant(Real theta) {

	// Return park matrix for theta
	// Assumes that d-axis starts aligned with phase a
	Matrix Tdq = Matrix::Zero(2, 3);
	Real k = sqrt(2. / 3.);
	Tdq <<
		k * cos(theta), k * cos(theta - 2. * M_PI / 3.), k * cos(theta + 2. * M_PI / 3.),
		-k * sin(theta), -k * sin(theta - 2. * M_PI / 3.), -k * sin(theta + 2. * M_PI / 3.);
	return Tdq;
}

Matrix EMT::Ph3::AvVoltageSourceInverterDQ::inverseParkTransformPowerInvariant(Real theta, Real fd, Real fq) {

	// Calculates fabc = Tabc * fdq
	// with d-axis starts aligned with phase a
	Matrix fabc = Matrix::Zero(3, 1);
	Matrix fdq = Matrix::Zero(2, 1);
	fdq << fd, fq;
	Matrix Tabc = getInverseParkTransformMatrixPowerInvariant(theta);
	fabc = Tabc * fdq;

	return fabc;
}


Matrix EMT::Ph3::AvVoltageSourceInverterDQ::getInverseParkTransformMatrixPowerInvariant(Real theta) {

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

void EMT::Ph3::AvVoltageSourceInverterDQ::initializeFromPowerflow(Real frequency) {

	// use complex interface quantities for initialization calculations
	MatrixComp intfVoltageComplex = Matrix::Zero(3, 1);
	MatrixComp intfCurrentComplex = Matrix::Zero(3, 1);

	// derive complex threephase initialization from single phase initial values (only valid for balanced systems)
	intfVoltageComplex(0, 0) = RMS3PH_TO_PEAK1PH * initialSingleVoltage(0);
	intfVoltageComplex(1, 0) = intfVoltageComplex(0, 0) * SHIFT_TO_PHASE_B;
	intfVoltageComplex(2, 0) = intfVoltageComplex(0, 0) * SHIFT_TO_PHASE_C;
	intfCurrentComplex(0, 0) = -std::conj(2./3.*Complex(mPref, mQref) / intfVoltageComplex(0, 0));
	intfCurrentComplex(1, 0) = intfCurrentComplex(0, 0) * SHIFT_TO_PHASE_B;
	intfCurrentComplex(2, 0) = intfCurrentComplex(0, 0) * SHIFT_TO_PHASE_C;

	MatrixComp filterInterfaceInitialVoltage = MatrixComp::Zero(3, 1);
	MatrixComp filterInterfaceInitialCurrent = MatrixComp::Zero(3, 1);
	if (mWithConnectionTransformer) {
		// calculate quantities of low voltage side of transformer (being the interface quantities of the filter, calculations only valid for symmetrical systems)
		// TODO: check possibility of more accurate solution as current only approximated
		filterInterfaceInitialVoltage = (intfVoltageComplex - Complex(mTransformerResistance, mTransformerInductance*mOmegaN)*intfCurrentComplex) / Complex(mTransformerRatioAbs, mTransformerRatioPhase);
		filterInterfaceInitialCurrent = intfCurrentComplex * Complex(mTransformerRatioAbs, mTransformerRatioPhase);

		// connect and init transformer
		mVirtualNodes[4]->setInitialVoltage(PEAK1PH_TO_RMS3PH * filterInterfaceInitialVoltage);
		mConnectionTransformer->connect({ mTerminals[0]->node(), mVirtualNodes[4] });
		mConnectionTransformer->setParameters(mTransformerRatioAbs, mTransformerRatioPhase, Matrix::Identity(3,3)*mTransformerResistance, Matrix::Identity(3,3)*mTransformerInductance);
		mConnectionTransformer->initialize(mFrequencies);
		mConnectionTransformer->initializeFromPowerflow(frequency);
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
	mVirtualNodes[1]->setInitialVoltage(PEAK1PH_TO_RMS3PH * vsInit);
	mVirtualNodes[2]->setInitialVoltage(PEAK1PH_TO_RMS3PH * vfInit);
	mVirtualNodes[3]->setInitialVoltage(PEAK1PH_TO_RMS3PH * vcInit);

	// save real interface quantities calculated from complex ones
	mIntfVoltage = intfVoltageComplex.real();
	mIntfCurrent = intfCurrentComplex.real();

	// Create sub components
	mSubResistorF = EMT::Ph3::Resistor::make(mName + "_resF", mLogLevel);
	mSubResistorC = EMT::Ph3::Resistor::make(mName + "_resC", mLogLevel);
	mSubCapacitorF = EMT::Ph3::Capacitor::make(mName + "_capF", mLogLevel);
	mSubInductorF = EMT::Ph3::Inductor::make(mName + "_indF", mLogLevel);
	mSubCtrledVoltageSource = EMT::Ph3::ControlledVoltageSource::make(mName + "_src", mLogLevel);

	// set filter parameters
	mSubResistorC->setParameters(Matrix::Identity(3,3)*mRc);
	mSubResistorF->setParameters(Matrix::Identity(3,3)*mRf);
	mSubInductorF->setParameters(Matrix::Identity(3,3)*mLf);
	mSubCapacitorF->setParameters(Matrix::Identity(3,3)*mCf);
	mSubCtrledVoltageSource->setParameters(vsInit.real());

	// connect subcomponents
	mSubCtrledVoltageSource->connect({ SimNode::GND, mVirtualNodes[1] });
	mSubCtrledVoltageSource->setVirtualNodeAt(mVirtualNodes[0], 0);
	mSubResistorF->connect({ mVirtualNodes[1], mVirtualNodes[2] });
	mSubInductorF->connect({ mVirtualNodes[2], mVirtualNodes[3] });
	mSubCapacitorF->connect({ mVirtualNodes[3], SimNode::GND });
	if (mWithConnectionTransformer)
		mSubResistorC->connect({ mVirtualNodes[3],  mVirtualNodes[4]});
	else
		mSubResistorC->connect({ mVirtualNodes[3],  mTerminals[0]->node()});

	// initialize subcomponents
	//mSubCtrledVoltageSource->initialize(mFrequencies);
	//mSubResistorF->initialize(mFrequencies);
	//mSubInductorF->initialize(mFrequencies);
	//mSubCapacitorF->initialize(mFrequencies);
	//mSubResistorC->initialize(mFrequencies);

	//mSubCtrledVoltageSource->initializeFromPowerflow(frequency);
	mSubResistorF->initializeFromPowerflow(frequency);
	mSubInductorF->initializeFromPowerflow(frequency);
	mSubCapacitorF->initializeFromPowerflow(frequency);
	mSubResistorC->initializeFromPowerflow(frequency);

	mSLog->info(
		"\n--- Initialization from powerflow ---"
		"\nVoltage across: {:s}"
		"\nCurrent: {:s}"
		"\nTerminal 0 voltage: {:s}"
		"\nTerminal 0 connected to {:s} = sim node {:d}"
		"\n--- Initialization from powerflow finished ---",
		Logger::phasorToString(intfVoltageComplex(0, 0)),
		Logger::phasorToString(intfCurrentComplex(0, 0)),
		Logger::phasorToString(RMS3PH_TO_PEAK1PH * initialSingleVoltage(0)),
		mTerminals[0]->node()->name(), mTerminals[0]->node()->matrixNodeIndex());
}

void EMT::Ph3::AvVoltageSourceInverterDQ::mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
	MNAInterface::mnaInitialize(omega, timeStep);
	updateMatrixNodeIndices();
	mTimeStep = timeStep;

	// set powers from profiles
	if (mGenProfile)
		mCurrentPower = mGenProfile->begin();

	MNAInterface::List subComps({ mSubResistorF, mSubInductorF, mSubCapacitorF, mSubResistorC, mSubCtrledVoltageSource });

	mSubResistorF->mnaInitialize(omega, timeStep, leftVector);
	mSubInductorF->mnaInitialize(omega, timeStep, leftVector);
	mSubCapacitorF->mnaInitialize(omega, timeStep, leftVector);
	mSubResistorC->mnaInitialize(omega, timeStep, leftVector);
	mSubCtrledVoltageSource->mnaInitialize(omega, timeStep, leftVector);
	initializeStateSpaceModel(omega, timeStep, leftVector);

	mRightVectorStamps.push_back(&mSubCapacitorF->attribute<Matrix>("right_vector")->get());
	mRightVectorStamps.push_back(&mSubInductorF->attribute<Matrix>("right_vector")->get());
	mRightVectorStamps.push_back(&mSubCtrledVoltageSource->attribute<Matrix>("right_vector")->get());

	// add tasks
	for (auto comp : subComps) {
		for (auto task : comp->mnaTasks())
			mMnaTasks.push_back(task);
	}
	if (mWithConnectionTransformer) {
		mConnectionTransformer->mnaInitialize(omega, timeStep, leftVector);
		mRightVectorStamps.push_back(&mConnectionTransformer->attribute<Matrix>("right_vector")->get());
		for (auto task : mConnectionTransformer->mnaTasks()) {
			mMnaTasks.push_back(task);
		}
	}
	mMnaTasks.push_back(std::make_shared<MnaPreStep>(*this));
	mMnaTasks.push_back(std::make_shared<AddBStep>(*this));
	//mMnaTasks.push_back(std::make_shared<CtrlStep>(*this));
	mMnaTasks.push_back(std::make_shared<MnaPostStep>(*this, leftVector));
	mRightVector = Matrix::Zero(leftVector->get().rows(), 1);
}


void EMT::Ph3::AvVoltageSourceInverterDQ::mnaApplySystemMatrixStamp(Matrix& systemMatrix) {
	mSubCtrledVoltageSource->mnaApplySystemMatrixStamp(systemMatrix);
	mSubResistorF->mnaApplySystemMatrixStamp(systemMatrix);
	mSubInductorF->mnaApplySystemMatrixStamp(systemMatrix);
	mSubCapacitorF->mnaApplySystemMatrixStamp(systemMatrix);
	mSubResistorC->mnaApplySystemMatrixStamp(systemMatrix);
	if (mWithConnectionTransformer)
		mConnectionTransformer->mnaApplySystemMatrixStamp(systemMatrix);
}

void EMT::Ph3::AvVoltageSourceInverterDQ::mnaApplyRightSideVectorStamp(Matrix& rightVector) {
	rightVector.setZero();
	for (auto stamp : mRightVectorStamps)
		rightVector += *stamp;
}

void EMT::Ph3::AvVoltageSourceInverterDQ::updateSetPoint(Real time) {
	if (mQRefInput)
		mQref = mQRefInput->get();
}

void EMT::Ph3::AvVoltageSourceInverterDQ::MnaPreStep::execute(Real time, Int timeStepCount) {
	mAvVoltageSourceInverterDQ.mVsabc =  mAvVoltageSourceInverterDQ.inverseParkTransformPowerInvariant(mAvVoltageSourceInverterDQ.mThetaPLL, mAvVoltageSourceInverterDQ.mVsdq(0, 0), mAvVoltageSourceInverterDQ.mVsdq(1, 0));
	mAvVoltageSourceInverterDQ.mSubCtrledVoltageSource->setParameters(mAvVoltageSourceInverterDQ.mVsabc);
}

void EMT::Ph3::AvVoltageSourceInverterDQ::MnaPostStep::execute(Real time, Int timeStepCount) {
	mAvVoltageSourceInverterDQ.mnaUpdateCurrent(*mLeftVector);
	mAvVoltageSourceInverterDQ.updateInputStateSpaceModel(*mLeftVector, time);
	mAvVoltageSourceInverterDQ.step(time, timeStepCount);
}

void EMT::Ph3::AvVoltageSourceInverterDQ::AddBStep::execute(Real time, Int timeStepCount) {
	mAvVoltageSourceInverterDQ.mnaApplyRightSideVectorStamp(mAvVoltageSourceInverterDQ.mRightVector);
}

void EMT::Ph3::AvVoltageSourceInverterDQ::mnaUpdateCurrent(const Matrix& leftvector) {
	if (mWithConnectionTransformer)
		mIntfCurrent = mConnectionTransformer->attribute<Matrix>("i_intf")->get();
	else
		mIntfCurrent = mSubResistorC->attribute<Matrix>("i_intf")->get();
}

//void EMT::Ph3::AvVoltageSourceInverterDQ::CtrlStep::execute(Real time, Int timeStepCount) {
//	mAvVoltageSourceInverterDQ.updateSetPoint(time);
//}
