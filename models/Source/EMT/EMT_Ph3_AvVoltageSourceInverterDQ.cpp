/* Copyright 2017-2020 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <cps/EMT/EMT_Ph3_AvVoltageSourceInverterDQ.h>

#define SHIFT_TO_PHASE_B Complex(cos(-2 * M_PI / 3), sin(-2 * M_PI / 3))
#define SHIFT_TO_PHASE_C Complex(cos(2 * M_PI / 3), sin(2 * M_PI / 3))


using namespace CPS;

EMT::Ph3::AvVoltageSourceInverterDQ::AvVoltageSourceInverterDQ(String uid, String name, Logger::Level logLevel, Bool withTrafo)
	: SimPowerComp<Real>(uid,name,logLevel), TopologicalPowerComp(uid, name, logLevel){
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

	addAttribute<Real>("p", &mP, Flags::read | Flags::write);
	addAttribute<Real>("q", &mQ, Flags::read | Flags::write);
	addAttribute<Real>("P_ref", &mPref, Flags::read | Flags::write);
	addAttribute<Real>("Q_ref", &mQref, Flags::read | Flags::write);
	addAttribute<Real>("theta", &mThetaPLL, Flags::read | Flags::write);
	addAttribute<Real>("phipll", &mPhiPLL, Flags::read | Flags::write);
	addAttribute<Real>("phid", &mPhi_d, Flags::read | Flags::write);
	addAttribute<Real>("phiq", &mPhi_q, Flags::read | Flags::write);
	addAttribute<Real>("gammad", &mGamma_d, Flags::read | Flags::write);
	addAttribute<Real>("gammaq", &mGamma_q, Flags::read | Flags::write);
	addAttribute<Bool>("ctrl_on", &mCtrlOn, Flags::read | Flags::write);

	//additional loggers
	addAttribute<Matrix>("Vcabc", &mVcabc, Flags::read | Flags::write);
	addAttribute<Matrix>("Vsdq", &mVsdq, Flags::read | Flags::write);
	addAttribute<Matrix>("Vcdq", &mVcdq, Flags::read | Flags::write);
	addAttribute<Matrix>("ifabc", &mIfabc, Flags::read | Flags::write);
	addAttribute<Matrix>("igdq", &mIgdq, Flags::read | Flags::write);
	addAttribute<Matrix>("ifdq", &mIfdq, Flags::read | Flags::write);
	addAttribute<Real>("omega", &mOmegaInst, Flags::read | Flags::write);
	addAttribute<Real>("freq", &mFreqInst, Flags::read | Flags::write);
}

void EMT::Ph3::AvVoltageSourceInverterDQ::setTransformerParameters(Real nomVoltageEnd1, Real nomVoltageEnd2, Real ratedPower, Real ratioAbs,
	Real ratioPhase, Matrix resistance, Matrix inductance, Real omega) {

	mTransformerResistance = resistance;
	mTransformerInductance = inductance;
	mTransformerRatioAbs = ratioAbs;
	mTransformerRatioPhase = ratioPhase;

	mSLog->info("Connection Transformer Parameters:");
	mSLog->info("Resistance={} [Ohm] Inductance={} [H]", mTransformerResistance, mTransformerInductance);
    mSLog->info("Tap Ratio={} [ ] Phase Shift={} [deg]", mTransformerRatioAbs, mTransformerRatioPhase);

	mConnectionTransformer->setParameters(mTransformerRatioAbs, mTransformerRatioPhase, mTransformerResistance, mTransformerInductance);
};

void EMT::Ph3::AvVoltageSourceInverterDQ::setParameters(Real sysOmega, Real sysVoltNom, Real Pref, Real Qref) {
	mPref = Pref;
	mQref = Qref;

	mVnom = sysVoltNom;
	mOmegaN = sysOmega;

	mSLog->info("General Parameters:");
	mSLog->info("Nominal Voltage={} [V] Nominal Omega={} [1/s]", mVnom, mOmegaN);
	mSLog->info("Active Power={} [W] Reactive Power={} [VAr]", mPref, mQref);    

	parametersSet = true;
}

void EMT::Ph3::AvVoltageSourceInverterDQ::setControllerParameters(Real Kp_pll, Real Ki_pll, Real Kp_powerCtrl, Real Ki_powerCtrl, Real Kp_currCtrl, Real Ki_currCtrl, Real Omega_cutoff) {
	mKpPLL = Kp_pll;
	mKiPLL = Ki_pll;

	mKiPowerCtrld = Ki_powerCtrl;
	mKiPowerCtrlq = Ki_powerCtrl;

	mKpPowerCtrld = Kp_powerCtrl;
	mKpPowerCtrlq = Kp_powerCtrl;

	mKiCurrCtrld = Ki_currCtrl;
	mKiCurrCtrlq = Ki_currCtrl;

	mKpCurrCtrld = Kp_currCtrl;
	mKpCurrCtrlq = Kp_currCtrl;

	mOmegaCutoff = Omega_cutoff;

	mSLog->info("Control Parameters:");
	mSLog->info("PLL: K_i = {}, K_p = {}", mKpPLL, mKiPLL);
	mSLog->info("Power Loop: K_i = {}, K_p = {}", mKpPowerCtrld, mKiPowerCtrld);
	mSLog->info("Current Loop: K_i = {}, K_p = {}", mKpCurrCtrld, mKiCurrCtrld);
	mSLog->info("Cut-Off Frequency = {}", mOmegaCutoff);
}

void EMT::Ph3::AvVoltageSourceInverterDQ::setFilterParameters(Matrix Lf, Matrix Cf, Matrix Rf, Matrix Rc) {
	mLf = Lf;
	mCf = Cf;
	mRf = Rf;
	mRc = Rc;

	mSLog->info("Filter Parameters:");
	mSLog->info("Inductance Lf={} [H] Capacitance Cf={} [F]", mLf, mCf);
	mSLog->info("Resistance Rf={} [H] Resistance Rc={} [F]", mRf, mRc);
}


SimPowerComp<Real>::Ptr EMT::Ph3::AvVoltageSourceInverterDQ::clone(String name) {
	auto copy = EMT::Ph3::AvVoltageSourceInverterDQ::make(name, mLogLevel);
	copy->setParameters(mOmegaN, mVnom, mPref, mQref);
	return copy;
}


void EMT::Ph3::AvVoltageSourceInverterDQ::updateMonitoredValues(const Matrix& leftVector, Real time) {

	mVcabc(0, 0) = Math::realFromVectorElement(leftVector, mSubCapacitorF->matrixNodeIndex(0, 0));
	mVcabc(1, 0) = Math::realFromVectorElement(leftVector, mSubCapacitorF->matrixNodeIndex(0, 1));
	mVcabc(2, 0) = Math::realFromVectorElement(leftVector, mSubCapacitorF->matrixNodeIndex(0, 2));

	mIfabc = -1 * mSubResistorF->attribute<Matrix>("i_intf")->get();
	mIgabc = -1. * mSubResistorC->attribute<Matrix>("i_intf")->get();
	mIfdq = parkTransform(mThetaSInit + mThetaPLL, mIfabc(0, 0), mIfabc(1, 0), mIfabc(2, 0));
	mIgdq = parkTransform(mThetaSInit + mThetaPLL, mIgabc(0, 0), mIgabc(1, 0), mIgabc(2, 0));
	mVcdq = parkTransform(mThetaSInit + mThetaPLL, mVcabc(0, 0), mVcabc(1, 0), mVcabc(2, 0));
	mIntfVoltage(0, 0) = Math::realFromVectorElement(leftVector, mTerminals[0]->matrixNodeIndices()[0]);
	mIntfVoltage(1, 0) = Math::realFromVectorElement(leftVector, mTerminals[0]->matrixNodeIndices()[1]);
	mIntfVoltage(2, 0) = Math::realFromVectorElement(leftVector, mTerminals[0]->matrixNodeIndices()[2]);

	updateLinearizedModel();
}

void EMT::Ph3::AvVoltageSourceInverterDQ::addAggregatedGenProfile(std::vector<Real>* genProfile, Real customerNumber) {
	std::transform(genProfile->begin(), genProfile->end(), genProfile->begin(),
		std::bind1st(std::multiplies<Real>(), customerNumber));
	mGenProfile = genProfile;
}



void EMT::Ph3::AvVoltageSourceInverterDQ::initializeModel(Real omega, Real timeStep,
	Attribute<Matrix>::Ptr leftVector) {
	mTimeStep = timeStep;
	mOmegaN = omega;
	mOmegaCutoff = omega;
	mThetaPLL = 0;

	// initialize mInftCurrent and mIgabc
	mIntfCurrent = mSubResistorC->attribute<Matrix>("i_intf")->get();
	mIfabc = -1 * mSubResistorF->attribute<Matrix>("i_intf")->get();
	mIgabc = -1. * mSubResistorC->attribute<Matrix>("i_intf")->get();
	mIfdq = parkTransform(mThetaPLL, mIfabc(0, 0), mIfabc(1, 0), mIfabc(2, 0));
	mIgdq = parkTransform(mThetaPLL, mIgabc(0, 0), mIgabc(1, 0), mIgabc(2, 0));
	mVcdq = parkTransform(mThetaPLL, mVcabc(0, 0), mVcabc(1, 0), mVcabc(2, 0));

	mU = Matrix::Zero(7, 1);
	mU <<
		mOmegaN, mPref, mQref, mVcdq(0, 0), mVcdq(1, 0), mIfdq(0, 0), mIfdq(1, 0);

	// create matrices for state space representation
	mA = Matrix::Zero(8, 8);
	mB = Matrix::Zero(8, 7);
	mC = Matrix::Zero(2, 8);
	mD = Matrix::Zero(2, 7);

	mA <<
		0, mKiPLL, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, -mOmegaCutoff, 0, 0, 0, 0, 0,
		0, 0, 0, -mOmegaCutoff, 0, 0, 0, 0,
		0, 0, -1, 0, 0, 0, 0, 0,
		0, 0, 0, 1, 0, 0, 0, 0,
		0, 0, -mKpPowerCtrld, 0, mKiPowerCtrld, 0, 0, 0,
		0, 0, 0, mKpPowerCtrlq, 0, mKiPowerCtrlq, 0, 0;

	mB <<
		1, 0, 0, 0, mKpPLL, 0, 0,
		0, 0, 0, 0, 1, 0, 0,
		0, 0, 0, 3. / 2. * mOmegaCutoff * mIgdq(0, 0), 3. / 2. * mOmegaCutoff * mIgdq(1, 0), 0, 0,
		0, 0, 0, -3. / 2. * mOmegaCutoff * mIgdq(1, 0), 3. / 2. * mOmegaCutoff * mIgdq(0, 0), 0, 0,
		0, 1, 0, 0, 0, 0, 0,
		0, 0, -1, 0, 0, 0, 0,
		0, mKpPowerCtrld, 0, 0, 0, -1, 0,
		0, 0, -mKpPowerCtrlq, 0, 0, 0, -1;

	mC <<
		0, 0, -mKpPowerCtrld * mKpCurrCtrld, 0, mKpCurrCtrld* mKiPowerCtrld, 0, mKiCurrCtrld, 0,
		0, 0, 0, mKpPowerCtrlq* mKpCurrCtrlq, 0, mKpCurrCtrlq* mKiPowerCtrlq, 0, mKiCurrCtrlq;

	mD <<
		0, mKpCurrCtrld * mKpPowerCtrld, 0, 0, 0, -mKpCurrCtrld, 0,
		0, 0, -mKpCurrCtrlq * mKpPowerCtrlq, 0, 0, 0, -mKpCurrCtrlq;


	mPhiPLL = 0;

	//mP = (3. / 2.  * (Td * mIgabc * Td * mVcabc + Tq * mIgabc * Tq * mVcabc)).coeff(0, 0);
	//mQ = (3. / 2.  * (Td * mIgabc * Tq * mVcabc - Tq * mIgabc * Td * mVcabc)).coeff(0, 0);
	mP = 0;
	mQ = 0;
	mPhi_d = 0;
	mPhi_q = 0;
	mGamma_d = 0;
	mGamma_q = 0;

	mStates = Matrix::Zero(8, 1);
	mStates <<
		mThetaPLL, mPhiPLL, mP, mQ, mPhi_d, mPhi_q, mGamma_d, mGamma_q;
	// initialize output
	mVsdq = mC * mStates + mD * mU;
	mVsabc = inverseParkTransform(mThetaPLL, mVsdq(0, 0), mVsdq(1, 0));
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
		//updateSetPoint(time);
		newU <<
			mOmegaN, mPref, mQref, mVcdq(0, 0), mVcdq(1, 0), mIfdq(0, 0), mIfdq(1, 0);

		newStates = Math::StateSpaceTrapezoidal(mStates, mA, mB, mTimeStep, newU, mU);

		// update states
		// update measurements ( for additional loggers)


	if (mCtrlOn) {
		mOmegaInst = (time > 0) ? mThetaPLL / time : 0;
		mFreqInst = mOmegaInst / 2 / PI;
		mThetaPLL = newStates(0, 0);
		mPhiPLL = newStates(1, 0);
		mP = newStates(2, 0);
		mQ = newStates(3, 0);
		mPhi_d = newStates(4, 0);
		mPhi_q = newStates(5, 0);
		mGamma_d = newStates(6, 0);
		mGamma_q = newStates(7, 0);

		mStates = newStates;
		mU = newU;
		// new output
		mVsdq = mC * mStates + mD * mU;
		mVsabc = inverseParkTransform(mThetaPLL, mVsdq(0, 0), mVsdq(1, 0));
	}
	else
	{
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
		Complex vIntfInit = mVirtualNodes[1]->initialSingleVoltage();
		mVsabc = inverseParkTransform(mOmegaN * time + Math::phase(vIntfInit), vIntfInit.real(), vIntfInit.imag());
	}
}

void EMT::Ph3::AvVoltageSourceInverterDQ::updateLinearizedModel() {
	mB.coeffRef(2, 3) = 3. / 2. * mOmegaCutoff * mIgdq(0, 0);
	mB.coeffRef(2, 4) = 3. / 2. * mOmegaCutoff * mIgdq(1, 0);
	mB.coeffRef(3, 3) = -3. / 2. * mOmegaCutoff * mIgdq(1, 0);
	mB.coeffRef(3, 4) = 3. / 2. * mOmegaCutoff * mIgdq(0, 0);
}



Matrix EMT::Ph3::AvVoltageSourceInverterDQ::parkTransform(Real theta, Real fa, Real fb, Real fc) {

	/// with d-axis starts 90deg behind to phase a
	//	Matrix Tdq = Matrix::Zero(2, 3);
//	Tdq <<
//		2. / 3. * sin(theta), 2. / 3. * sin(theta - 2. * M_PI / 3.), 2. / 3. * sin(theta + 2. * M_PI / 3.),
//		2. / 3. * cos(theta), 2. / 3. * cos(theta - 2. * M_PI / 3.), 2. / 3. * cos(theta + 2. * M_PI / 3.);
//
//	return Tdq;

/// with d-axis starts aligned with phase a
	Matrix fabc = Matrix::Zero(3, 1);
	fabc <<
		fa, fb, fc;
	Matrix Tdq = Matrix::Zero(2, 3);
	Tdq <<
		2. / 3. * cos(theta), 2. / 3. * cos(theta - 2. * M_PI / 3.), 2. / 3. * cos(theta + 2. * M_PI / 3.),
		-2. / 3. * sin(theta), -2. / 3. * sin(theta - 2. * M_PI / 3.), -2. / 3. * sin(theta + 2. * M_PI / 3.);
	Matrix dqvector = Tdq * fabc;

	return dqvector;

}

// fdq = Tdq * fabc
Matrix EMT::Ph3::AvVoltageSourceInverterDQ::getParkTransformMatrix(Real theta) {

	/// with d-axis starts 90deg behind to phase a
	//	Matrix Tdq = Matrix::Zero(2, 3);
//	Tdq <<
//		2. / 3. * sin(theta), 2. / 3. * sin(theta - 2. * M_PI / 3.), 2. / 3. * sin(theta + 2. * M_PI / 3.),
//		2. / 3. * cos(theta), 2. / 3. * cos(theta - 2. * M_PI / 3.), 2. / 3. * cos(theta + 2. * M_PI / 3.);
//
//	return Tdq;

/// with d-axis starts aligned with phase a
	Matrix Tdq = Matrix::Zero(2, 3);
	Tdq <<
		2. / 3. * cos(theta), 2. / 3. * cos(theta - 2. * M_PI / 3.), 2. / 3. * cos(theta + 2. * M_PI / 3.),
		-2. / 3. * sin(theta), -2. / 3. * sin(theta - 2. * M_PI / 3.), -2. / 3. * sin(theta + 2. * M_PI / 3.);

	return Tdq;
}

// fabc = Tabc * fdq
Matrix EMT::Ph3::AvVoltageSourceInverterDQ::inverseParkTransform(Real theta, Real fd, Real fq) {
	Matrix fabc = Matrix::Zero(3, 1);
	Matrix fdq = Matrix::Zero(2, 1);
	fdq <<
		fd, fq;

	/// with d-axis starts 90deg behind to phase a
	//	Matrix Tabc = Matrix::Zero(3, 2);
	//	Tabc <<
	//		sin(theta), cos(theta),
	//		sin(theta - 2. * M_PI / 3.), cos(theta - 2. * M_PI / 3.),
	//		sin(theta + 2. * M_PI / 3.), cos(theta + 2. * M_PI / 3.);
	//
	//	return Tabc;


	/// with d-axis starts aligned with phase a
	Matrix Tabc = Matrix::Zero(3, 2);
	Tabc <<
		cos(theta), -sin(theta),
		cos(theta - 2. * M_PI / 3.), -sin(theta - 2. * M_PI / 3.),
		cos(theta + 2. * M_PI / 3.), -sin(theta + 2. * M_PI / 3.);

	fabc = Tabc * fdq;

	return fabc;
}


Matrix EMT::Ph3::AvVoltageSourceInverterDQ::getInverseParkTransformMatrix(Real theta) {

/// with d-axis starts 90deg behind to phase a
//	Matrix Tabc = Matrix::Zero(3, 2);
//	Tabc <<
//		sin(theta), cos(theta),
//		sin(theta - 2. * M_PI / 3.), cos(theta - 2. * M_PI / 3.),
//		sin(theta + 2. * M_PI / 3.), cos(theta + 2. * M_PI / 3.);
//
//	return Tabc;


/// with d-axis starts aligned with phase a
	Matrix Tabc = Matrix::Zero(3, 2);
	Tabc <<
		cos(theta), -sin(theta),
		cos(theta - 2. * M_PI / 3.), -sin(theta - 2. * M_PI / 3.),
		cos(theta + 2. * M_PI / 3.), -sin(theta + 2. * M_PI / 3.);

	return Tabc;
}

void EMT::Ph3::AvVoltageSourceInverterDQ::initializeFromPowerflow(Real frequency) {

	checkForUnconnectedTerminals();

	// use complex interface quantities for initialization calculations
	MatrixComp intfVoltageComplex = Matrix::Zero(3, 1);
	MatrixComp intfCurrentComplex = Matrix::Zero(3, 1);

	// derive complex threephase initialization from single phase initial values (only valid for balanced systems)
	intfVoltageComplex(0, 0) = initialSingleVoltage(0);
	intfVoltageComplex(1, 0) = intfVoltageComplex(0, 0) * SHIFT_TO_PHASE_B;
	intfVoltageComplex(2, 0) = intfVoltageComplex(0, 0) * SHIFT_TO_PHASE_C;
	intfCurrentComplex(0, 0) = -std::conj(Complex(mPref, mQref) / intfVoltageComplex(0, 0));
	intfCurrentComplex(1, 0) = intfCurrentComplex(0, 0) * SHIFT_TO_PHASE_B;
	intfCurrentComplex(2, 0) = intfCurrentComplex(0, 0) * SHIFT_TO_PHASE_C;

	MatrixComp filterInterfaceInitialVoltage = MatrixComp::Zero(3, 1);
	MatrixComp filterInterfaceInitialCurrent = MatrixComp::Zero(3, 1);
	if (mWithConnectionTransformer) {		
		// calculate quantities of low voltage side of transformer (being the interface quantities of the filter, calculations only valid for symmetrical systems)
		// TODO: check possibility of more accurate solution as current only approximated
		filterInterfaceInitialVoltage = (intfVoltageComplex - Complex(mTransformerResistance(0, 0), mTransformerInductance(0, 0)*mOmegaN)*intfCurrentComplex) / Complex(mTransformerRatioAbs, mTransformerRatioPhase);
		filterInterfaceInitialCurrent = intfCurrentComplex * Complex(mTransformerRatioAbs, mTransformerRatioPhase);

		// connect and init transformer
		mVirtualNodes[4]->setInitialVoltage(filterInterfaceInitialVoltage);		
		mConnectionTransformer->connect({ mTerminals[0]->node(), mVirtualNodes[4] });
		mConnectionTransformer->initialize(mFrequencies);
		mConnectionTransformer->initializeFromPowerflow(frequency);
	} else {
		// if no transformer used, filter interface equal to inverter interface
		filterInterfaceInitialVoltage = intfVoltageComplex;
		filterInterfaceInitialCurrent = intfCurrentComplex;
	}

	// derive initialization quantities of filter (calculations only valid for symmetrical systems)
	MatrixComp vcInit = filterInterfaceInitialVoltage - filterInterfaceInitialCurrent * Complex(mRc(0, 0), 0);
	MatrixComp icfInit = vcInit * Complex(0., 2. * PI * frequency * mCf(0, 0));
	MatrixComp vfInit = vcInit - (filterInterfaceInitialCurrent - icfInit) * Complex(0., 2. * PI * frequency * mLf(0, 0));
	MatrixComp vsInit = vfInit - (filterInterfaceInitialCurrent - icfInit) * Complex(mRf(0, 0), 0);
	mVirtualNodes[1]->setInitialVoltage(vsInit);
	mVirtualNodes[2]->setInitialVoltage(vfInit);
	mVirtualNodes[3]->setInitialVoltage(vcInit);

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
	mSubResistorC->setParameters(mRc);
	mSubResistorF->setParameters(mRf);
	mSubInductorF->setParameters(mLf);
	mSubCapacitorF->setParameters(mCf);
	mSubCtrledVoltageSource->setParameters(vsInit.real());

	// connect subcomponents
	mSubCtrledVoltageSource->connect({ SimNode::GND, mVirtualNodes[1] });
	mSubCtrledVoltageSource->setVirtualNodeAt(mVirtualNodes[0], 0);
	mSubResistorF->connect({ mVirtualNodes[1], mVirtualNodes[2] });
	mSubInductorF->connect({ mVirtualNodes[2], mVirtualNodes[3] });
	mSubCapacitorF->connect({ mVirtualNodes[3], SimNode::GND });
	mSubResistorC->connect({ mVirtualNodes[3], mTerminals[0]->node() });
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
		Logger::phasorToString(initialSingleVoltage(0)),
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
	initializeModel(omega, timeStep, leftVector);
	mSubResistorF->mnaInitialize(omega, timeStep, leftVector);
	mSubInductorF->mnaInitialize(omega, timeStep, leftVector);
	mSubCapacitorF->mnaInitialize(omega, timeStep, leftVector);
	mSubResistorC->mnaInitialize(omega, timeStep, leftVector);
	mSubCtrledVoltageSource->mnaInitialize(omega, timeStep, leftVector);

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


void EMT::Ph3::AvVoltageSourceInverterDQ::mnaUpdateCurrent(const Matrix& leftvector) {
	mIntfCurrent = mSubResistorC->attribute<Matrix>("i_intf")->get();
}

void EMT::Ph3::AvVoltageSourceInverterDQ::updateSetPoint(Real time) {
	if (mQRefInput)
		mQref = mQRefInput->get();
}

void EMT::Ph3::AvVoltageSourceInverterDQ::MnaPreStep::execute(Real time, Int timeStepCount) {
	// update voltage of subVoltage source
	mAvVoltageSourceInverterDQ.mSubCtrledVoltageSource->setParameters(mAvVoltageSourceInverterDQ.mVsabc);
}

void EMT::Ph3::AvVoltageSourceInverterDQ::MnaPostStep::execute(Real time, Int timeStepCount) {
	mAvVoltageSourceInverterDQ.mnaUpdateCurrent(*mLeftVector);
	// update Vcabc, Igabc
	mAvVoltageSourceInverterDQ.updateMonitoredValues(*mLeftVector, time);
	mAvVoltageSourceInverterDQ.step(time, timeStepCount);
}

void EMT::Ph3::AvVoltageSourceInverterDQ::AddBStep::execute(Real time, Int timeStepCount) {
	mAvVoltageSourceInverterDQ.mnaApplyRightSideVectorStamp(mAvVoltageSourceInverterDQ.mRightVector);
}

//void EMT::Ph3::AvVoltageSourceInverterDQ::CtrlStep::execute(Real time, Int timeStepCount) {
//	mAvVoltageSourceInverterDQ.updateSetPoint(time);
//}
