/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <cps/SP/SP_Ph1_SynchronGeneratorTrStab.h>
using namespace CPS;

SP::Ph1::SynchronGeneratorTrStab::SynchronGeneratorTrStab(String uid, String name, Logger::Level logLevel)
	: SimPowerComp<Complex>(uid, name, logLevel) {
	setVirtualNodeNumber(2);
	setTerminalNumber(1);
	mIntfVoltage = MatrixComp::Zero(1, 1);
	mIntfCurrent = MatrixComp::Zero(1, 1);

	// Register attributes
	addAttribute<Complex>("Ep", &mEp, Flags::read);
	addAttribute<Real>("Ep_mag", &mEp_abs, Flags::read);
	addAttribute<Real>("Ep_phase", &mEp_phase, Flags::read);
	addAttribute<Real>("delta_r", &mDelta_p, Flags::read);
	addAttribute<Real>("P_elec", &mElecActivePower, Flags::read);
	addAttribute<Real>("Q_elec", &mElecReactivePower, Flags::read);
	addAttribute<Real>("P_mech", &mMechPower, Flags::read);
	addAttribute<Real>("w_r", &mOmMech, Flags::read);
	addAttribute<Real>("inertia", &mInertia, Flags::read | Flags::write);
	addAttribute<Real>("w_ref", Flags::read | Flags::write);
	addAttribute<Real>("delta_ref", Flags::read | Flags::write);

	mStates = Matrix::Zero(10,1);
}

SimPowerComp<Complex>::Ptr SP::Ph1::SynchronGeneratorTrStab::clone(String name) {
	auto copy = SynchronGeneratorTrStab::make(name, mLogLevel);
	copy->setStandardParametersPU(mNomPower, mNomVolt, mNomFreq, mXpd / mBase_Z, mInertia, mRs, mKd);
	return copy;
}

void SP::Ph1::SynchronGeneratorTrStab::setFundamentalParametersPU(Real nomPower, Real nomVolt, Real nomFreq,
	Real Ll, Real Lmd, Real Llfd, Real inertia, Real D) {
	setBaseParameters(nomPower, nomVolt, nomFreq);
	mSLog->info("\n--- Base Parameters ---"
	"\nnomPower: {:f}"
	"\nnomVolt: {:f}"
	"\nnomFreq: {:f}", mNomPower, mNomVolt, mNomFreq);

	// Input is in per unit but all values are converted to absolute values.
	mParameterType = ParameterType::statorReferred;
	mStateType = StateType::statorReferred;

	mLl = Ll;
	mLmd = Lmd;
	mLd = mLl + mLmd;
	mLlfd = Llfd;
	mLfd = mLlfd + mLmd;
	// M = 2*H where H = inertia
	mInertia = inertia;
	// X'd in absolute values
	mXpd = mNomOmega * (mLd - mLmd*mLmd / mLfd) * mBase_L;
	mLpd = (mLd - mLmd*mLmd / mLfd) * mBase_L;

	//The units of D are per unit power divided by per unit speed deviation.
	// D is transformed to an absolute value to obtain Kd, which will be used in the swing equation
	mKd= D*mNomPower/mNomOmega;

	mSLog->info("\n--- Parameters ---"
				"\nimpedance: {:f}"
				"\ninductance: {:f}"
				"\ninertia: {:f}"
				"\ndamping: {:f}", mXpd, mLpd, mInertia, mKd);
}

void SP::Ph1::SynchronGeneratorTrStab::setStandardParametersSI(Real nomPower, Real nomVolt, Real nomFreq, Int polePairNumber,
	Real Rs, Real Lpd, Real inertiaJ, Real Kd) {
	setBaseParameters(nomPower, nomVolt, nomFreq);
	mSLog->info("\n--- Base Parameters ---"
		"\nnomPower: {:f}"
		"\nnomVolt: {:f}"
		"\nnomFreq: {:f}", mNomPower, mNomVolt, mNomFreq);

	mParameterType = ParameterType::statorReferred;
	mStateType = StateType::statorReferred;

	// M = 2*H where H = inertia
	// H = J * 0.5 * omegaNom^2 / polePairNumber
	mInertia = calcHfromJ(inertiaJ, 2*PI*nomFreq, polePairNumber);
	// X'd in absolute values
	mXpd = mNomOmega * Lpd;
	mLpd = Lpd;

	mSLog->info("\n--- Parameters ---"
				"\nimpedance: {:f}"
			"\ninductance: {:f}"
			"\ninertia: {:f}"
			"\ndamping: {:f}", mXpd, mLpd, mInertia, mKd);
}

void SP::Ph1::SynchronGeneratorTrStab::setStandardParametersPU(Real nomPower, Real nomVolt, Real nomFreq,
	Real Xpd, Real inertia, Real Rs, Real D) {
	setBaseParameters(nomPower, nomVolt, nomFreq);
	mSLog->info("\n--- Base Parameters ---"
	"\nnomPower: {:f}"
	"\nnomVolt: {:f}"
	"\nnomFreq: {:f}", mNomPower, mNomVolt, mNomFreq);

	mSLog->info("\n--- Parameters Per-Unit ---"
				"\n Xpd: {:f} [p.u.]", Xpd);

	// Input is in per unit but all values are converted to absolute values.
	mParameterType = ParameterType::statorReferred;
	mStateType = StateType::statorReferred;

	// M = 2*H where H = inertia
	mInertia = inertia;
	// X'd in absolute values
	mXpd = Xpd * mBase_Z;
	mLpd = Xpd * mBase_L;

	mRs= Rs;
	//The units of D are per unit power divided by per unit speed deviation.
	// D is transformed to an absolute value to obtain Kd, which will be used in the swing equation
	mKd= D*mNomPower/mNomOmega;

	mSLog->info("\n--- Parameters ---"
				"\nXpd: {:f} [Ohm]"
				"\nLpd: {:f} [H]"
				"\nInertia: {:f} [s]"
				"\nDamping: {:f}", mXpd, mLpd, mInertia, mKd);
}

void SP::Ph1::SynchronGeneratorTrStab::setModelFlags(Bool useOmegaRef, Bool convertWithOmegaMech) {
	mUseOmegaRef = useOmegaRef;
	mConvertWithOmegaMech = convertWithOmegaMech;

	mSLog->info("\n--- Model flags ---"
			"\nuseOmegaRef: {:s}"
			"\nconvertWithOmegaMech: {:s}", std::to_string(mUseOmegaRef), std::to_string(mConvertWithOmegaMech));
}

void SP::Ph1::SynchronGeneratorTrStab::setInitialValues(Complex elecPower, Real mechPower) {
	mInitElecPower = elecPower;
	mInitMechPower = mechPower;
}

void SP::Ph1::SynchronGeneratorTrStab::initializeFromNodesAndTerminals(Real frequency) {

	// Initialize omega mech with nominal system frequency
	mOmMech = mNomOmega;

	// Static calculation based on load flow
	mIntfVoltage(0,0) = initialSingleVoltage(0);
	mInitElecPower = (mInitElecPower == Complex(0,0))
		? -terminal(0)->singlePower()
		: mInitElecPower;
	mInitMechPower = (mInitElecPower == Complex(0,0))
		? mInitElecPower.real()
		: mInitMechPower;

	//I_intf is the current which is flowing into the Component, while mInitElecPower is flowing out of it
	mIntfCurrent(0,0) = std::conj( - mInitElecPower / mIntfVoltage(0,0) );

	mImpedance = Complex(mRs, mXpd);

	// Calculate initial emf behind reactance from power flow results
	mEp = mIntfVoltage(0,0) - mImpedance * mIntfCurrent(0,0);

	// The absolute value of Ep is constant, only delta_p changes every step
	mEp_abs = Math::abs(mEp);
	// Delta_p is the angular position of mEp with respect to the synchronously rotating reference
	mDelta_p= Math::phase(mEp);

	// Update electrical power
	// TODO: review for Rs != 0
	mElecActivePower = ( mIntfVoltage(0,0) *  std::conj( -mIntfCurrent(0,0)) ).real();
	mElecReactivePower = ( mIntfVoltage(0,0) *  std::conj( -mIntfCurrent(0,0)) ).imag();

	// Start in steady state so that electrical and mech. power are the same
	// because of the initial condition mOmMech = mNomOmega the damping factor is not considered at the initialisation
	// TODO: review for Rs != 0
	mMechPower = mElecActivePower;

	// Initialize node between X'd and Ep
	mVirtualNodes[0]->setInitialVoltage(mEp);

	// Create sub voltage source for emf
	mSubVoltageSource = SP::Ph1::VoltageSource::make(mName + "_src", mLogLevel);
	mSubVoltageSource->setParameters(mEp);
	mSubVoltageSource->connect({SimNode::GND, mVirtualNodes[0]});
	mSubVoltageSource->setVirtualNodeAt(mVirtualNodes[1], 0);
	mSubVoltageSource->initialize(mFrequencies);
	mSubVoltageSource->initializeFromNodesAndTerminals(frequency);

	// Create sub inductor as Xpd
	mSubInductor = SP::Ph1::Inductor::make(mName + "_ind", mLogLevel);
	mSubInductor->setParameters(mLpd);
	mSubInductor->connect({mVirtualNodes[0], terminal(0)->node()});
	mSubInductor->initialize(mFrequencies);
	mSubInductor->initializeFromNodesAndTerminals(frequency);

	mSLog->info("\n--- Initialize according to powerflow ---"
				"\nTerminal 0 voltage: {:e}<{:e}"
				"\nVoltage behind reactance: {:e}<{:e}"
				"\ninitial electrical power: {:e}+j{:e}"
				"\nactive electrical power: {:e}"
				"\nreactive electrical power: {:e}"
				"\nmechanical power: {:e}"
				"\n--- End of powerflow initialization ---",
				Math::abs(mIntfVoltage(0,0)), Math::phaseDeg(mIntfVoltage(0,0)),
				Math::abs(mEp), Math::phaseDeg(mEp),
				mInitElecPower.real(), mInitElecPower.imag(),
				mElecActivePower, mElecReactivePower, mMechPower);
}

void SP::Ph1::SynchronGeneratorTrStab::step(Real time) {

	// #### Calculations based on values from time step k ####
	// Electrical power at time step k
	// TODO: review for Rs != 0
	mElecActivePower = (mIntfVoltage(0,0) *  std::conj( -mIntfCurrent(0,0)) ).real();
	mElecReactivePower = (mIntfVoltage(0,0) *  std::conj( -mIntfCurrent(0,0)) ).imag();

	// Mechanical speed derivative at time step k
	// convert torque to power with actual rotor angular velocity or nominal omega
	Real dOmMech;
	if (mConvertWithOmegaMech)
		dOmMech = mNomOmega*mNomOmega / (2.*mInertia * mNomPower*mOmMech) * (mMechPower - mElecActivePower - mKd*(mOmMech - mNomOmega));
	else
		dOmMech = mNomOmega / (2.*mInertia * mNomPower) * (mMechPower - mElecActivePower - mKd*(mOmMech - mNomOmega));

	// #### Calculate states for time step k+1 applying semi-implicit Euler ####
	// Mechanical speed at time step k+1 applying Euler forward
	if (mBehaviour == Behaviour::Simulation)
		mOmMech = mOmMech + mTimeStep * dOmMech;

	// Derivative of rotor angle at time step k + 1
	// if reference omega is set, calculate delta with respect to reference
	Real refOmega;
	Real refDelta;
	if (mUseOmegaRef) {
		refOmega = attribute<Real>("w_ref")->get();
		refDelta = attribute<Real>("delta_ref")->get();
	} else {
		refOmega = mNomOmega;
		refDelta = 0;
	}
	Real dDelta_p = mOmMech - refOmega;

	// Rotor angle at time step k + 1 applying Euler backward
	// Update emf - only phase changes
	if (mBehaviour == Behaviour::Simulation) {
		mDelta_p = mDelta_p + mTimeStep * dDelta_p;
		mEp = Complex(mEp_abs * cos(mDelta_p), mEp_abs * sin(mDelta_p));
	}

	mStates << Math::abs(mEp), Math::phaseDeg(mEp), mElecActivePower, mMechPower,
		mDelta_p, mOmMech, dOmMech, dDelta_p, mIntfVoltage(0,0).real(), mIntfVoltage(0,0).imag();
	SPDLOG_LOGGER_DEBUG(mSLog, "\nStates, time {:f}: \n{:s}", time, Logger::matrixToString(mStates));
}

void SP::Ph1::SynchronGeneratorTrStab::mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
	MNAInterface::mnaInitialize(omega, timeStep);
	updateMatrixNodeIndices();

	mSubVoltageSource->mnaInitialize(omega, timeStep, leftVector);
	mSubInductor->mnaInitialize(omega, timeStep, leftVector);
	mTimeStep = timeStep;
	mRightVector = Matrix::Zero(leftVector->get().rows(), 1);
	for (auto task : mSubVoltageSource->mnaTasks()) {
		mMnaTasks.push_back(task);
	}
	for (auto task : mSubInductor->mnaTasks()) {
		mMnaTasks.push_back(task);
	}
	mMnaTasks.push_back(std::make_shared<MnaPreStep>(*this));
	mMnaTasks.push_back(std::make_shared<AddBStep>(*this));
	mMnaTasks.push_back(std::make_shared<MnaPostStep>(*this, leftVector));
}

void SP::Ph1::SynchronGeneratorTrStab::mnaApplySystemMatrixStamp(Matrix& systemMatrix) {
	mSubVoltageSource->mnaApplySystemMatrixStamp(systemMatrix);
	mSubInductor->mnaApplySystemMatrixStamp(systemMatrix);
}

void SP::Ph1::SynchronGeneratorTrStab::mnaApplyRightSideVectorStamp(Matrix& rightVector) {
	mSubVoltageSource->mnaApplyRightSideVectorStamp(rightVector);
	mSubInductor->mnaApplyRightSideVectorStamp(rightVector);
}

void SP::Ph1::SynchronGeneratorTrStab::MnaPreStep::execute(Real time, Int timeStepCount) {
	mGenerator.step(time);
	//change V_ref of subvoltage source
	mGenerator.mSubVoltageSource->attribute<Complex>("V_ref")->set(mGenerator.mEp);
}

void SP::Ph1::SynchronGeneratorTrStab::AddBStep::execute(Real time, Int timeStepCount) {
	mGenerator.mRightVector =
		mGenerator.mSubInductor->attribute<Matrix>("right_vector")->get()
		+ mGenerator.mSubVoltageSource->attribute<Matrix>("right_vector")->get();
}

void SP::Ph1::SynchronGeneratorTrStab::MnaPostStep::execute(Real time, Int timeStepCount) {
	mGenerator.mnaUpdateVoltage(*mLeftVector);
	mGenerator.mnaUpdateCurrent(*mLeftVector);
}

void SP::Ph1::SynchronGeneratorTrStab::mnaUpdateVoltage(const Matrix& leftVector) {
	SPDLOG_LOGGER_DEBUG(mSLog, "Read voltage from {:d}", matrixNodeIndex(0));
	mIntfVoltage(0,0) = Math::complexFromVectorElement(leftVector, matrixNodeIndex(0));
}

void SP::Ph1::SynchronGeneratorTrStab::mnaUpdateCurrent(const Matrix& leftVector) {
	SPDLOG_LOGGER_DEBUG(mSLog, "Read current from {:d}", matrixNodeIndex(0));
	//Current flowing out of component
	mIntfCurrent = mSubInductor->attribute<MatrixComp>("i_intf")->get();
}

void SP::Ph1::SynchronGeneratorTrStab::setReferenceOmega(Attribute<Real>::Ptr refOmegaPtr, Attribute<Real>::Ptr refDeltaPtr) {
	setAttributeRef("w_ref", refOmegaPtr);
	setAttributeRef("delta_ref", refDeltaPtr);
	mUseOmegaRef=true;

	mSLog->info("Use of reference omega.");
}
