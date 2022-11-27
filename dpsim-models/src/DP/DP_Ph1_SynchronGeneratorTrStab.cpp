/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/DP/DP_Ph1_SynchronGeneratorTrStab.h>
using namespace CPS;

DP::Ph1::SynchronGeneratorTrStab::SynchronGeneratorTrStab(String uid, String name, Logger::Level logLevel)
	: Base::SynchronGenerator(mAttributes), SimPowerComp<Complex>(uid, name, logLevel),
	mEp(Attribute<Complex>::create("Ep", mAttributes)),
	mEp_abs(Attribute<Real>::create("Ep_mag", mAttributes)),
	mEp_phase(Attribute<Real>::create("Ep_phase", mAttributes)),
	mDelta_p(Attribute<Real>::create("delta_r", mAttributes)),
	mRefOmega(Attribute<Real>::createDynamic("w_ref", mAttributes)),
	mRefDelta(Attribute<Real>::createDynamic("delta_ref", mAttributes)) {
	setVirtualNodeNumber(2);
	setTerminalNumber(1);
	**mIntfVoltage = MatrixComp::Zero(1, 1);
	**mIntfCurrent = MatrixComp::Zero(1, 1);

	mStates = Matrix::Zero(10,1);
}

SimPowerComp<Complex>::Ptr DP::Ph1::SynchronGeneratorTrStab::clone(String name) {
	auto copy = SynchronGeneratorTrStab::make(name, mLogLevel);
	copy->setStandardParametersPU(mNomPower, mNomVolt, mNomFreq, mXpd / mBase_Z, **mInertia, **mRs, mKd);
	return copy;
}

void DP::Ph1::SynchronGeneratorTrStab::setFundamentalParametersPU(Real nomPower, Real nomVolt, Real nomFreq,
	Real Ll, Real Lmd, Real Llfd, Real inertia, Real D) {
	setBaseParameters(nomPower, nomVolt, nomFreq);
	mSLog->info("\n--- Base Parameters ---"
	"\nnomPower: {:f}"
	"\nnomVolt: {:f}"
	"\nnomFreq: {:f}", mNomPower, mNomVolt, mNomFreq);

	// Input is in per unit but all values are converted to absolute values.
	mParameterType = ParameterType::statorReferred;
	mStateType = StateType::statorReferred;

	**mLl = Ll;
	mLmd = Lmd;
	**mLd = **mLl + mLmd;
	mLlfd = Llfd;
	mLfd = mLlfd + mLmd;
	// M = 2*H where H = inertia
	**mInertia = inertia;
	// X'd in absolute values
	mXpd = mNomOmega * (**mLd - mLmd*mLmd / mLfd) * mBase_L;
	mLpd = (**mLd - mLmd*mLmd / mLfd) * mBase_L;

	//The units of D are per unit power divided by per unit speed deviation.
	// D is transformed to an absolute value to obtain Kd, which will be used in the swing equation
	mKd= D*mNomPower/mNomOmega;

	mSLog->info("\n--- Parameters ---"
				"\nimpedance: {:f}"
				"\ninductance: {:f}"
				"\ninertia: {:f}"
				"\ndamping: {:f}", mXpd, mLpd, **mInertia, mKd);
}

void DP::Ph1::SynchronGeneratorTrStab::setStandardParametersSI(Real nomPower, Real nomVolt, Real nomFreq, Int polePairNumber,
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
	**mInertia = calcHfromJ(inertiaJ, 2*PI*nomFreq, polePairNumber);
	// X'd in absolute values
	mXpd = mNomOmega * Lpd;
	mLpd = Lpd;

	mSLog->info("\n--- Parameters ---"
			"\nimpedance: {:f}"
			"\ninductance: {:f}"
			"\ninertia: {:f}"
			"\ndamping: {:f}", mXpd, mLpd, **mInertia, mKd);
}

void DP::Ph1::SynchronGeneratorTrStab::setStandardParametersPU(Real nomPower, Real nomVolt, Real nomFreq,
	Real Xpd, Real inertia, Real Rs, Real D) {
	setBaseParameters(nomPower, nomVolt, nomFreq);
	mSLog->info("\n--- Base Parameters ---"
	"\nnomPower: {:f}"
	"\nnomVolt: {:f}"
	"\nnomFreq: {:f}", mNomPower, mNomVolt, mNomFreq);

	// Input is in per unit but all values are converted to absolute values.
	mParameterType = ParameterType::statorReferred;
	mStateType = StateType::statorReferred;

	// M = 2*H where H = inertia
	**mInertia = inertia;
	// X'd in absolute values
	mXpd = Xpd * mBase_Z;
	mLpd = Xpd * mBase_L;

	**mRs= Rs;
	//The units of D are per unit power divided by per unit speed deviation.
	// D is transformed to an absolute value to obtain Kd, which will be used in the swing equation
	mKd= D*mNomPower/mNomOmega;

	mSLog->info("\n--- Parameters ---"
			"\nimpedance: {:f}"
			"\ninductance: {:f}"
			"\ninertia: {:f}"
			"\ndamping: {:f}", mXpd, mLpd, **mInertia, mKd);
}

void DP::Ph1::SynchronGeneratorTrStab::setModelFlags(Bool convertWithOmegaMech) {
	mConvertWithOmegaMech = convertWithOmegaMech;

	mSLog->info("\n--- Model flags ---"
			"\nconvertWithOmegaMech: {:s}", std::to_string(mConvertWithOmegaMech));
}

void DP::Ph1::SynchronGeneratorTrStab::setInitialValues(Complex elecPower, Real mechPower) {
	mInitElecPower = elecPower;
	mInitMechPower = mechPower;
}

void DP::Ph1::SynchronGeneratorTrStab::initializeFromNodesAndTerminals(Real frequency) {

	// Initialize omega mech with nominal system frequency
	**mOmMech = mNomOmega;

	// Static calculation based on load flow
	(**mIntfVoltage)(0,0) = initialSingleVoltage(0);
	mInitElecPower = (mInitElecPower == Complex(0,0))
		? -terminal(0)->singlePower()
		: mInitElecPower;
	mInitMechPower = (mInitElecPower == Complex(0,0))
		? mInitElecPower.real()
		: mInitMechPower;

	//I_intf is the current which is flowing into the Component, while mInitElecPower is flowing out of it
	(**mIntfCurrent)(0,0) = std::conj( - mInitElecPower / (**mIntfVoltage)(0,0) );

	mImpedance = Complex(**mRs, mXpd);

	// Calculate initial emf behind reactance from power flow results
	**mEp = (**mIntfVoltage)(0,0) - mImpedance * (**mIntfCurrent)(0,0);

	// The absolute value of Ep is constant, only delta_p changes every step
	**mEp_abs = Math::abs(**mEp);
	// Delta_p is the angular position of mEp with respect to the synchronously rotating reference
	**mDelta_p= Math::phase(**mEp);

	// Update active electrical power that is compared with the mechanical power
	**mElecActivePower = ( (**mIntfVoltage)(0,0) *  std::conj( -(**mIntfCurrent)(0,0)) ).real();

	// Start in steady state so that electrical and mech. power are the same
	// because of the initial condition mOmMech = mNomOmega the damping factor is not considered at the initialisation
	**mMechPower = **mElecActivePower;

	// Initialize node between X'd and Ep
	mVirtualNodes[0]->setInitialVoltage(**mEp);

	// Create sub voltage source for emf
	mSubVoltageSource = DP::Ph1::VoltageSource::make(**mName + "_src", mLogLevel);
	mSubVoltageSource->setParameters(**mEp);
	mSubVoltageSource->connect({SimNode::GND, mVirtualNodes[0]});
	mSubVoltageSource->setVirtualNodeAt(mVirtualNodes[1], 0);
	mSubVoltageSource->initialize(mFrequencies);
	mSubVoltageSource->initializeFromNodesAndTerminals(frequency);
	mSubComponents.push_back(mSubVoltageSource);

	// Create sub inductor as Xpd
	mSubInductor = DP::Ph1::Inductor::make(**mName + "_ind", mLogLevel);
	mSubInductor->setParameters(mLpd);
	mSubInductor->connect({mVirtualNodes[0], terminal(0)->node()});
	mSubInductor->initialize(mFrequencies);
	mSubInductor->initializeFromNodesAndTerminals(frequency);
	mSubComponents.push_back(mSubInductor);

	mSLog->info("\n--- Initialize according to powerflow ---"
				"\nTerminal 0 voltage: {:e}<{:e}"
				"\nVoltage behind reactance: {:e}<{:e}"
				"\ninitial electrical power: {:e}+j{:e}"
				"\nactive electrical power: {:e}"
				"\nmechanical power: {:e}"
				"\n--- End of powerflow initialization ---",
				Math::abs((**mIntfVoltage)(0,0)), Math::phaseDeg((**mIntfVoltage)(0,0)),
				Math::abs(**mEp), Math::phaseDeg(**mEp),
				mInitElecPower.real(), mInitElecPower.imag(),
				**mElecActivePower, **mMechPower);
}

void DP::Ph1::SynchronGeneratorTrStab::step(Real time) {

	// #### Calculations based on values from time step k ####
	// Electrical power at time step k
	**mElecActivePower = ((**mIntfVoltage)(0,0) *  std::conj( -(**mIntfCurrent)(0,0)) ).real();

	// Mechanical speed derivative at time step k
	// convert torque to power with actual rotor angular velocity or nominal omega
	Real dOmMech;
	if (mConvertWithOmegaMech)
		dOmMech = mNomOmega*mNomOmega / (2.* **mInertia * mNomPower* **mOmMech) * (**mMechPower - **mElecActivePower - mKd*(**mOmMech - mNomOmega));
	else
		dOmMech = mNomOmega / (2.* **mInertia * mNomPower) * (**mMechPower - **mElecActivePower - mKd*(**mOmMech - mNomOmega));

	// #### Calculate states for time step k+1 applying semi-implicit Euler ####
	// Mechanical speed at time step k+1 applying Euler forward
	if (mBehaviour == Behaviour::MNASimulation)
		**mOmMech = **mOmMech + mTimeStep * dOmMech;

	// Derivative of rotor angle at time step k + 1
	// if reference omega is set, calculate delta with respect to reference
	Real dDelta_p = **mOmMech - (mUseOmegaRef ? **mRefOmega : mNomOmega);

	// Rotor angle at time step k + 1 applying Euler backward
	// Update emf - only phase changes
	if (mBehaviour == Behaviour::MNASimulation) {
		**mDelta_p = **mDelta_p + mTimeStep * dDelta_p;
		**mEp = Complex(**mEp_abs * cos(**mDelta_p), **mEp_abs * sin(**mDelta_p));
	}

	mStates << Math::abs(**mEp), Math::phaseDeg(**mEp), **mElecActivePower, **mMechPower,
		**mDelta_p, **mOmMech, dOmMech, dDelta_p, (**mIntfVoltage)(0,0).real(), (**mIntfVoltage)(0,0).imag();
	SPDLOG_LOGGER_DEBUG(mSLog, "\nStates, time {:f}: \n{:s}", time, Logger::matrixToString(mStates));
}

void DP::Ph1::SynchronGeneratorTrStab::mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
	MNAInterface::mnaInitialize(omega, timeStep);
	updateMatrixNodeIndices();

	for (auto subComp : mSubComponents) {
		if (auto mnasubcomp = std::dynamic_pointer_cast<MNAInterface>(subComp))
			mnasubcomp->mnaInitialize(omega, timeStep, leftVector);
	}

	mTimeStep = timeStep;
	**mRightVector = Matrix::Zero(leftVector->get().rows(), 1);
	mMnaTasks.push_back(std::make_shared<MnaPreStep>(*this));
	mMnaTasks.push_back(std::make_shared<AddBStep>(*this));
	mMnaTasks.push_back(std::make_shared<MnaPostStep>(*this, leftVector));
}

void DP::Ph1::SynchronGeneratorTrStab::mnaApplySystemMatrixStamp(Matrix& systemMatrix) {
	for (auto subComp : mSubComponents) {
		if (auto mnasubcomp = std::dynamic_pointer_cast<MNAInterface>(subComp))
			mnasubcomp->mnaApplySystemMatrixStamp(systemMatrix);
	}
}

void DP::Ph1::SynchronGeneratorTrStab::mnaApplyRightSideVectorStamp(Matrix& rightVector) {
	for (auto subComp : mSubComponents) {
		if (auto mnasubcomp = std::dynamic_pointer_cast<MNAInterface>(subComp))
			mnasubcomp->mnaApplyRightSideVectorStamp(rightVector);
	}
}

void DP::Ph1::SynchronGeneratorTrStab::mnaAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) {
	prevStepDependencies.push_back(mIntfVoltage);
	for (auto subComp : mSubComponents) {
		if (auto mnasubcomp = std::dynamic_pointer_cast<MNAInterface>(subComp))
			mnasubcomp->mnaAddPreStepDependencies(prevStepDependencies, attributeDependencies, modifiedAttributes);
	}
}

void DP::Ph1::SynchronGeneratorTrStab::mnaAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector) {
	attributeDependencies.push_back(leftVector);
	modifiedAttributes.push_back(mIntfVoltage);
	for (auto subComp : mSubComponents) {
		if (auto mnasubcomp = std::dynamic_pointer_cast<MNAInterface>(subComp))
			mnasubcomp->mnaAddPostStepDependencies(prevStepDependencies, attributeDependencies, modifiedAttributes, leftVector);
	}
}

void DP::Ph1::SynchronGeneratorTrStab::MnaPreStep::execute(Real time, Int timeStepCount) {
	for (auto subComp : mGenerator.mSubComponents) {
		if (auto mnasubcomp = std::dynamic_pointer_cast<MNAInterface>(subComp))
			mnasubcomp->mnaPreStep(time, timeStepCount);
	}

	mGenerator.step(time);
	//change V_ref of subvoltage source
	mGenerator.mSubVoltageSource->mVoltageRef->set(**mGenerator.mEp);
}

void DP::Ph1::SynchronGeneratorTrStab::AddBStep::execute(Real time, Int timeStepCount) {
	**mGenerator.mRightVector =
		mGenerator.mSubInductor->mRightVector->get()
		+ mGenerator.mSubVoltageSource->mRightVector->get();
}

void DP::Ph1::SynchronGeneratorTrStab::MnaPostStep::execute(Real time, Int timeStepCount) {
	for (auto subComp : mGenerator.mSubComponents) {
		if (auto mnasubcomp = std::dynamic_pointer_cast<MNAInterface>(subComp))
			mnasubcomp->mnaPostStep(time, timeStepCount, mLeftVector);
	}

	mGenerator.mnaUpdateVoltage(**mLeftVector);
	mGenerator.mnaUpdateCurrent(**mLeftVector);
}

void DP::Ph1::SynchronGeneratorTrStab::mnaUpdateVoltage(const Matrix& leftVector) {
	SPDLOG_LOGGER_DEBUG(mSLog, "Read voltage from {:d}", matrixNodeIndex(0));
	(**mIntfVoltage)(0,0) = Math::complexFromVectorElement(leftVector, matrixNodeIndex(0));
}

void DP::Ph1::SynchronGeneratorTrStab::mnaUpdateCurrent(const Matrix& leftVector) {
	SPDLOG_LOGGER_DEBUG(mSLog, "Read current from {:d}", matrixNodeIndex(0));
	//Current flowing out of component
	**mIntfCurrent = mSubInductor->mIntfCurrent->get();
}

void DP::Ph1::SynchronGeneratorTrStab::setReferenceOmega(Attribute<Real>::Ptr refOmegaPtr, Attribute<Real>::Ptr refDeltaPtr) {
	mRefOmega->setReference(refOmegaPtr);
	mRefDelta->setReference(refDeltaPtr);
	mUseOmegaRef=true;

	mSLog->info("Use of reference omega.");
}
