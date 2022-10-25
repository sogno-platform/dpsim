/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/EMT/EMT_Ph3_SynchronGeneratorVBR.h>

using namespace CPS;

// !!! TODO: 	Adaptions to use in EMT_Ph3 models phase-to-ground peak variables
// !!! 			with initialization from phase-to-phase RMS variables

EMT::Ph3::SynchronGeneratorVBR::SynchronGeneratorVBR(String uid, String name, Logger::Level logLevel)
	: MNASimPowerComp<Real>(uid, name, true, true, logLevel), Base::SynchronGenerator(mAttributes) {
	mPhaseType = PhaseType::ABC;
	setTerminalNumber(1);
	**mIntfVoltage = Matrix::Zero(3,1);
	**mIntfCurrent = Matrix::Zero(3,1);
}

EMT::Ph3::SynchronGeneratorVBR::SynchronGeneratorVBR(String name, Logger::Level logLevel)
	: SynchronGeneratorVBR(name, name, logLevel) {
}

void EMT::Ph3::SynchronGeneratorVBR::initialize(Matrix frequencies) {
	SimPowerComp<Real>::initialize(frequencies);
}

Matrix& EMT::Ph3::SynchronGeneratorVBR::dqStatorCurrents() { return mDqStatorCurrents; }

Real EMT::Ph3::SynchronGeneratorVBR::electricalTorque() const { return **mElecTorque * mBase_T; }

Real EMT::Ph3::SynchronGeneratorVBR::rotationalSpeed() const { return **mOmMech * mBase_OmMech; }

Real EMT::Ph3::SynchronGeneratorVBR::rotorPosition() const { return mThetaMech; }

Matrix& EMT::Ph3::SynchronGeneratorVBR::statorCurrents() { return mIabc; }

Bool EMT::Ph3::SynchronGeneratorVBR::hasParameterChanged() { return true; }

void EMT::Ph3::SynchronGeneratorVBR::setBaseAndOperationalPerUnitParameters(
	Real nomPower, Real nomVolt, Real nomFreq, Int poleNumber, Real nomFieldCur,
	Real Rs, Real Ld, Real Lq, Real Ld_t, Real Lq_t, Real Ld_s, Real Lq_s,
	Real Ll, Real Td0_t, Real Tq0_t, Real Td0_s, Real Tq0_s, Real inertia) {

	Base::SynchronGenerator::setBaseAndOperationalPerUnitParameters(nomPower, nomVolt, nomFreq, poleNumber, nomFieldCur,
	 																Rs, Ld, Lq, Ld_t, Lq_t, Ld_s, Lq_s,
	 																Ll, Td0_t, Tq0_t, Td0_s, Tq0_s, inertia);

	SPDLOG_LOGGER_INFO(mSLog, "Set base parameters: \n"
				"nomPower: {:e}\nnomVolt: {:e}\nnomFreq: {:e}\n nomFieldCur: {:e}\n",
				nomPower, nomVolt, nomFreq, nomFieldCur);

	SPDLOG_LOGGER_INFO(mSLog, "Set operational parameters in per unit: \n"
			"poleNumber: {:d}\ninertia: {:e}\n"
			"Rs: {:e}\nLd: {:e}\nLq: {:e}\nLl: {:e}\n"
			"Ld_t: {:e}\nLq_t: {:e}\nLd_s: {:e}\nLq_s: {:e}\n"
			"Td0_t: {:e}\nTq0_t: {:e}\nTd0_s: {:e}\nTq0_s: {:e}\n",
			poleNumber, inertia,
			Rs, Ld, Lq, Ll,
			Ld_t, Lq_t, Ld_s, Lq_s,
			Td0_t, Tq0_t, Td0_s, Tq0_s);

	SPDLOG_LOGGER_INFO(mSLog, "Set fundamental parameters in per unit: \n"
			"Rs: {:e}\nLl: {:e}\nLmd: {:e}\nLmq: {:e}\nRfd: {:e}\nLlfd: {:e}\nRkd: {:e}\n"
			"Llkd: {:e}\nRkq1: {:e}\nLlkq1: {:e}\nRkq2: {:e}\nLlkq2: {:e}\n",
			**mRs, **mLl, mLmd, mLmq, mRfd, mLlfd, mRkd, mLlkd, mRkq1, mLlkq1, mRkq2, mLlkq2);
}

void EMT::Ph3::SynchronGeneratorVBR::setBaseAndFundamentalPerUnitParameters(
	Real nomPower, Real nomVolt, Real nomFreq, Int poleNumber, Real nomFieldCur,
	Real Rs, Real Ll, Real Lmd, Real Lmq, Real Rfd, Real Llfd, Real Rkd, Real Llkd,
	Real Rkq1, Real Llkq1, Real Rkq2, Real Llkq2, Real inertia) {

	Base::SynchronGenerator::setBaseAndFundamentalPerUnitParameters(
		nomPower, nomVolt, nomFreq, nomFieldCur,
		poleNumber, Rs, Ll, Lmd, Lmq, Rfd, Llfd, Rkd, Llkd, Rkq1, Llkq1, Rkq2, Llkq2, inertia);

	SPDLOG_LOGGER_INFO(mSLog, "Set base and fundamental parameters in per unit: \n"
				"nomPower: {:e}\nnomVolt: {:e}\nnomFreq: {:e}\npoleNumber: {:d}\nnomFieldCur: {:e}\n"
				"Rs: {:e}\nLl: {:e}\nLmd: {:e}\nLmq: {:e}\nRfd: {:e}\nLlfd: {:e}\nRkd: {:e}\n"
				"Llkd: {:e}\nRkq1: {:e}\nLlkq1: {:e}\nRkq2: {:e}\nLlkq2: {:e}\ninertia: {:e}",
				nomPower, nomVolt, nomFreq, poleNumber, nomFieldCur,
				Rs, Ll, Lmd, Lmq, Rfd, Llfd, Rkd, Llkd, Rkq1, Llkq1, Rkq2, Llkq2, inertia);
}

void EMT::Ph3::SynchronGeneratorVBR::setInitialValues(Real initActivePower, Real initReactivePower,
			Real initTerminalVolt, Real initVoltAngle, Real initMechPower) {

	Base::SynchronGenerator::setInitialValues(initActivePower, initReactivePower,
		initTerminalVolt, initVoltAngle, initMechPower);

	SPDLOG_LOGGER_INFO(mSLog, "Set initial values: \n"
				"initActivePower: {:e}\ninitReactivePower: {:e}\ninitTerminalVolt: {:e}\n"
				"initVoltAngle: {:e} \ninitMechPower: {:e}",
				initActivePower, initReactivePower, initTerminalVolt,
				initVoltAngle, initMechPower);
}

void EMT::Ph3::SynchronGeneratorVBR::initializeFromNodesAndTerminals(Real frequency) {
	if(!mInitialValuesSet) {
		SPDLOG_LOGGER_INFO(mSLog, "--- Initialization from powerflow ---");

		// terminal powers in consumer system -> convert to generator system
		Real activePower = -terminal(0)->singlePower().real();
		Real reactivePower = -terminal(0)->singlePower().imag();

		// 	voltage magnitude in phase-to-phase RMS -> convert to phase-to-ground peak expected by setInitialValues
		Real voltMagnitude = RMS3PH_TO_PEAK1PH*Math::abs(initialSingleVoltage(0));

		this->setInitialValues(activePower, reactivePower, voltMagnitude, Math::phase(initialSingleVoltage(0)), activePower);

		SPDLOG_LOGGER_INFO(mSLog, "\nTerminal 0 voltage: {:s}"
					"\nTerminal 0 power: {:s}"
					"\n--- Initialization from powerflow finished ---",
					Logger::phasorToString(initialSingleVoltage(0)),
					Logger::complexToString(terminal(0)->singlePower()));
		mSLog->flush();
	} else {
		SPDLOG_LOGGER_INFO(mSLog, "Initial values already set, skipping initializeFromNodesAndTerminals.");
		mSLog->flush();
	}
}

void EMT::Ph3::SynchronGeneratorVBR::mnaCompInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
		updateMatrixNodeIndices();

	for (UInt phase1Idx = 0; phase1Idx < 3; ++phase1Idx)
		for (UInt phase2Idx = 0; phase2Idx < 3; ++phase2Idx)
			mVariableSystemMatrixEntries.push_back(std::make_pair<UInt,UInt>(matrixNodeIndex(0, phase1Idx),matrixNodeIndex(0, phase2Idx)));

	SPDLOG_LOGGER_INFO(mSLog, "List of index pairs of varying matrix entries: ");
	for (auto indexPair : mVariableSystemMatrixEntries)
		SPDLOG_LOGGER_INFO(mSLog, "({}, {})", indexPair.first, indexPair.second);


	mSystemOmega = omega;
	mTimeStep = timeStep;

	mResistanceMat = MatrixFixedSize<3, 3>::Zero(3, 3);
	mResistanceMat <<
		**mRs, 0, 0,
		0, **mRs, 0,
		0, 0, **mRs;

	//Dynamic mutual inductances
	mDLmd = 1. / (1. / mLmd + 1. / mLlfd + 1. / mLlkd);

	if (mNumDampingWindings == 2)
		mDLmq = 1. / (1. / mLmq + 1. / mLlkq1 + 1. / mLlkq2);
	else
	{
		mDLmq = 1. / (1. / mLmq + 1. / mLlkq1);
		K1a = Matrix::Zero(2, 1);
		K1 = Matrix::Zero(2, 1);
	}

	mLa = (mDLmq + mDLmd) / 3.;
	mLb = (mDLmd - mDLmq) / 3.;

	// steady state per unit initial value
	initPerUnitStates();


	// Correcting variables
	mThetaMech = mThetaMech + PI / 2;
	**mMechTorque = - **mMechTorque;
	mIq = -mIq;
	mId = -mId;

	// Init stator currents
	mIq = mIsr(3,0);
	mId = mIsr(0,0);
	mI0 = mIsr(6,0);

	// Init stator voltages
	mVq = mVsr(3,0);
	mVd = mVsr(0,0);
	mV0 = mVsr(6,0);

	// Init magnetizing flux linkage
	mPsikq1 = mPsisr(4,0);
	mPsikq2 = mPsisr(5,0);
	mPsikd = mPsisr(2,0);
	mPsifd = mPsisr(1,0);
	mPsimq = mPsisr(3,0);
	mPsimd = mPsisr(0,0);

	/// Init voltage excitation
	mVfd = mVsr(1,0);


	// #### VBR Model Dynamic variables #######################################
	CalculateAuxiliarConstants(mTimeStep*mBase_OmElec);

	if (mNumDampingWindings == 2)
		mPsimq = mDLmq*(mPsikq1 / mLlkq1 + mPsikq2 / mLlkq2 + mIq);
	else
		mPsimq = mDLmq*(mPsikq1 / mLlkq1 + mIq);

	mPsimd = mDLmd*(mPsifd / mLlfd + mPsikd / mLlkd + mId);

	mDqStatorCurrents <<
		mIq,
		mId;

	mPsikq1kq2 <<
		mPsikq1,
		mPsikq2;
	mPsifdkd <<
		mPsifd,
		mPsikd;

	CalculateAuxiliarVariables();
	K1K2 << K1, K2;
	mDVqd = K1K2*mDqStatorCurrents + h_qdr;
	mDVq = mDVqd(0);
	mDVd = mDVqd(1);

	mDVa = inverseParkTransform(mThetaMech, mDVq, mDVd, 0)(0);
	mDVb = inverseParkTransform(mThetaMech, mDVq, mDVd, 0)(1);
	mDVc = inverseParkTransform(mThetaMech, mDVq, mDVd, 0)(2);

	mDVabc <<
		mDVa,
		mDVb,
		mDVc;

	mVa = inverseParkTransform(mThetaMech, mVq, mVd, mV0)(0);
	mVb = inverseParkTransform(mThetaMech, mVq, mVd, mV0)(1);
	mVc = inverseParkTransform(mThetaMech, mVq, mVd, mV0)(2);

	mIa = inverseParkTransform(mThetaMech, mIq, mId, mI0)(0);
	mIb = inverseParkTransform(mThetaMech, mIq, mId, mI0)(1);
	mIc = inverseParkTransform(mThetaMech, mIq, mId, mI0)(2);

	CalculateL();

	SPDLOG_LOGGER_INFO(mSLog, "Initialize right side vector of size {}", leftVector->get().rows());
	SPDLOG_LOGGER_INFO(mSLog, "Component affects right side vector entries {}, {} and {}", matrixNodeIndex(0,0), matrixNodeIndex(0,1), matrixNodeIndex(0,2));
}

void EMT::Ph3::SynchronGeneratorVBR::mnaCompPreStep(Real time, Int timeStepCount) {
	stepInPerUnit();
	mnaCompApplyRightSideVectorStamp(**mRightVector);
}

void EMT::Ph3::SynchronGeneratorVBR::mnaCompApplySystemMatrixStamp(SparseMatrixRow& systemMatrix) {
	if (terminalNotGrounded(0)) {
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 0), matrixNodeIndex(0, 0), mConductanceMat(0, 0));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 0), matrixNodeIndex(0, 1), mConductanceMat(0, 1));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 0), matrixNodeIndex(0, 2), mConductanceMat(0, 2));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 1), matrixNodeIndex(0, 0), mConductanceMat(1, 0));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 1), matrixNodeIndex(0, 1), mConductanceMat(1, 1));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 1), matrixNodeIndex(0, 2), mConductanceMat(1, 2));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 2), matrixNodeIndex(0, 0), mConductanceMat(2, 0));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 2), matrixNodeIndex(0, 1), mConductanceMat(2, 1));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 2), matrixNodeIndex(0, 2), mConductanceMat(2, 2));
		// SPDLOG_LOGGER_INFO(mSLog, "Add {} to {}, {}", conductance, matrixNodeIndex(0,0), matrixNodeIndex(0,0));
		// SPDLOG_LOGGER_INFO(mSLog, "Add {} to {}, {}", conductance, matrixNodeIndex(0,1), matrixNodeIndex(0,1));
		// SPDLOG_LOGGER_INFO(mSLog, "Add {} to {}, {}", conductance, matrixNodeIndex(0,2), matrixNodeIndex(0,2));
	}
}

void EMT::Ph3::SynchronGeneratorVBR::mnaCompAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) {
	// add pre-step dependencies of component itself
	prevStepDependencies.push_back(mIntfCurrent);
	prevStepDependencies.push_back(mIntfVoltage);
	modifiedAttributes.push_back(mRightVector);
}

void EMT::Ph3::SynchronGeneratorVBR::mnaCompApplyRightSideVectorStamp(Matrix& rightVector) {
	if (terminalNotGrounded(0)) {
		Math::setVectorElement(rightVector, matrixNodeIndex(0,0), mISourceEq(0));
		Math::setVectorElement(rightVector, matrixNodeIndex(0,1), mISourceEq(1));
		Math::setVectorElement(rightVector, matrixNodeIndex(0,2), mISourceEq(2));
	}
}

void EMT::Ph3::SynchronGeneratorVBR::stepInPerUnit() {

	// Update of mechanical torque from turbine governor
	if (mHasTurbineGovernor)
		**mMechTorque = -mTurbineGovernor->step(**mOmMech, 1,  mInitElecPower.real() / mNomPower, mTimeStep);

	// Estimate mechanical variables with euler
	**mElecTorque = (mPsimd*mIq - mPsimq*mId);
	**mOmMech = **mOmMech + mTimeStep * (1. / (2. * **mInertia) * (**mElecTorque - **mMechTorque));
	mThetaMech = mThetaMech + mTimeStep * (**mOmMech * mBase_OmMech);

	// Calculate equivalent Resistance and current source
	mVabc <<
		mVa,
		mVb,
		mVc;

	mIabc <<
		mIa,
		mIb,
		mIc;

	mEsh_vbr = (mResistanceMat - (2 / (mTimeStep*mBase_OmElec))*mDInductanceMat)*mIabc + mDVabc - mVabc;

	CalculateL();

	mPsikq1kq2 <<
		mPsikq1,
		mPsikq2;
	mPsifdkd <<
		mPsifd,
		mPsikd;

	CalculateAuxiliarVariables();

	R_eq_vbr = mResistanceMat + (2 / (mTimeStep*mBase_OmElec))*mDInductanceMat + K;
	E_eq_vbr = mEsh_vbr + E_r_vbr;

	MatrixFixedSize<3, 3> R_eq_vbr_mBase_Z = R_eq_vbr*mBase_Z;

	mConductanceMat = R_eq_vbr_mBase_Z.inverse();
	mISourceEq = R_eq_vbr.inverse()*E_eq_vbr*mBase_I;
}

void EMT::Ph3::SynchronGeneratorVBR::mnaCompPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
	if ( terminalNotGrounded(0) ) {
		mVa = Math::realFromVectorElement(*leftVector, matrixNodeIndex(0,0)) / mBase_V;
		mVb = Math::realFromVectorElement(*leftVector, matrixNodeIndex(0,1)) / mBase_V;
		mVc = Math::realFromVectorElement(*leftVector, matrixNodeIndex(0,2)) / mBase_V;
	} else {
		mVa = 0;
		mVb = 0;
		mVc = 0;
	}

	// ################ Update machine stator and rotor variables ############################
	mVabc <<
		mVa,
		mVb,
		mVc;

	mVq = parkTransform(mThetaMech, mVa, mVb, mVc)(0);
	mVd = parkTransform(mThetaMech, mVa, mVb, mVc)(1);
	mV0 = parkTransform(mThetaMech, mVa, mVb, mVc)(2);

	if (mHasExciter){
		// Get exciter output voltage
		// Note: scaled by Rfd/Lmd to transform from exciter pu system
		// to the synchronous generator pu system
		mVfd = (mRfd / mLmd)*mExciter->step(mVd, mVq, mTimeStep);
	}
	mIabc = R_eq_vbr.inverse()*(mVabc - E_eq_vbr);

	mIa = mIabc(0);
	mIb = mIabc(1);
	mIc = mIabc(2);

	mIq_hist = mIq;
	mId_hist = mId;

	mIq = parkTransform(mThetaMech, mIa, mIb, mIc)(0);
	mId = parkTransform(mThetaMech, mIa, mIb, mIc)(1);
	mI0 = parkTransform(mThetaMech, mIa, mIb, mIc)(2);

	// Calculate rotor flux likanges
	if (mNumDampingWindings == 2) {
		mDqStatorCurrents <<
			mIq,
			mId;

		mPsikq1kq2 = E1*mIq + E2*mPsikq1kq2 + E1*mIq_hist;
		mPsifdkd = F1*mId + F2*mPsifdkd + F1*mId_hist + F3*mVfd;

		mPsikq1 = mPsikq1kq2(0);
		mPsikq2 = mPsikq1kq2(1);
		mPsifd = mPsifdkd(0);
		mPsikd = mPsifdkd(1);

	}
	else {

		mDqStatorCurrents <<
			mIq,
			mId;

		mPsikq1 = E1_1d*mIq + E2_1d*mPsikq1 + E1_1d*mIq_hist;
		mPsifdkd = F1*mId + F2*mPsifdkd + F1*mId_hist + F3 * mVfd;

		mPsifd = mPsifdkd(0);
		mPsikd = mPsifdkd(1);
	}


	// Calculate dynamic flux likages
	if (mNumDampingWindings == 2) {
		mPsimq = mDLmq*(mPsikq1 / mLlkq1 + mPsikq2 / mLlkq2 + mIq);
	}
	else {
		mPsimq = mDLmq*(mPsikq1 / mLlkq1 + mIq);
	}

	mPsimd = mDLmd*(mPsifd / mLlfd + mPsikd / mLlkd + mId);

	K1K2 << K1, K2;
	mDVqd = K1K2*mDqStatorCurrents + h_qdr;
	mDVq = mDVqd(0);
	mDVd = mDVqd(1);

	mDVa = inverseParkTransform(mThetaMech, mDVq, mDVd, 0)(0);
	mDVb = inverseParkTransform(mThetaMech, mDVq, mDVd, 0)(1);
	mDVc = inverseParkTransform(mThetaMech, mDVq, mDVd, 0)(2);
	mDVabc <<
		mDVa,
		mDVb,
		mDVc;

	**mIntfVoltage = mVabc*mBase_V;
	**mIntfCurrent = mIabc*mBase_I;
}

void EMT::Ph3::SynchronGeneratorVBR::mnaCompAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector) {
	// add post-step dependencies of component itself
	attributeDependencies.push_back(leftVector);
	modifiedAttributes.push_back(mIntfVoltage);
	modifiedAttributes.push_back(mIntfCurrent);
}

void EMT::Ph3::SynchronGeneratorVBR::CalculateL() {
	mDInductanceMat <<
		**mLl + mLa - mLb*cos(2 * mThetaMech), -mLa / 2 - mLb*cos(2 * mThetaMech - 2 * PI / 3), -mLa / 2 - mLb*cos(2 * mThetaMech + 2 * PI / 3),
		-mLa / 2 - mLb*cos(2 * mThetaMech - 2 * PI / 3), **mLl + mLa - mLb*cos(2 * mThetaMech - 4 * PI / 3), -mLa / 2 - mLb*cos(2 * mThetaMech),
		-mLa / 2 - mLb*cos(2 * mThetaMech + 2 * PI / 3), -mLa / 2 - mLb*cos(2 * mThetaMech), **mLl + mLa - mLb*cos(2 * mThetaMech + 4 * PI / 3);
}

void EMT::Ph3::SynchronGeneratorVBR::CalculateAuxiliarConstants(Real dt) {

	b11 = (mRkq1 / mLlkq1)*(mDLmq / mLlkq1 - 1);
	b13 = mRkq1*mDLmq / mLlkq1;
	b31 = (mRfd / mLlfd)*(mDLmd / mLlfd - 1);
	b32 = mRfd*mDLmd / (mLlfd*mLlkd);
	b33 = mRfd*mDLmd / mLlfd;
	b41 = mRkd*mDLmd / (mLlfd*mLlkd);
	b42 = (mRkd / mLlkd)*(mDLmd / mLlkd - 1);
	b43 = mRkd*mDLmd / mLlkd;

	c23 = mDLmd*mRfd / (mLlfd*mLlfd)*(mDLmd / mLlfd - 1) + mDLmd*mDLmd*mRkd / (mLlkd*mLlkd*mLlfd);
	c24 = mDLmd*mRkd / (mLlkd*mLlkd)*(mDLmd / mLlkd - 1) + mDLmd*mDLmd*mRfd / (mLlfd*mLlfd*mLlkd);
	c25 = (mRfd / (mLlfd*mLlfd) + mRkd / (mLlkd*mLlkd))*mDLmd*mDLmd;
	c26 = mDLmd / mLlfd;

	if (mNumDampingWindings == 2) {
		b12 = mRkq1*mDLmq / (mLlkq1*mLlkq2);
		b21 = mRkq2*mDLmq / (mLlkq1*mLlkq2);
		b22 = (mRkq2 / mLlkq2)*(mDLmq / mLlkq2 - 1);
		b23 = mRkq2*mDLmq / mLlkq2;
		c11 = mDLmq*mRkq1 / (mLlkq1*mLlkq1)*(mDLmq / mLlkq1 - 1) + mDLmq*mDLmq*mRkq2 / (mLlkq2*mLlkq2*mLlkq1);
		c12 = mDLmq*mRkq2 / (mLlkq2*mLlkq2)*(mDLmq / mLlkq2 - 1) + mDLmq*mDLmq*mRkq1 / (mLlkq1*mLlkq1*mLlkq2);
		c15 = (mRkq1 / (mLlkq1*mLlkq1) + mRkq2 / (mLlkq2*mLlkq2))*mDLmq*mDLmq;

		Ea <<
			2 - dt*b11, -dt*b12,
			-dt*b21, 2 - dt*b22;
		E1b <<
			dt*b13,
			dt*b23;

		Matrix Ea_inv = Ea.inverse();

		E1 = Ea_inv * E1b;

		E2b <<
			2 + dt*b11, dt*b12,
			dt*b21, 2 + dt*b22;
		E2 = Ea_inv * E2b;
	}
	else {
		c11 = mDLmq*mRkq1 / (mLlkq1*mLlkq1)*(mDLmq / mLlkq1 - 1);
		c15 = (mRkq1 / (mLlkq1*mLlkq1))*mDLmq*mDLmq;

		E1_1d = (1 / (2 - dt*b11))*dt*b13;
		E2_1d = (1 / (2 - dt*b11))*(2 + dt*b11);
	}

	Fa <<
		2 - dt*b31, -dt*b32,
		-dt*b41, 2 - dt*b42;

	F1b <<
		dt*b33,
		dt*b43;
	F1 = Fa.inverse() * F1b;

	F2b <<
		2 + dt*b31, dt*b32,
		dt*b41, 2 + dt*b42;

	F2 = Fa.inverse() * F2b;

	F3b <<
		2 * dt,
		0;
	F3 = Fa.inverse() * F3b;

	C26 <<
		0,
		c26;
}

void EMT::Ph3::SynchronGeneratorVBR::CalculateAuxiliarVariables() {

	if (mNumDampingWindings == 2) {
		c21_omega = - **mOmMech * mDLmq / mLlkq1;
		c22_omega = - **mOmMech * mDLmq / mLlkq2;
		c13_omega = **mOmMech * mDLmd / mLlfd;
		c14_omega = **mOmMech * mDLmd / mLlkd;

		K1a <<
			c11, c12,
			c21_omega, c22_omega;
		K1b <<
			c15,
			0;
		K1 = K1a*E1 + K1b;
	}
	else {
		c21_omega = - **mOmMech * mDLmq / mLlkq1;
		c13_omega = **mOmMech * mDLmd / mLlfd;
		c14_omega = **mOmMech * mDLmd / mLlkd;

		K1a <<
			c11,
			c21_omega;
		K1b <<
			c15,
			0;
		K1 = K1a*E1_1d + K1b;
	}

	K2a <<
		c13_omega, c14_omega,
		c23, c24;
	K2b <<
		0,
		c25;
	K2 = K2a*F1 + K2b;

	K <<
		K1, K2, Matrix::Zero(2, 1),
		0, 0, 0;

	mKrs_teta <<
		2. / 3. * cos(mThetaMech), 2. / 3. * cos(mThetaMech - 2. * M_PI / 3.), 2. / 3. * cos(mThetaMech + 2. * M_PI / 3.),
		2. / 3. * sin(mThetaMech), 2. / 3. * sin(mThetaMech - 2. * M_PI / 3.), 2. / 3. * sin(mThetaMech + 2. * M_PI / 3.),
		1. / 3., 1. / 3., 1. / 3.;

	mKrs_teta_inv <<
		cos(mThetaMech), sin(mThetaMech), 1.,
		cos(mThetaMech - 2. * M_PI / 3.), sin(mThetaMech - 2. * M_PI / 3.), 1,
		cos(mThetaMech + 2. * M_PI / 3.), sin(mThetaMech + 2. * M_PI / 3.), 1.;

	K = mKrs_teta_inv*K*mKrs_teta;

	if (mNumDampingWindings == 2)
		h_qdr = K1a*E2*mPsikq1kq2 + K1a*E1*mIq + K2a*F2*mPsifdkd + K2a*F1*mId + (K2a*F3 + C26) * mVfd;
	else
		h_qdr = K1a*E2_1d*mPsikq1 + K1a*E1_1d*mIq + K2a*F2*mPsifdkd + K2a*F1*mId + (K2a*F3 + C26) * mVfd;

	H_qdr << h_qdr,
		0;

	E_r_vbr = mKrs_teta_inv*H_qdr;
}

Matrix EMT::Ph3::SynchronGeneratorVBR::parkTransform(Real theta, Real a, Real b, Real c) {

	Matrix dq0vector(3, 1);

	Real q, d, zero;

	q = 2. / 3. * cos(theta)*a + 2. / 3. * cos(theta - 2. * M_PI / 3.)*b + 2. / 3. * cos(theta + 2. * M_PI / 3.)*c;
	d = 2. / 3. * sin(theta)*a + 2. / 3. * sin(theta - 2. * M_PI / 3.)*b + 2. / 3. * sin(theta + 2. * M_PI / 3.)*c;
	zero = 1. / 3.*a + 1. / 3.*b + 1. / 3.*c;

	dq0vector << q,
		d,
		zero;

	return dq0vector;
}

Matrix EMT::Ph3::SynchronGeneratorVBR::inverseParkTransform(Real theta, Real q, Real d, Real zero) {

	Matrix abcVector(3, 1);

	Real a, b, c;

	a = cos(theta)*q + sin(theta)*d + 1.*zero;
	b = cos(theta - 2. * M_PI / 3.)*q + sin(theta - 2. * M_PI / 3.)*d + 1.*zero;
	c = cos(theta + 2. * M_PI / 3.)*q + sin(theta + 2. * M_PI / 3.)*d + 1.*zero;

	abcVector << a,
		b,
		c;

	return abcVector;
}
