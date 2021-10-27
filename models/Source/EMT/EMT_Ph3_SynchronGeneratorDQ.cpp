/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <cps/EMT/EMT_Ph3_SynchronGeneratorDQ.h>

using namespace CPS;

// !!! TODO: 	Adaptions to use in EMT_Ph3 models phase-to-ground peak variables
// !!! 			with initialization from phase-to-phase RMS variables

EMT::Ph3::SynchronGeneratorDQ::SynchronGeneratorDQ(String uid, String name, Logger::Level logLevel)
	: SimPowerComp<Real>(uid, name, logLevel) {
	mPhaseType = PhaseType::ABC;
	setTerminalNumber(1);
	mIntfVoltage = Matrix::Zero(3,1);
	mIntfCurrent = Matrix::Zero(3,1);

	addAttribute<Real>("Rs", &mRs, Flags::read | Flags::write);
	addAttribute<Real>("Ll", &mLl, Flags::read | Flags::write);
	addAttribute<Real>("Ld", &mLd, Flags::read | Flags::write);
	addAttribute<Real>("Lq", &mLq, Flags::read | Flags::write);

	addAttribute<Real>("Ld_t", &mLd_t, Flags::read | Flags::write);
	addAttribute<Real>("Ld_s", &mLd_s, Flags::read | Flags::write);
	addAttribute<Real>("Lq_t", &mLq_t, Flags::read | Flags::write);
	addAttribute<Real>("Lq_s", &mLq_s, Flags::read | Flags::write);

	addAttribute<Real>("Td0_t", &mTd0_t, Flags::read | Flags::write);
	addAttribute<Real>("Td0_s", &mTd0_s, Flags::read | Flags::write);
	addAttribute<Real>("Tq0_t", &mTq0_t, Flags::read | Flags::write);
	addAttribute<Real>("Tq0_s", &mTq0_s, Flags::read | Flags::write);

	addAttribute<Real>("w_r", &mOmMech, Flags::read);
	addAttribute<Real>("delta_r", &mDelta, Flags::read);
}

EMT::Ph3::SynchronGeneratorDQ::SynchronGeneratorDQ(String name, Logger::Level logLevel)
	: SynchronGeneratorDQ(name, name, logLevel) {
}

EMT::Ph3::SynchronGeneratorDQ::~SynchronGeneratorDQ() {
}

void EMT::Ph3::SynchronGeneratorDQ::setParametersFundamentalPerUnit(
	Real nomPower, Real nomVolt, Real nomFreq, Int poleNumber, Real nomFieldCur,
	Real Rs, Real Ll, Real Lmd, Real Lmq, Real Rfd, Real Llfd, Real Rkd, Real Llkd,
	Real Rkq1, Real Llkq1, Real Rkq2, Real Llkq2, Real inertia,
	Real initActivePower, Real initReactivePower, Real initTerminalVolt, Real initVoltAngle,
	Real initFieldVoltage, Real initMechPower) {

	Base::SynchronGenerator::setBaseAndFundamentalPerUnitParameters(
		nomPower, nomVolt, nomFreq, nomFieldCur,
		poleNumber, Rs, Ll, Lmd, Lmq, Rfd, Llfd, Rkd, Llkd, Rkq1, Llkq1, Rkq2, Llkq2, inertia);

	mSLog->info("Set base and fundamental parameters in per unit: \n"
				"nomPower: {:e}\nnomVolt: {:e}\nnomFreq: {:e}\npoleNumber: {:d}\nnomFieldCur: {:e}\n"
				"Rs: {:e}\nLl: {:e}\nLmd: {:e}\nLmq: {:e}\nRfd: {:e}\nLlfd: {:e}\nRkd: {:e}\n"
				"Llkd: {:e}\nRkq1: {:e}\nLlkq1: {:e}\nRkq2: {:e}\nLlkq2: {:e}\ninertia: {:e}",
				nomPower, nomVolt, nomFreq, poleNumber, nomFieldCur,
				Rs, Ll, Lmd, Lmq, Rfd, Llfd, Rkd, Llkd, Rkq1, Llkq1, Rkq2, Llkq2, inertia);

	Base::SynchronGenerator::setInitialValues(initActivePower, initReactivePower,
		initTerminalVolt, initVoltAngle, initFieldVoltage, initMechPower);

	mSLog->info("Set initial values: \n"
				"initActivePower: {:e}\ninitReactivePower: {:e}\ninitTerminalVolt: {:e}\n"
				"initVoltAngle: {:e}\ninitFieldVoltage: {:e}\ninitMechPower: {:e}",
				initActivePower, initReactivePower, initTerminalVolt,
				initVoltAngle, initFieldVoltage, initMechPower);
}

void EMT::Ph3::SynchronGeneratorDQ::setParametersOperationalPerUnit(
	Real nomPower, Real nomVolt, Real nomFreq, Int poleNumber, Real nomFieldCur,
	Real Rs, Real Ld, Real Lq, Real Ld_t, Real Lq_t, Real Ld_s, Real Lq_s,
	Real Ll, Real Td0_t, Real Tq0_t, Real Td0_s, Real Tq0_s,
	Real inertia, Real initActivePower, Real initReactivePower, Real initTerminalVolt, Real initVoltAngle,
	Real initFieldVoltage, Real initMechPower) {

	setBaseParameters(nomPower, nomVolt, nomFreq, nomFieldCur);
	mSLog->info("Set base parameters: \n"
				"nomPower: {:e}\nnomVolt: {:e}\nnomFreq: {:e}\n nomFieldCur: {:e}\n",
				nomPower, nomVolt, nomFreq, nomFieldCur);

	setOperationalPerUnitParameters(poleNumber, inertia,
									Rs, Ld, Lq, Ll,
									Ld_t, Lq_t, Ld_s, Lq_s,
									Td0_t, Tq0_t, Td0_s, Tq0_s);
	mSLog->info("Set operational parameters in per unit: \n"
			"poleNumber: {:d}\ninertia: {:e}\n"
			"Rs: {:e}\nLd: {:e}\nLq: {:e}\nLl: {:e}\n"
			"Ld_t: {:e}\nLq_t: {:e}\nLd_s: {:e}\nLq_s: {:e}\n"
			"Td0_t: {:e}\nTq0_t: {:e}\nTd0_s: {:e}\nTq0_s: {:e}\n",
			poleNumber, inertia,
			Rs, Ld, Lq, Ll,
			Ld_t, Lq_t, Ld_s, Lq_s,
			Td0_t, Tq0_t, Td0_s, Tq0_s);

	Base::SynchronGenerator::calculateFundamentalFromOperationalParameters();
	mSLog->info("Set fundamental parameters in per unit: \n"
			"Rs: {:e}\nLl: {:e}\nLmd: {:e}\nLmq: {:e}\nRfd: {:e}\nLlfd: {:e}\nRkd: {:e}\n"
			"Llkd: {:e}\nRkq1: {:e}\nLlkq1: {:e}\nRkq2: {:e}\nLlkq2: {:e}\n",
			mRs, mLl, mLmd, mLmq, mRfd, mLlfd, mRkd, mLlkd, mRkq1, mLlkq1, mRkq2, mLlkq2);

	Base::SynchronGenerator::setInitialValues(initActivePower, initReactivePower,
	initTerminalVolt, initVoltAngle, initFieldVoltage, initMechPower);
	mSLog->info("Set initial values: \n"
				"initActivePower: {:e}\ninitReactivePower: {:e}\ninitTerminalVolt: {:e}\n"
				"initVoltAngle: {:e}\ninitFieldVoltage: {:e}\ninitMechPower: {:e}",
				initActivePower, initReactivePower, initTerminalVolt,
				initVoltAngle, initFieldVoltage, initMechPower);
}

void EMT::Ph3::SynchronGeneratorDQ::applyParametersOperationalPerUnit() {

	mSLog->info("Apply operational parameters in per unit: \n"
			"poleNumber: {:d}\ninertia: {:e}\n"
			"Rs: {:e}\nLd: {:e}\nLq: {:e}\nLl: {:e}\n"
			"Ld_t: {:e}\nLq_t: {:e}\nLd_s: {:e}\nLq_s: {:e}\n"
			"Td0_t: {:e}\nTq0_t: {:e}\nTd0_s: {:e}\nTq0_s: {:e}\n",
			mPoleNumber, mInertia,
			mRs, mLd, mLq, mLl,
			mLd_t, mLq_t, mLd_s, mLq_s,
			mTd0_t, mTq0_t, mTd0_s, mTq0_s);

	Base::SynchronGenerator::calculateFundamentalFromOperationalParameters();
	mSLog->info("Updated fundamental parameters in per unit: \n"
			"Rs: {:e}\nLl: {:e}\nLmd: {:e}\nLmq: {:e}\nRfd: {:e}\nLlfd: {:e}\nRkd: {:e}\n"
			"Llkd: {:e}\nRkq1: {:e}\nLlkq1: {:e}\nRkq2: {:e}\nLlkq2: {:e}\n",
			mRs, mLl, mLmd, mLmq, mRfd, mLlfd, mRkd, mLlkd, mRkq1, mLlkq1, mRkq2, mLlkq2);
}


void EMT::Ph3::SynchronGeneratorDQ::initialize(Matrix frequencies) {
	SimPowerComp<Real>::initialize(frequencies);

	// #### Compensation ####
	mCompensationOn = false;
	mCompensationCurrent = Matrix::Zero(3,1);
	// Calculate real compensation resistance from per unit
	mRcomp = mRcomp*mBase_Z;

	// #### General setup ####
	// Create matrices for state space representation
	calcStateSpaceMatrixDQ();

	// steady state per unit initial value
	initPerUnitStates();

	mVdq0 = Matrix::Zero(3,1);
	mIdq0 = Matrix::Zero(3,1);
	if (mNumDampingWindings == 2) {
		mVdq0 << mVsr(0,0), mVsr(3,0), mVsr(6,0);
		mIdq0 << mIsr(0,0), mIsr(3,0), mIsr(6,0);
	}
	else {
		mVdq0 << mVsr(0,0), mVsr(3,0), mVsr(5,0);
		mIdq0 << mIsr(0,0), mIsr(3,0), mIsr(5,0);
	}

	mIntfVoltage = mBase_V * dq0ToAbcTransform(mThetaMech, mVdq0);
	mIntfCurrent = mBase_I * dq0ToAbcTransform(mThetaMech, mIdq0);
}

void EMT::Ph3::SynchronGeneratorDQ::mnaApplySystemMatrixStamp(Matrix& systemMatrix) {
	if (!mCompensationOn)
		return;

	Real conductance = 1. / mRcomp;

	if (terminalNotGrounded(0)) {
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0,0), matrixNodeIndex(0,0), conductance);
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0,1), matrixNodeIndex(0,1), conductance);
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0,2), matrixNodeIndex(0,2), conductance);
		mSLog->info("Add {} to {}, {}", conductance, matrixNodeIndex(0,0), matrixNodeIndex(0,0));
		mSLog->info("Add {} to {}, {}", conductance, matrixNodeIndex(0,1), matrixNodeIndex(0,1));
		mSLog->info("Add {} to {}, {}", conductance, matrixNodeIndex(0,2), matrixNodeIndex(0,2));
	}
}

void EMT::Ph3::SynchronGeneratorDQ::mnaApplyRightSideVectorStamp(Matrix& rightVector) {
	if (mCompensationOn)
		mCompensationCurrent = mIntfVoltage / mRcomp;

	// If the interface current is positive, it is flowing out of the connected node and into ground.
	// Therefore, the generator is interfaced as a consumer but since the currents are reversed the equations
	// are in generator mode.
	if (terminalNotGrounded(0)) {
		Math::setVectorElement(rightVector, matrixNodeIndex(0,0), -mIntfCurrent(0,0) + mCompensationCurrent(0,0));
		Math::setVectorElement(rightVector, matrixNodeIndex(0,1), -mIntfCurrent(1,0) + mCompensationCurrent(1,0));
		Math::setVectorElement(rightVector, matrixNodeIndex(0,2), -mIntfCurrent(2,0) + mCompensationCurrent(2,0));
	}
}

void EMT::Ph3::SynchronGeneratorDQ::mnaUpdateVoltage(const Matrix& leftVector) {
	mIntfVoltage(0,0) = Math::realFromVectorElement(leftVector, matrixNodeIndex(0,0));
	mIntfVoltage(1,0) = Math::realFromVectorElement(leftVector, matrixNodeIndex(0,1));
	mIntfVoltage(2,0) = Math::realFromVectorElement(leftVector, matrixNodeIndex(0,2));
}

void EMT::Ph3::SynchronGeneratorDQ::MnaPostStep::execute(Real time, Int timeStepCount) {
	mSynGen.mnaUpdateVoltage(*mLeftVector);
}

Matrix EMT::Ph3::SynchronGeneratorDQ::abcToDq0Transform(Real theta, Matrix& abcVector) {
	Matrix dq0Vector(3, 1);
	Matrix abcToDq0(3, 3);

	// Park transform according to Kundur
	abcToDq0 <<
		 2./3.*cos(theta),	2./3.*cos(theta - 2.*PI/3.),  2./3.*cos(theta + 2.*PI/3.),
		-2./3.*sin(theta), -2./3.*sin(theta - 2.*PI/3.), -2./3.*sin(theta + 2.*PI/3.),
		 1./3., 			1./3., 						  1./3.;

	dq0Vector = abcToDq0 * abcVector;

	return dq0Vector;
}

Matrix EMT::Ph3::SynchronGeneratorDQ::dq0ToAbcTransform(Real theta, Matrix& dq0Vector) {
	Matrix abcVector(3, 1);
	Matrix dq0ToAbc(3, 3);

	// Park transform according to Kundur
	dq0ToAbc <<
		cos(theta), 		   -sin(theta), 		   1.,
		cos(theta - 2.*PI/3.), -sin(theta - 2.*PI/3.), 1.,
		cos(theta + 2.*PI/3.), -sin(theta + 2.*PI/3.), 1.;

	abcVector = dq0ToAbc * dq0Vector;

	return abcVector;
}
