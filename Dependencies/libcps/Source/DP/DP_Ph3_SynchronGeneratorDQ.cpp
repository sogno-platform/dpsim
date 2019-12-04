/**
 * @file
 * @author Markus Mirz <mmirz@eonerc.rwth-aachen.de>
 * @copyright 2017-2018, Institute for Automation of Complex Power Systems, EONERC
 *
 * CPowerSystems
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *********************************************************************************/

#include <cps/DP/DP_Ph3_SynchronGeneratorDQ.h>

//Added for testing/printing purposes:
#include <iostream>
#include <fstream>

using namespace CPS;
using namespace std;

DP::Ph3::SynchronGeneratorDQ::SynchronGeneratorDQ(String uid, String name, Logger::Level logLevel)
	: PowerComponent<Complex>(uid, name, logLevel) {
	mPhaseType = PhaseType::ABC;
	setTerminalNumber(1);
	mIntfVoltage = MatrixComp::Zero(3,1);
	mIntfCurrent = MatrixComp::Zero(3,1);

	addAttribute<Real>("w_r", &mOmMech, Flags::read);
}

DP::Ph3::SynchronGeneratorDQ::SynchronGeneratorDQ(String name, Logger::Level logLevel)
	: SynchronGeneratorDQ(name, name, logLevel) {
}

DP::Ph3::SynchronGeneratorDQ::~SynchronGeneratorDQ() {
}

void DP::Ph3::SynchronGeneratorDQ::setParametersFundamentalPerUnit(
	Real nomPower, Real nomVolt, Real nomFreq, Int poleNumber, Real nomFieldCur,
	Real Rs, Real Ll, Real Lmd, Real Lmq, Real Rfd, Real Llfd, Real Rkd, Real Llkd,
	Real Rkq1, Real Llkq1, Real Rkq2, Real Llkq2, Real inertia,
	Real initActivePower, Real initReactivePower, Real initTerminalVolt, Real initVoltAngle,
	Real initFieldVoltage, Real initMechPower) {

	Base::SynchronGenerator::setBaseAndFundamentalPerUnitParameters(
		nomPower, nomVolt, nomFreq, poleNumber, nomFieldCur,
		Rs, Ll, Lmd, Lmq, Rfd, Llfd, Rkd, Llkd, Rkq1, Llkq1, Rkq2, Llkq2, inertia);
	Base::SynchronGenerator::setInitialValues(initActivePower, initReactivePower,
		initTerminalVolt, initVoltAngle, initFieldVoltage, initMechPower);
}

void DP::Ph3::SynchronGeneratorDQ::initialize(Matrix frequencies) {
	PowerComponent<Complex>::initialize(frequencies);
	mSystemOmega = frequencies(0,0);

	// #### Compensation ####
	mCompensationOn = false;
	mCompensationCurrent = MatrixComp::Zero(3,1);
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

void DP::Ph3::SynchronGeneratorDQ::mnaApplySystemMatrixStamp(Matrix& systemMatrix) {
	if (!mCompensationOn)
		return;

	Real conductance = 1. / mRcomp;

	if (terminalNotGrounded(0)) {
		Math::addToMatrixElement(systemMatrix, simNodes(0)[0], simNodes(0)[0], Complex(conductance, 0));
		Math::addToMatrixElement(systemMatrix, simNodes(0)[1], simNodes(0)[1], Complex(conductance, 0));
		Math::addToMatrixElement(systemMatrix, simNodes(0)[2], simNodes(0)[2], Complex(conductance, 0));
		mSLog->info("Add {} to {}, {}", conductance, simNodes(0)[0], simNodes(0)[0]);
		mSLog->info("Add {} to {}, {}", conductance, simNodes(0)[1], simNodes(0)[1]);
		mSLog->info("Add {} to {}, {}", conductance, simNodes(0)[2], simNodes(0)[2]);
	}
}

void DP::Ph3::SynchronGeneratorDQ::mnaApplyRightSideVectorStamp(Matrix& rightVector) {
	if (mCompensationOn)
		mCompensationCurrent = mIntfVoltage / mRcomp;

	// If the interface current is positive, it is flowing out of the connected node and into ground.
	// Therefore, the generator is interfaced as a consumer but since the currents are reversed the equations
	// are in generator mode.
	if (terminalNotGrounded(0)) {
		Math::setVectorElement(rightVector, simNode(0,0), -mIntfCurrent(0,0) + mCompensationCurrent(0,0));
		Math::setVectorElement(rightVector, simNode(0,1), -mIntfCurrent(1,0) + mCompensationCurrent(1,0));
		Math::setVectorElement(rightVector, simNode(0,2), -mIntfCurrent(2,0) + mCompensationCurrent(2,0));
	}
}

void DP::Ph3::SynchronGeneratorDQ::mnaUpdateVoltage(const Matrix& leftVector) {
	mIntfVoltage(0,0) = Math::complexFromVectorElement(leftVector, simNode(0,0));
	mIntfVoltage(1,0) = Math::complexFromVectorElement(leftVector, simNode(0,1));
	mIntfVoltage(2,0) = Math::complexFromVectorElement(leftVector, simNode(0,2));
}

void DP::Ph3::SynchronGeneratorDQ::MnaPostStep::execute(Real time, Int timeStepCount) {
	mSynGen.mnaUpdateVoltage(*mLeftVector);
}

Matrix DP::Ph3::SynchronGeneratorDQ::abcToDq0Transform(Real theta, MatrixComp& abcVector) {
	// Balanced case because we do not return the zero sequence component
	Complex alpha(cos(2. / 3. * PI), sin(2. / 3. * PI));
	Complex thetaCompInv(cos(-theta), sin(-theta));

	MatrixComp abcToPnz(3, 3);
	abcToPnz <<
		1, 1, 1,
		1, alpha, pow(alpha, 2),
		1, pow(alpha, 2), alpha;
	abcToPnz = (1. / 3.) * abcToPnz;

	MatrixComp pnzVector(3, 1);
	pnzVector = abcToPnz * abcVector * thetaCompInv;

	Matrix dq0Vector(3, 1);
	dq0Vector <<
		pnzVector(1, 0).real(),
		pnzVector(1, 0).imag(),
		0;

	return dq0Vector;
}

MatrixComp DP::Ph3::SynchronGeneratorDQ::dq0ToAbcTransform(Real theta, Matrix& dq0) {
	// Balanced case because we do not consider the zero sequence component
	Complex alpha(cos(2. / 3. * PI), sin(2. / 3. * PI));
	Complex thetaComp(cos(theta), sin(theta));
	// Matrix to transform from symmetrical components to ABC
	MatrixComp pnzToAbc(3, 3);
	pnzToAbc <<
		1, 1, 1,
		1, pow(alpha, 2), alpha,
		1, alpha, pow(alpha, 2);
	// Symmetrical components vector
	MatrixComp pnzVector(3, 1);
	// Picking only d and q for positive sequence component
	pnzVector <<
		0,
		Complex(dq0(0,0), dq0(1,0)),
		0;
	// ABC vector
	MatrixComp abcCompVector(3, 1);
	abcCompVector = pnzToAbc * pnzVector * thetaComp;

	return abcCompVector;
}

void DP::Ph3::SynchronGeneratorDQ::trapezoidalFluxStates() {
	/*
	// Calculation of rotational speed with euler
	if (mHasTurbineGovernor == true)
		mMechTorque = mTurbineGovernor.step(mOmMech, 1, 300e6 / 555e6, mTimeStep);

	mElecTorque = (mPsid*mIq - mPsiq*mId);
	mOmMech = mOmMech + mTimeStep * (1 / (2 * mH) * (mMechTorque - mElecTorque));

	//Calculation of flux
	Matrix A = (mResistanceMat*mReactanceMat - mOmMech*mOmegaFluxMat);
	Matrix B = Matrix::Identity(7, 7);

	if (numMethod == NumericalMethod::Trapezoidal_flux)
		Fluxes = Math::StateSpaceTrapezoidal(Fluxes, A, B, mTimeStep*mBase_OmElec, dqVoltages);
	else
		Fluxes = Math::StateSpaceEuler(Fluxes, A, B, mTimeStep*mBase_OmElec, dqVoltages);

	// Calculation of currents based on inverse of inductance matrix
	mId = ((mLlfd*mLlkd + mLmd*(mLlfd + mLlkd))*mPsid - mLmd*mLlkd*mPsifd - mLlfd*mLmd*mPsikd) / detLd;
	mIfd = (mLlkd*mLmd*mPsid - (mLl*mLlkd + mLmd*(mLl + mLlkd))*mPsifd + mLmd*mLl*mPsikd) / detLd;
	mIkd = (mLmd*mLlfd*mPsid + mLmd*mLl*mPsifd - (mLmd*(mLlfd + mLl) + mLl*mLlfd)*mPsikd) / detLd;
	mIq = ((mLlkq1*mLlkq2 + mLmq*(mLlkq1 + mLlkq2))*mPsiq - mLmq*mLlkq2*mPsikq1 - mLmq*mLlkq1*mPsikq2) / detLq;
	mIkq1 = (mLmq*mLlkq2*mPsiq - (mLmq*(mLlkq2 + mLl) + mLl*mLlkq2)*mPsikq1 + mLmq*mLl*mPsikq2) / detLq;
	mIkq2 = (mLmq*mLlkq1*mPsiq + mLmq*mLl*mPsikq1 - (mLmq*(mLlkq1 + mLl) + mLl*mLlkq1)*mPsikq2) / detLq;
	mI0 = -mPsi0 / mLl;
	*/
}

void DP::Ph3::SynchronGeneratorDQ::trapezoidalCurrentStates() {
	/*
	Matrix A = (mReactanceMat*mResistanceMat);
	Matrix B = mReactanceMat;
	Matrix C = Matrix::Zero(7, 1);
	C(0, 0) = -mOmMech*mPsid;
	C(1, 0) = mOmMech*mPsiq;
	C = mReactanceMat*C;

	dqCurrents = Math::StateSpaceTrapezoidal(dqCurrents, A, B, C, mTimeStep*mOmMech, dqVoltages);

	//Calculation of currents based on inverse of inductance matrix
	mPsiq = -(mLl + mLmq)*mIq + mLmq*mIkq1 + mLmq*mIkq2;
	mPsid = -(mLl + mLmd)*mId + mLmd*mIfd + mLmd*mIkd;
	mPsi0 = -mLl*mI0;
	mPsikq1 = -mLmq*mIq + (mLlkq1 + mLmq)*mIkq1 + mLmq*mIkq2;
	mPsikq2 = -mLmq*mIq + mLmq*mIkq1 + (mLlkq2 + mLmq)*mIkq2;
	mPsifd = -mLmd*mId + (mLlfd + mLmd)*mIfd + mLmd*mIkd;
	mPsikd = -mLmd*mId + mLmd*mIfd + (mLlkd + mLmd)*mIkd;
	*/
}
