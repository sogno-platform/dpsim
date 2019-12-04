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

#include <cps/EMT/EMT_Ph3_SynchronGeneratorDQ.h>

using namespace CPS;


EMT::Ph3::SynchronGeneratorDQ::SynchronGeneratorDQ(String uid, String name, Logger::Level logLevel)
	: PowerComponent<Real>(uid, name, logLevel) {
	mPhaseType = PhaseType::ABC;
	setTerminalNumber(1);
	mIntfVoltage = Matrix::Zero(3,1);
	mIntfCurrent = Matrix::Zero(3,1);

	addAttribute<Real>("w_r", &mOmMech, Flags::read);
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
		nomPower, nomVolt, nomFreq, poleNumber, nomFieldCur,
		Rs, Ll, Lmd, Lmq, Rfd, Llfd, Rkd, Llkd, Rkq1, Llkq1, Rkq2, Llkq2, inertia);
	Base::SynchronGenerator::setInitialValues(initActivePower, initReactivePower,
		initTerminalVolt, initVoltAngle, initFieldVoltage, initMechPower);
}

void EMT::Ph3::SynchronGeneratorDQ::initialize(Matrix frequencies) {
	PowerComponent<Real>::initialize(frequencies);

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
		Math::addToMatrixElement(systemMatrix, simNode(0,0), simNode(0,0), conductance);
		Math::addToMatrixElement(systemMatrix, simNode(0,1), simNode(0,1), conductance);
		Math::addToMatrixElement(systemMatrix, simNode(0,2), simNode(0,2), conductance);
		mSLog->info("Add {} to {}, {}", conductance, simNode(0,0), simNode(0,0));
		mSLog->info("Add {} to {}, {}", conductance, simNode(0,1), simNode(0,1));
		mSLog->info("Add {} to {}, {}", conductance, simNode(0,2), simNode(0,2));
	}
}

void EMT::Ph3::SynchronGeneratorDQ::mnaApplyRightSideVectorStamp(Matrix& rightVector) {
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

void EMT::Ph3::SynchronGeneratorDQ::mnaUpdateVoltage(const Matrix& leftVector) {
	mIntfVoltage(0,0) = Math::realFromVectorElement(leftVector, simNode(0,0));
	mIntfVoltage(1,0) = Math::realFromVectorElement(leftVector, simNode(0,1));
	mIntfVoltage(2,0) = Math::realFromVectorElement(leftVector, simNode(0,2));
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
