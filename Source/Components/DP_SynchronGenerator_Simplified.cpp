/** Synchron generator
*
* @author Markus Mirz <mmirz@eonerc.rwth-aachen.de>
* @copyright 2017, Institute for Automation of Complex Power Systems, EONERC
*
* DPsim
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

#include "DP_SynchronGenerator_Simplified.h"
#include "../IntegrationMethod.h"

using namespace DPsim;

Components::DP::SynchronGeneratorSimplified::SynchronGeneratorSimplified(String name, Int node1, Int node2, Int node3,
		Real nomPower, Real nomVolt, Real nomFreq, Int poleNumber, Real nomFieldCur,
		Real Rs, Real Ll, Real Lmd, Real Lmd0, Real Lmq, Real Lmq0,
		Real Rfd, Real Llfd, Real Rkd, Real Llkd,
		Real Rkq1, Real Llkq1, Real Rkq2, Real Llkq2,
		Real inertia, Real Ra, Logger::Level logLevel)
		: SynchronGeneratorBase(name, node1, node2, node3, nomPower, nomVolt, nomFreq, poleNumber, nomFieldCur,
				Rs, Ll, Lmd, Lmd0, Lmq, Lmq0, Rfd, Llfd, Rkd, Llkd, Rkq1, Llkq1, Rkq2, Llkq2,
				inertia, logLevel)
{
		mRa = Ra;
}

Components::DP::SynchronGeneratorSimplified::~SynchronGeneratorSimplified() {

}

void Components::DP::SynchronGeneratorSimplified::applySystemMatrixStamp(SystemModel& system)
{
		mRa = mRa*mBase_Z;

		Real mConductance = 1 / mRa;

		// Set diagonal entries
		if (mNode1 >= 0) {
				system.addCompToSystemMatrix(mNode1, mNode1, Complex(mConductance, 0));
		}
		if (mNode2 >= 0) {
				//mLog.Log(Logger::Level::DEBUG) << "Add " << mConductance << " to " << mNode2 << "," << mNode2 << std::endl;
				system.addCompToSystemMatrix(mNode2, mNode2, Complex(mConductance, 0));
		}
		if (mNode3 >= 0) {
				//mLog.Log(Logger::Level::DEBUG) << "Add " << mConductance << " to " << mNode2 << "," << mNode2 << std::endl;
				system.addCompToSystemMatrix(mNode3, mNode3, Complex(mConductance, 0));
		}

}


void Components::DP::SynchronGeneratorSimplified::initialize(Real om, Real dt,
		Real initActivePower, Real initReactivePower, Real initTerminalVolt,
		Real initVoltAngle, Real initFieldVoltage, Real initMechPower)
{
		// Create matrices for state space representation
		mVoltages = Matrix::Zero(3, 1);
		mFluxes = Matrix::Zero(3, 1);
		mCurrents = Matrix::Zero(3, 1);
		mInductanceMat = Matrix::Zero(3, 3);
		mResistanceMat = Matrix::Zero(3, 3);
		mReactanceMat = Matrix::Zero(3, 3);
		mOmegaFluxMat = Matrix::Zero(3, 3);

		mInductanceMat <<
				-(mLl + mLmq), 0, 0,
				0, -(mLl + mLmd), mLmd,
				0, -mLmd, mLlfd + mLmd;

		mResistanceMat <<
				mRs, 0, 0,
				0, mRs, 0,
				0, 0, -mRfd;

		mOmegaFluxMat <<
				0, 1, 0,
				-1, 0, 0,
				0, 0, 0;

		mReactanceMat = mInductanceMat.inverse();

		// steady state per unit initial value
		initStatesInPerUnit(initActivePower, initReactivePower, initTerminalVolt, initVoltAngle, initFieldVoltage, initMechPower);

		mVaRe = dq0ToAbcTransform(mThetaMech, mVd* mBase_v, mVq* mBase_v, mV0* mBase_v)(0);
		mVbRe = dq0ToAbcTransform(mThetaMech, mVd* mBase_v, mVq* mBase_v, mV0* mBase_v)(1);
		mVcRe = dq0ToAbcTransform(mThetaMech, mVd* mBase_v, mVq* mBase_v, mV0* mBase_v)(2);
		mVaIm = dq0ToAbcTransform(mThetaMech, mVd* mBase_v, mVq* mBase_v, mV0* mBase_v)(3);
		mVbIm = dq0ToAbcTransform(mThetaMech, mVd* mBase_v, mVq* mBase_v, mV0* mBase_v)(4);
		mVcIm = dq0ToAbcTransform(mThetaMech, mVd* mBase_v, mVq* mBase_v, mV0* mBase_v)(5);

		mIaRe = dq0ToAbcTransform(mThetaMech, mId * mBase_i, mIq * mBase_i, mI0 * mBase_i)(0);
		mIbRe = dq0ToAbcTransform(mThetaMech, mId * mBase_i, mIq * mBase_i, mI0 * mBase_i)(1);
		mIcRe = dq0ToAbcTransform(mThetaMech, mId * mBase_i, mIq * mBase_i, mI0 * mBase_i)(2);
		mIaIm = dq0ToAbcTransform(mThetaMech, mId * mBase_i, mIq * mBase_i, mI0 * mBase_i)(3);
		mIbIm = dq0ToAbcTransform(mThetaMech, mId * mBase_i, mIq * mBase_i, mI0 * mBase_i)(4);
		mIcIm = dq0ToAbcTransform(mThetaMech, mId * mBase_i, mIq * mBase_i, mI0 * mBase_i)(5);
}

void Components::DP::SynchronGeneratorSimplified::step(SystemModel& system, Real time)
{
		mG_load = system.getCurrentSystemMatrix();
		mR_load = mG_load.inverse() / mBase_Z;

		stepInPerUnit(system.getOmega(), system.getTimeStep(), time, system.getNumMethod());

		if (mNode1 >= 0) {
				system.addCompToRightSideVector(mNode1, Complex(mIaRe + mVaRe / mRa, mIaIm + mVaIm / mRa));
		}
		if (mNode2 >= 0) {
				system.addCompToRightSideVector(mNode2, Complex(mIbRe + mVbRe / mRa, mIbIm + mVbIm / mRa));
		}
		if (mNode3 >= 0) {
				system.addCompToRightSideVector(mNode3, Complex(mIcRe + mVcRe / mRa, mIcIm + mVcIm / mRa));
		}

		if (mLogLevel != Logger::Level::NONE) {
				Matrix logValues(getStatorCurrents().rows() + getFluxes().rows() + getVoltages().rows() + getCurrents().rows() + 3, 1);
				logValues << getStatorCurrents(), getFluxes(), getVoltages(), getCurrents(), getElectricalTorque(), getRotationalSpeed(), getRotorPosition();
				mLog->LogDataLine(time, logValues);
		}
}

void Components::DP::SynchronGeneratorSimplified::stepInPerUnit(Real om, Real dt, Real time, NumericalMethod numMethod)
{
		//mVaRe = (1 / mBase_v) * mVaRe;
		//mVaIm = (1 / mBase_v) * mVaIm;
		//mVbRe = (1 / mBase_v) * mVbRe;
		//mVbIm = (1 / mBase_v) * mVbIm;
		//mVcRe = (1 / mBase_v) * mVcRe;
		//mVcIm = (1 / mBase_v) * mVcIm;

		//mIaRe = (1 / mBase_i) * mIaRe;
		//mIaIm = (1 / mBase_i) * mIaIm;
		//mIbRe = (1 / mBase_i) * mIbRe;
		//mIbIm = (1 / mBase_i) * mIbIm;
		//mIcRe = (1 / mBase_i) * mIcRe;
		//mIcIm = (1 / mBase_i) * mIcIm;

		//mVq = abcToDq0Transform(mThetaMech, mVaRe, mVbRe, mVcRe, mVaIm, mVbIm, mVcIm)(0);
		//mVd = abcToDq0Transform(mThetaMech, mVaRe, mVbRe, mVcRe, mVaIm, mVbIm, mVcIm)(1);
		//mV0 = abcToDq0Transform(mThetaMech, mVaRe, mVbRe, mVcRe, mVaIm, mVbIm, mVcIm)(2);


	   // Calculation of rotational speed with euler
		mElecTorque = (mPsid*mIq - mPsiq*mId);
		mOmMech = mOmMech + dt * (1 / (2 * mH) * (mMechTorque - mElecTorque));

		// Update mechanical rotor angle with respect to electrical angle
		mThetaMech = mThetaMech + dt * ((mOmMech - 1) * mBase_OmMech);

		mPsifd = mPsifd + dt*mBase_OmMech*(mVfd - mRfd*mIfd);

		mR_eq <<
				-mRs - mR_load(0, 0), (mLl + mLmq),
				-(mLl + mLmd) + mLmd*mLmd / (mLlfd + mLmd), -mRs - mR_load(0, 0);
		mE_eq <<
				0,
				(mLmd)*mPsifd / (mLlfd + mLmd);

		Matrix mIdq = Matrix::Zero(2, 1);
		mIdq = -mR_eq.inverse()*mE_eq;
		mId = mIdq(0);
		mIq = mIdq(1);
		mIfd = (mPsifd + mLmd*mId) / (mLlfd + mLmd);

		mCurrents(0, 0) = mIq;
		mCurrents(1, 0) = mId;
		mCurrents(2, 0) = mIfd;

		mFluxes = mInductanceMat*mCurrents;

		mPsiq = mFluxes(0, 0);
		mPsid = mFluxes(1, 0);
		mPsifd = mFluxes(2, 0);


		mIaRe = mBase_i * dq0ToAbcTransform(mThetaMech, mId, mIq, mI0)(0);
		mIbRe = mBase_i * dq0ToAbcTransform(mThetaMech, mId, mIq, mI0)(1);
		mIcRe = mBase_i * dq0ToAbcTransform(mThetaMech, mId, mIq, mI0)(2);
		mIaIm = mBase_i * dq0ToAbcTransform(mThetaMech, mId, mIq, mI0)(3);
		mIbIm = mBase_i * dq0ToAbcTransform(mThetaMech, mId, mIq, mI0)(4);
		mIcIm = mBase_i * dq0ToAbcTransform(mThetaMech, mId, mIq, mI0)(5);

		mIabc <<
				mIaRe,
				mIbRe,
				mIcRe,
				mIaIm,
				mIbIm,
				mIcIm;

}

void Components::DP::SynchronGeneratorSimplified::postStep(SystemModel& system)
{
		if (mNode1 >= 0) {
				mVaRe = system.getCompFromLeftSideVector(mNode1).real();
				mVaIm = system.getCompFromLeftSideVector(mNode1).imag();
		}
		else {
				mVaRe = 0;
				mVaIm = 0;
		}
		if (mNode2 >= 0) {
				mVbRe = system.getCompFromLeftSideVector(mNode2).real();
				mVbIm = system.getCompFromLeftSideVector(mNode2).imag();
		}
		else {
				mVbRe = 0;
				mVbIm = 0;
		}
		if (mNode3 >= 0) {
				mVcRe = system.getCompFromLeftSideVector(mNode3).real();
				mVcIm = system.getCompFromLeftSideVector(mNode3).imag();
		}
		else {
				mVcRe = 0;
				mVcIm = 0;
		}
}

Matrix Components::DP::SynchronGeneratorSimplified::abcToDq0Transform(Real theta, Real aRe, Real bRe, Real cRe, Real aIm, Real bIm, Real cIm)
{
		// Balanced case
		Complex alpha(cos(2. / 3. * PI), sin(2. / 3. * PI));
		Complex thetaCompInv(cos(-theta), sin(-theta));
		MatrixComp AbcToPnz(3, 3);
		AbcToPnz <<
				1, 1, 1,
				1, alpha, pow(alpha, 2),
				1, pow(alpha, 2), alpha;
		AbcToPnz = (1. / 3.) * AbcToPnz;

		MatrixComp abcVector(3, 1);
		abcVector <<
				Complex(aRe, aIm),
				Complex(bRe, bIm),
				Complex(cRe, cIm);

		MatrixComp pnzVector(3, 1);
		pnzVector = AbcToPnz * abcVector * thetaCompInv;

		Matrix dq0Vector(3, 1);
		dq0Vector <<
				pnzVector(1, 0).imag(),
				pnzVector(1, 0).real(),
				0;

		return dq0Vector;
}

Matrix Components::DP::SynchronGeneratorSimplified::dq0ToAbcTransform(Real theta, Real d, Real q, Real zero)
{
		// Balanced case
		Complex alpha(cos(2. / 3. * PI), sin(2. / 3. * PI));
		Complex thetaComp(cos(theta), sin(theta));
		MatrixComp PnzToAbc(3, 3);
		PnzToAbc <<
				1, 1, 1,
				1, pow(alpha, 2), alpha,
				1, alpha, pow(alpha, 2);

		MatrixComp pnzVector(3, 1);
		pnzVector <<
				0,
				Complex(d, q),
				Complex(0, 0);

		MatrixComp abcCompVector(3, 1);
		abcCompVector = PnzToAbc * pnzVector * thetaComp;

		Matrix abcVector(6, 1);
		abcVector <<
				abcCompVector(0, 0).real(),
				abcCompVector(1, 0).real(),
				abcCompVector(2, 0).real(),
				abcCompVector(0, 0).imag(),
				abcCompVector(1, 0).imag(),
				abcCompVector(2, 0).imag();

		return abcVector;
}
