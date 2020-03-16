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

#include <cps/Base/Base_SynchronGenerator.h>

using namespace CPS;

void Base::SynchronGenerator::setBaseParameters(Real nomPower, Real nomVolt, Real nomFreq) {
	mNomPower = nomPower;
	mNomVolt = nomVolt;
	mNomFreq = nomFreq;
	mNomOmega = nomFreq * 2*PI;

	// Set base stator values
	mBase_V_RMS = mNomVolt / sqrt(3);
	mBase_V = mBase_V_RMS * sqrt(2);
	mBase_I_RMS = mNomPower / (3 * mBase_V_RMS);
	mBase_I = mBase_I_RMS * sqrt(2);
	mBase_Z = mBase_V / mBase_I;

	mBase_OmElec = mNomOmega;
	mBase_L = mBase_Z / mBase_OmElec;
	mBase_Psi = mBase_L * mBase_I;

	mBase_OmMech = mBase_OmElec;
	mBase_T = mNomPower / mBase_OmMech;
}

void Base::SynchronGenerator::setBaseAndFundamentalPerUnitParameters(
	Real nomPower, Real nomVolt, Real nomFreq, Int poleNumber, Real nomFieldCur,
	Real Rs, Real Ll, Real Lmd, Real Lmq, Real Rfd, Real Llfd,
	Real Rkd, Real Llkd, Real Rkq1, Real Llkq1, Real Rkq2, Real Llkq2,
	Real inertia) {

	// PoleNumber otherwise not set but currently not used in SynchronGeneratorDQ
	mPoleNumber = poleNumber;

	mParameterType = ParameterType::perUnit;
	mNumericalMethod = NumericalMethod::Trapezoidal;

	setBaseParameters(nomPower, nomVolt, nomFreq);

	if (Rkq2 == 0 && Llkq2 == 0)
		mNumDampingWindings = 1;
	else
		mNumDampingWindings = 2;

	setFundamentalPerUnitParameters(Rs, Ll, Lmd, Lmq, Rfd, Llfd, Rkd, Llkd, Rkq1, Llkq1, Rkq2, Llkq2, inertia);
}

void Base::SynchronGenerator::setFundamentalPerUnitParameters(
	Real Rs, Real Ll, Real Lmd, Real Lmq,
	Real Rfd, Real Llfd, Real Rkd, Real Llkd,
	Real Rkq1, Real Llkq1, Real Rkq2, Real Llkq2,
	Real inertia) {

	// base rotor values
	mBase_ifd = Lmd * mNomFieldCur;
	mBase_vfd = mNomPower / mBase_ifd;
	mBase_Zfd = mBase_vfd / mBase_ifd;
	mBase_Lfd = mBase_Zfd / mBase_OmElec;

	mRs = Rs;
	mLl = Ll;
	mLmd = Lmd;
	mLd = mLl + mLmd;
	mLmq = Lmq;
	mLq = mLl + mLmq;
	mRfd = Rfd;
	mLlfd = Llfd;
	mLfd = mLlfd + mLmd;
	mRkd = Rkd;
	mLlkd = Llkd;
	mLkd = mLlkd + mLmd;
	mRkq1 = Rkq1;
	mLlkq1 = Llkq1;
	mLkq1 = mLlkq1 + mLmq;
	mRkq2 = Rkq2;
	mLlkq2 = Llkq2;
	mLkq2 = mLlkq2 + mLmq;
	mInertia = inertia;

	if (mNumDampingWindings == 1) {
		mVsr = Matrix::Zero(6, 1);
		mIsr = Matrix::Zero(6, 1);
		mPsisr = Matrix::Zero(6, 1);
		mInductanceMat = Matrix::Zero(6, 6);
		mResistanceMat = Matrix::Zero(6, 6);

		// Determinant of Lq(inductance matrix of q axis)
		//Real detLq = -(mLl + mLmq)*(mLlkq1 + mLmq) + mLmq*mLmq;
		// Determinant of Ld (inductance matrix of d axis)
		//Real detLd = (mLmd + mLl)*(-mLlfd*mLlkd - mLlfd*mLmd - mLmd*mLlkd) + mLmd*mLmd*(mLlfd + mLlkd);

		mInductanceMat <<
			mLd,	mLmd,	mLmd,	0,		0, 		0,
			mLmd, 	mLfd, 	mLmd,	0,		0, 		0,
			mLmd, 	mLmd, 	mLkd,	0,		0, 		0,
			0,		0,		0,		mLq, 	mLmq,	0,
			0,		0,		0,		mLmq, 	mLkq1, 	0,
			0,		0,		0,		0,		0,		mLl;

		mResistanceMat <<
			mRs, 	0, 		0, 		0, 		0, 		0,
			0, 		mRfd, 	0, 		0, 		0, 		0,
			0, 		0, 		mRkd, 	0, 		0, 		0,
			0, 		0, 		0, 		mRs, 	0, 		0,
			0, 		0, 		0, 		0, 		mRkq1,	0,
			0, 		0, 		0, 		0, 		0,		mRs;

		//Compute inverse Inductance Matrix:
		mInvInductanceMat = mInductanceMat.inverse();
	}
	else {
		mVsr = Matrix::Zero(7, 1);
		mIsr = Matrix::Zero(7, 1);
		mPsisr = Matrix::Zero(7, 1);
		mInductanceMat = Matrix::Zero(7, 7);
		mResistanceMat = Matrix::Zero(7, 7);

		// Determinant of Lq(inductance matrix of q axis)
		//Real detLq = -mLmq*mLlkq2*(mLlkq1 + mLl) - mLl*mLlkq1*(mLlkq2 + mLmq);
		// Determinant of Ld (inductance matrix of d axis)
		//Real detLd = (mLmd + mLl)*(-mLlfd*mLlkd - mLlfd*mLmd - mLmd*mLlkd) + mLmd*mLmd*(mLlfd + mLlkd);

		mInductanceMat <<
			mLd,	mLmd,	mLmd,	0,		0, 		0,		0,
			mLmd, 	mLfd, 	mLmd,	0,		0, 		0,		0,
			mLmd, 	mLmd, 	mLkd,	0,		0, 		0,		0,
			0,		0,		0,		mLq, 	mLmq,	mLmq,	0,
			0,		0,		0,		mLmq, 	mLkq1, 	mLmq, 	0,
			0,		0,		0,		mLmq, 	mLmq, 	mLkq2, 	0,
			0,		0,		0,		0,		0,		0,		mLl;

		mResistanceMat <<
			mRs, 	0, 		0, 		0, 		0, 		0, 		0,
			0, 		mRfd, 	0, 		0, 		0, 		0, 		0,
			0, 		0, 		mRkd, 	0, 		0, 		0, 		0,
			0, 		0, 		0, 		mRs, 	0, 		0, 		0,
			0, 		0, 		0, 		0, 		mRkq1,	0, 		0,
			0, 		0, 		0, 		0, 		0,		mRkq2,	0,
			0, 		0, 		0, 		0, 		0,		0,		mRs;

		//Compute inverse Inductance Matrix:
		mInvInductanceMat = mInductanceMat.inverse();
	}
}

void Base::SynchronGenerator::setInitialValues(Real initActivePower, Real initReactivePower,
	Real initTerminalVolt, Real initVoltAngle,
	Real initFieldVoltage, Real initMechPower) {
	mInitElecPower = Complex(initActivePower, initReactivePower);
	mInitTerminalVoltage = initTerminalVolt;
	mInitVoltAngle = initVoltAngle;
	mInitFieldVoltage = initFieldVoltage;
	mInitMechPower = initMechPower;
}

void Base::SynchronGenerator::initPerUnitStates() {
	// Power in per unit
	Real init_P = mInitElecPower.real() / mNomPower;
	Real init_Q = mInitElecPower.imag() / mNomPower;
	Real init_S_abs = sqrt(pow(init_P, 2.) + pow(init_Q, 2.));
	//Complex init_S = mInitElecPower;
	// Terminal voltage in pu
	Real init_vt_abs = mInitTerminalVoltage / mBase_V;
	//Complex init_vt = Complex(mInitTerminalVoltage*cos(mInitVoltAngle), mInitTerminalVoltage*sin(mInitVoltAngle));
	Real init_it_abs = init_S_abs / init_vt_abs;
	//Complex init_it = std::conj( init_S / init_vt );
	// Power factor
	Real init_pf = acos(init_P / init_S_abs);
	// Load angle
	Real init_delta = atan(((mLmq + mLl) * init_it_abs * cos(init_pf) - mRs * init_it_abs * sin(init_pf)) /
		(init_vt_abs + mRs * init_it_abs * cos(init_pf) + (mLmq + mLl) * init_it_abs * sin(init_pf)));
	//Real init_delta_deg = init_delta / PI * 180;


	// Electrical torque
	//Real init_Te = init_P + mRs * pow(init_it, 2.);

	// dq stator voltages and currents
	Real init_vd = init_vt_abs * sin(init_delta);
	Real init_vq = init_vt_abs * cos(init_delta);
	Real init_id = init_it_abs * sin(init_delta + init_pf);
	Real init_iq = init_it_abs * cos(init_delta + init_pf);

	// Rotor voltage and current
	Real init_ifd = (init_vq + mRs * init_iq + (mLmd + mLl) * init_id) / mLmd;
	Real init_vfd = mRfd * init_ifd;

	// Flux linkages
	Real init_psid = init_vq + mRs * init_iq;
	Real init_psiq = -init_vd - mRs * init_id;
	Real init_psifd = mLfd * init_ifd - mLmd * init_id;
	Real init_psikd = mLmd * (init_ifd - init_id);
	Real init_psiq1 = -mLmq * init_iq;
	Real init_psiq2 = -mLmq * init_iq;

	// Initialize mechanical variables
	mOmMech = 1;
	mMechPower = mInitMechPower / mNomPower;
	mMechTorque = mMechPower / 1;
	mThetaMech = mInitVoltAngle + init_delta - PI / 2.;

	if (mNumDampingWindings == 2) {
		mVsr << init_vd, init_vfd, 0, init_vq, 0, 0, 0;
		mIsr << -init_id, init_ifd, 0, -init_iq, 0, 0, 0;
		mPsisr << init_psid, init_psifd, init_psikd, init_psiq, init_psiq1, init_psiq2, 0;
	}
	else {
		mVsr << init_vd, init_vfd, 0, init_vq, 0, 0;
		mIsr << -init_id, init_ifd, 0, -init_iq, 0, 0;
		mPsisr << init_psid, init_psifd, init_psikd, init_psiq, init_psiq1, 0;
	}

	mElecTorque = (mPsisr(3,0)*mIsr(0,0) - mPsisr(0,0)*mIsr(3,0));
}

void Base::SynchronGenerator::calcStateSpaceMatrixDQ() {
	if (mNumDampingWindings == 2) {
		mLad = 1. / (1./mLmd + 1./mLl + 1./mLlfd + 1./mLlkd);
		mLaq = 1. / (1./mLmq + 1./mLl + 1./mLlkq1 + 1./mLlkq2);

		mPsisr = Matrix::Zero(7, 1);

		mOmegaFluxMat = Matrix::Zero(7, 7);
		mOmegaFluxMat <<
		0,	0,	0, 	1, 	0, 	0, 	0,
		0, 	0, 	0, 	0, 	0, 	0, 	0,
		0, 	0, 	0, 	0, 	0, 	0, 	0,
		-1,	0, 	0, 	0, 	0, 	0, 	0,
		0, 	0, 	0, 	0, 	0, 	0, 	0,
		0, 	0, 	0, 	0, 	0, 	0, 	0,
		0, 	0, 	0, 	0, 	0, 	0, 	0;

		mFluxStateSpaceMat = Matrix::Zero(7, 7); //order of lambdas: ds; fd; kd; qs; kq1; kq2; 0s
		mFluxStateSpaceMat <<
		mRs/mLl*mLad/mLl -mRs/mLl,	mRs/mLl*mLad/mLlfd, 				mRs/mLl*mLad/mLlkd,					0, 							0, 										0,										0,
		mRfd/mLlfd*mLad/mLl,		mRfd/mLlfd*mLad/mLlfd -mRfd/mLlfd, 	mRfd/mLlfd*mLad/mLlkd,				0, 							0, 										0,										0,
		mRkd/mLlkd*mLad/mLl, 		mRkd/mLlkd*mLad/mLlfd, 				mRkd/mLlkd*mLad/mLlkd -mRkd/mLlkd,	0, 							0, 										0,										0,
		0, 							0, 									0, 									mRs/mLl*mLaq/mLl -mRs/mLl,	mRs/mLl*mLaq/mLlkq1,					mRs/mLl*mLaq/mLlkq2,					0,
		0, 							0, 									0, 									mRkq1/mLlkq1*mLaq/mLl,		mRkq1/mLlkq1*mLaq/mLlkq1 -mRkq1/mLlkq1,	mRkq1/mLlkq1*mLaq/mLlkq2, 				0,
		0, 							0, 									0, 									mRkq2/mLlkq2*mLaq/mLl, 		mRkq2/mLlkq2*mLaq/mLlkq1,				mRkq2/mLlkq2*mLaq/mLlkq2 -mRkq2/mLlkq2,	0,
		0, 							0, 									0, 									0, 							0, 										0, 										-mRs/mLl;

		mFluxToCurrentMat = Matrix::Zero(7, 7); //need for electric torque id, iq ->1st and 4th row
		mFluxToCurrentMat <<
		1./mLl -mLad/mLl/mLl,	-mLad/mLlfd/mLl,			-mLad/mLlkd/mLl, 			0,						0,								0,								0,
		-mLad/mLl/mLlfd,		1./mLlfd -mLad/mLlfd/mLlfd,	-mLad/mLlkd/mLlfd,			0,						0,								0,								0,
		-mLad/mLl/mLlkd,		-mLad/mLlfd/mLlkd,			1./mLlkd -mLad/mLlkd/mLlkd,	0,						0,								0,								0,
		0,						0,							0,							1./mLl -mLaq/mLl/mLl,	-mLaq/mLlkq1/mLl,				-mLaq/mLlkq2/mLl, 				0,
		0,						0,							0,							-mLaq/mLl/mLlkq1,		1./mLlkq1 -mLaq/mLlkq1/mLlkq1,	-mLaq/mLlkq2/mLlkq1,			0,
		0,						0,							0,							-mLaq/mLl/mLlkq2,		-mLaq/mLlkq1/mLlkq2,			1./mLlkq2 -mLaq/mLlkq2/mLlkq2,	0,
		0,						0,							0,							0,						0,								0,								1./mLl;
	}
	else {
		mLad = 1. / (1. / mLmd + 1. / mLl + 1. / mLlfd + 1. / mLlkd);
		mLaq = 1. / (1. / mLmq + 1. / mLl + 1. / mLlkq1);

		mPsisr = Matrix::Zero(6, 1);

		mOmegaFluxMat = Matrix::Zero(6, 6);
		mOmegaFluxMat <<
		0,	0,	0, 	1, 	0, 	0,
		0, 	0, 	0, 	0, 	0, 	0,
		0, 	0, 	0, 	0, 	0, 	0,
		-1, 0, 	0, 	0, 	0, 	0,
		0, 	0, 	0, 	0, 	0, 	0,
		0, 	0, 	0, 	0, 	0, 	0;

		mFluxStateSpaceMat = Matrix::Zero(6, 6);
		mFluxStateSpaceMat << //same order as above; only without kq2
		mRs/mLl*mLad/mLl -mRs/mLl,	mRs/mLl*mLad/mLlfd, 				mRs/mLl*mLad/mLlkd,					0, 							0, 										0,
		mRfd/mLlfd*mLad/mLl,		mRfd/mLlfd*mLad/mLlfd -mRfd/mLlfd, 	mRfd/mLlfd*mLad/mLlkd,				0, 							0, 										0,
		mRkd/mLlkd*mLad/mLl, 		mRkd/mLlkd*mLad/mLlfd, 				mRkd/mLlkd*mLad/mLlkd -mRkd/mLlkd,	0, 							0, 										0,
		0, 							0, 									0, 									mRs/mLl*mLaq/mLl -mRs/mLl,	mRs/mLl*mLaq/mLlkq1,					0,
		0, 							0, 									0, 									mRkq1/mLlkq1*mLaq/mLl,		mRkq1/mLlkq1*mLaq/mLlkq1 -mRkq1/mLlkq1,	0,
		0, 							0, 									0, 									0, 							0, 										-mRs/mLl;

		mFluxToCurrentMat = Matrix::Zero(6, 6);
		mFluxToCurrentMat <<
		1./mLl -mLad/mLl/mLl,	-mLad/mLlfd/mLl,			-mLad/mLlkd/mLl, 			0,						0,								0,
		-mLad/mLl/mLlfd,		1./mLlfd -mLad/mLlfd/mLlfd,	-mLad/mLlkd/mLlfd,			0,						0,								0,
		-mLad/mLl/mLlkd,		-mLad/mLlfd/mLlkd,			1./mLlkd -mLad/mLlkd/mLlkd,	0,						0,								0,
		0,						0,							0,							1./mLl -mLaq/mLl/mLl,	-mLaq/mLlkq1/mLl,				0,
		0,						0,							0,							-mLaq/mLl/mLlkq1,		1./mLlkq1 -mLaq/mLlkq1/mLlkq1,	0,
		0,						0,							0,							0,						0,								1./mLl;
	}
}

Real Base::SynchronGenerator::calcHfromJ(Real J, Real omegaNominal, Int polePairNumber) {
	return J * 0.5 * omegaNominal*omegaNominal / polePairNumber;
}

void Base::SynchronGenerator::addExciter(Real Ta, Real Ka, Real Te, Real Ke,
	Real Tf, Real Kf, Real Tr, Real Lad, Real Rfd) {
	//mExciter = Signal::Exciter(Ta, Ka, Te, Ke, Tf, Kf, Tr, Lad, Rfd);
	//mExciter.initialize(1, 1);
	mHasExciter = true;
}

void Base::SynchronGenerator::addGovernor(Real Ta, Real Tb, Real Tc, Real Fa,
	Real Fb, Real Fc, Real K, Real Tsr, Real Tsm, Real Tm_init, Real PmRef) {
	//mTurbineGovernor = Signal::TurbineGovernor(Ta, Tb, Tc, Fa, Fb, Fc, K, Tsr, Tsm);
	//mTurbineGovernor.initialize(PmRef, Tm_init);
	mHasTurbineGovernor = true;
}
