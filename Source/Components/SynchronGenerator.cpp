#include "SynchronGenerator.h"

using namespace DPsim;

SynchronGenerator::SynchronGenerator(std::string name, int node1, int node2, int node3,
	Real nomPower, Real nomVolt, Real nomFreq, int poleNumber, Real nomFieldCur,
	Real Rs, Real Ll, Real Lmd, Real Lmd0, Real Lmq, Real Lmq0,
	Real Rfd, Real Llfd, Real Rkd, Real Llkd,
	Real Rkq1, Real Llkq1, Real Rkq2, Real Llkq2,
	Real inertia) {

	this->mNode1 = node1 - 1;
	this->mNode2 = node2 - 1;
	this->mNode3 = node3 - 1;

	mNomPower = nomPower;
	mNomVolt = nomVolt;
	mNomFreq = nomFreq;
	mPoleNumber = poleNumber;
	mNomFieldCur = nomFieldCur;

	// base stator values
	mBase_V_RMS = mNomVolt / sqrt(3);
	mBase_v = mBase_V_RMS * sqrt(2);
	mBase_I_RMS = mNomPower / (3 * mBase_V_RMS);
	mBase_i = mBase_I_RMS * sqrt(2);
	mBase_Z = mBase_v / mBase_i;
	mBase_OmElec = 2 * DPS_PI * mNomFreq;
	mBase_OmMech = mBase_OmElec / (mPoleNumber / 2);
	mBase_L = mBase_Z / mBase_OmElec;
	mBase_Psi = mBase_L * mBase_i;
	mBase_T = mNomPower / mBase_OmMech;

	initWithPerUnitParam(Rs, Ll, Lmd, Lmd0, Lmq, Lmq0, Rfd, Llfd, Rkd, Llkd, Rkq1, Llkq1, Rkq2, Llkq2, inertia);

}

void SynchronGenerator::initWithPerUnitParam(
	Real Rs, Real Ll, Real Lmd, Real Lmd0, Real Lmq, Real Lmq0,
	Real Rfd, Real Llfd, Real Rkd, Real Llkd,
	Real Rkq1, Real Llkq1, Real Rkq2, Real Llkq2,
	Real H) {

	// base rotor values
	mBase_ifd = Lmd * mNomFieldCur;
	mBase_vfd = mNomPower / mBase_ifd;
	mBase_Zfd = mBase_vfd / mBase_ifd;
	mBase_Lfd = mBase_Zfd / mBase_OmElec;
	
	mRs = Rs;
	mLl = Ll;
	mLmd = Lmd;
	mLmd0 = Lmd0;
	mLmq = Lmq;
	mLmq0 = Lmq0;
	mRfd = Rfd;
	mLlfd = Llfd;
	mRkd = Rkd;
	mLlkd = Llkd;
	mRkq1 = Rkq1;
	mLlkq1 = Llkq1;
	mRkq2 = Rkq2;
	mLlkq2 = Llkq2;
	mH = H;
	// Additional inductances according to Krause
	mLaq = 1 / (1 / mLmq + 1 / mLl + 1 / mLlkq1 + 1 / mLlkq2);
	mLad = 1 / (1 / mLmd + 1 / mLl + 1 / mLlkd + 1 / mLlfd);

	// Determinant of Ld (inductance matrix of d axis)
	detLd = (mLmd + mLl)*(-mLlfd*mLlkd - mLlfd*mLmd - mLmd*mLlkd) + mLmd*mLmd*(mLlfd + mLlkd);
	// Determinant of Lq (inductance matrix of q axis)
	detLq = -mLmq*mLlkq2*(mLlkq1 + mLl) - mLl*mLlkq1*(mLlkq2 + mLmq);
		
}

void SynchronGenerator::init(Real om, Real dt,
	Real initActivePower, Real initReactivePower, Real initTerminalVolt, Real initVoltAngle) {

	//// Create matrices for state space representation 
	//mInductanceMat <<
	//	mLl + mLmq, 0, 0, mLmq, mLmq, 0, 0,
	//	0, mLl + mLmd, 0, 0, 0, mLmd, mLmd,
	//	0, 0, mLl, 0, 0, 0, 0,
	//	mLmq, 0, 0, mLlkq1 + mLmq, mLmq, 0, 0,
	//	mLmq, 0, 0, mLmq, mLlkq2 + mLmq, 0, 0,
	//	0, mLmd, 0, 0, 0, mLlfd + mLmd, mLmd,
	//	0, mLmd, 0, 0, 0, mLmd, mLlkd + mLmd;

	//mResistanceMat <<
	//	mRs, 0, 0, 0, 0, 0, 0,
	//	0, mRs, 0, 0, 0, 0, 0,
	//	0, 0, mRs, 0, 0, 0, 0,
	//	0, 0, 0, mRkq1, 0, 0, 0,
	//	0, 0, 0, 0, mRkq2, 0, 0,
	//	0, 0, 0, 0, 0, mRfd, 0,
	//	0, 0, 0, 0, 0, 0, mRkd;

	//mOmegaFluxMat <<
	//	0, 1, 0, 0, 0, 0, 0,
	//	-1, 0, 0, 0, 0, 0, 0,
	//	0, 0, 0, 0, 0, 0, 0,
	//	0, 0, 0, 0, 0, 0, 0,
	//	0, 0, 0, 0, 0, 0, 0,
	//	0, 0, 0, 0, 0, 0, 0,
	//	0, 0, 0, 0, 0, 0, 0;

	//mReverseCurrents <<
	//	-1, 0, 0, 0, 0, 0, 0,
	//	0, -1, 0, 0, 0, 0, 0,
	//	0, 0, -1, 0, 0, 0, 0,
	//	0, 0, 0, 1, 0, 0, 0,
	//	0, 0, 0, 0, 1, 0, 0,
	//	0, 0, 0, 0, 0, 1, 0,
	//	0, 0, 0, 0, 0, 0, 1;

	//mReactanceMat = mInductanceMat.inverse();
	//
	// steady state per unit initial value
	initStatesInPerUnit(initActivePower, initReactivePower, initTerminalVolt, initVoltAngle);
	
	//mDq0Voltages(0, 0) = mVoltages(0, 0);
	//mDq0Voltages(1, 0) = mVoltages(1, 0);
	//mDq0Voltages(2, 0) = mVoltages(2, 0);
	//mDq0Voltages = mDq0Voltages * mBase_v;
	//mAbcsVoltages = dq0ToAbcTransform(mThetaMech, mDq0Voltages);

	mVaRe = dq0ToAbcTransform2(mThetaMech, mVd* mBase_v, mVq* mBase_v, mV0* mBase_v)(0);
	mVbRe = dq0ToAbcTransform2(mThetaMech, mVd* mBase_v, mVq* mBase_v, mV0* mBase_v)(1);
	mVcRe = dq0ToAbcTransform2(mThetaMech, mVd* mBase_v, mVq* mBase_v, mV0* mBase_v)(2);
	mVaIm = dq0ToAbcTransform2(mThetaMech, mVd* mBase_v, mVq* mBase_v, mV0* mBase_v)(3);
	mVbIm = dq0ToAbcTransform2(mThetaMech, mVd* mBase_v, mVq* mBase_v, mV0* mBase_v)(4);
	mVcIm = dq0ToAbcTransform2(mThetaMech, mVd* mBase_v, mVq* mBase_v, mV0* mBase_v)(5);


	//mDq0Currents(0, 0) = mCurrents(0, 0);
	//mDq0Currents(1, 0) = mCurrents(1, 0);
	//mDq0Currents(2, 0) = mCurrents(2, 0);
	//mDq0Currents = mDq0Currents * mBase_i;
	//mAbcsCurrents = dq0ToAbcTransform(mThetaMech, mDq0Currents);

	mIaRe = dq0ToAbcTransform2(mThetaMech, mId * mBase_i, mIq * mBase_i, mI0 * mBase_i)(0);
	mIbRe = dq0ToAbcTransform2(mThetaMech, mId * mBase_i, mIq * mBase_i, mI0 * mBase_i)(1);
	mIcRe = dq0ToAbcTransform2(mThetaMech, mId * mBase_i, mIq * mBase_i, mI0 * mBase_i)(2);
	mIaIm = dq0ToAbcTransform2(mThetaMech, mId * mBase_i, mIq * mBase_i, mI0 * mBase_i)(3);
	mIbIm = dq0ToAbcTransform2(mThetaMech, mId * mBase_i, mIq * mBase_i, mI0 * mBase_i)(4);
	mIcIm = dq0ToAbcTransform2(mThetaMech, mId * mBase_i, mIq * mBase_i, mI0 * mBase_i)(5);
}

void SynchronGenerator::initStatesInPerUnit(Real initActivePower, Real initReactivePower,
	Real initTerminalVolt, Real initVoltAngle) {

	double init_P = initActivePower / mNomPower;
	double init_Q = initReactivePower / mNomPower;
	double init_S = sqrt(pow(init_P, 2.) + pow(init_Q, 2.));
	double init_vt = initTerminalVolt / mBase_v;
	double init_it = init_S / init_vt;

	// power factor
	double init_pf = acos(init_P / init_S);

	// load angle
	double init_delta = atan(((mLmq + mLl) * init_it * cos(init_pf) - mRs * init_it * sin(init_pf)) /
		(init_vt + mRs * init_it * cos(init_pf) + (mLmq + mLl) * init_it * sin(init_pf)));
	double init_delta_deg = init_delta / DPS_PI * 180;

	// dq stator voltages and currents
	double init_vd = init_vt * sin(init_delta);
	double init_vq = init_vt * cos(init_delta);
	double init_id = init_it * sin(init_delta + init_pf);
	double init_iq = init_it * cos(init_delta + init_pf);

	// rotor voltage and current
	double init_ifd = (init_vq + mRs * init_iq + (mLmd + mLl) * init_id) / mLmd;
	double init_vfd = mRfd * init_ifd;

	// flux linkages
	double init_psid = init_vq + mRs * init_iq;
	double init_psiq = -init_vd - mRs * init_id;
	double init_psifd = (mLmd + mLlfd) * init_ifd - mLmd * init_id;
	double init_psid1 = mLmd * (init_ifd - init_id);
	double init_psiq1 = -mLmq * init_iq;
	double init_psiq2 = -mLmq * init_iq;

	// rotor mechanical variables
	double init_Te = init_P + mRs * pow(init_it, 2.);
	mOmMech = 1;

	//mVoltages(0, 0) = init_vq;
	//mVoltages(1, 0) = init_vd;
	//mVoltages(2, 0) = 0;
	//mVoltages(3, 0) = 0;
	//mVoltages(4, 0) = 0;
	//mVoltages(5, 0) = init_vfd;
	//mVoltages(6, 0) = 0;

	mVd = init_vd;
	mVq = init_vq;
	mV0 = 0;
	mVfd = init_vfd;
	mVkd = 0;
	mVkq1 = 0;
	mVkq2 = 0;

	//mCurrents(0, 0) = init_iq;
	//mCurrents(1, 0) = init_id;
	//mCurrents(2, 0) = 0;
	//mCurrents(3, 0) = 0;
	//mCurrents(4, 0) = 0;
	//mCurrents(5, 0) = init_ifd;
	//mCurrents(6, 0) = 0;

	mId = init_id;
	mIq = init_iq;
	mI0 = 0;
	mIfd = init_ifd;
	mIkd = 0;
	mIkq1 = 0;
	mIkq2 = 0;

	//mFluxes(0, 0) = init_psiq;
	//mFluxes(1, 0) = init_psid;
	//mFluxes(2, 0) = 0;
	//mFluxes(3, 0) = init_psiq1;
	//mFluxes(4, 0) = init_psiq2;
	//mFluxes(5, 0) = init_psifd;
	//mFluxes(6, 0) = init_psid1;

	
	mPsid = init_psid;
	mPsiq = init_psiq;
	mPsi0 = 0;
	mPsifd = init_psifd;
	mPsikd = init_psid1;
	mPsikq1 = init_psiq1;
	mPsikq2 = init_psiq2;
	

	// Initialize mechanical angle
	mThetaMech = initVoltAngle + init_delta - PI / 2.;
}

void SynchronGenerator::step(SystemModel& system, Real fieldVoltage, Real mechPower) {

	stepInPerUnit(system.getOmega(), system.getTimeStep(), fieldVoltage, mechPower);
	
	// Update current source accordingly
	//if (mNode1 >= 0) {
	//	system.addCompToRightSideVector(mNode1, mAbcsCurrents(0, 0), mAbcsCurrents(3, 0));
	//}
	//if (mNode2 >= 0) {
	//	system.addCompToRightSideVector(mNode2, mAbcsCurrents(1, 0), mAbcsCurrents(4, 0));
	//}
	//if (mNode3 >= 0) {
	//	system.addCompToRightSideVector(mNode3, mAbcsCurrents(2, 0), mAbcsCurrents(5, 0));
	//}

	if (mNode1 >= 0) {
		system.addCompToRightSideVector(mNode1, mIaRe, mIaIm);
	}
	if (mNode2 >= 0) {
		system.addCompToRightSideVector(mNode2, mIbRe, mIbIm);
	}
	if (mNode3 >= 0) {
		system.addCompToRightSideVector(mNode3, mIcRe, mIcIm);
	}


}

void SynchronGenerator::stepInPerUnit(Real om, Real dt, Real fieldVoltage, Real mechPower) {
	// retrieve voltages
	//mAbcsVoltages = (1 / mBase_v) * mAbcsVoltages;
 //   mAbcsCurrents = (1 / mBase_i) * mAbcsCurrents;

	mVaRe = (1 / mBase_v) * mVaRe;
	mVaIm = (1 / mBase_v) * mVaIm;
	mVbRe = (1 / mBase_v) * mVbRe;
	mVbIm = (1 / mBase_v) * mVbIm;
	mVcRe = (1 / mBase_v) * mVcRe;
	mVcIm = (1 / mBase_v) * mVcIm;

	mIaRe = (1 / mBase_i) * mIaRe;
	mIaIm = (1 / mBase_i) * mIaIm;
	mIbRe = (1 / mBase_i) * mIbRe;
	mIbIm = (1 / mBase_i) * mIbIm;
	mIcRe = (1 / mBase_i) * mIcRe;
	mIcIm = (1 / mBase_i) * mIcIm;

	 //mVoltages(5, 0) = fieldVoltage / mBase_v;

	 mVfd = fieldVoltage / mBase_v;
	 //TODO calculate effect of changed field voltage

	//// dq-transform of interface voltage
	//mDq0Voltages = abcToDq0Transform(mThetaMech, mAbcsVoltages);
	//mVoltages(0, 0) = mDq0Voltages(0, 0);
	//mVoltages(1, 0) = mDq0Voltages(1, 0);
	//mVoltages(2, 0) = mDq0Voltages(2, 0);

	mVq = abcToDq0Transform2(mThetaMech, mVaRe, mVbRe, mVcRe, mVaIm, mVbIm, mVcIm)(0);
	mVd = abcToDq0Transform2(mThetaMech, mVaRe, mVbRe, mVcRe, mVaIm, mVbIm, mVcIm)(1);
	mV0 = abcToDq0Transform2(mThetaMech, mVaRe, mVbRe, mVcRe, mVaIm, mVbIm, mVcIm)(2);

	// calculate mechanical states
	mMechPower = mechPower / mNomPower;
	mMechTorque = mMechPower / mOmMech;
	//mElecTorque = (mFluxes(1, 0)*mCurrents(0, 0) - mFluxes(0, 0)*mCurrents(1, 0));
	mElecTorque = (mPsid*mIq - mPsiq*mId);

	// Euler step forward	
	mOmMech = mOmMech + dt * (1 / (2 * mH) * (mMechTorque - mElecTorque));

	//Matrix currents = mReverseCurrents * mReactanceMat * mFluxes;

	////Calculation of currents based on inverse of inductance matrix
	double Id = ((mLlfd*mLlkd + mLmd*(mLlfd + mLlkd))*mPsid - mLmd*mLlkd*mPsifd - mLlfd*mLmd*mPsikd) / detLd;
	double Ifd = (mLlkd*mLmd*mPsid - (mLl*mLlkd + mLmd*(mLl + mLlkd))*mPsifd + mLmd*mLl*mPsikd) / detLd;
	double Ikd = (mLmd*mLlfd*mPsid + mLmd*mLl*mPsifd - (mLmd*(mLlfd + mLl) + mLl*mLlfd)*mPsikd) / detLd;
	double Iq = ((mLlkq1*mLlkq2 + mLmq*(mLlkq1 + mLlkq2))*mPsiq - mLmq*mLlkq2*mPsikq1 - mLmq*mLlkq1*mPsikq2) / detLq;
	double Ikq1 = (mLmq*mLlkq2*mPsiq - (mLmq*(mLlkq2 + mLl) + mLl*mLlkq2)*mPsikq1 + mLmq*mLl*mPsikq2) / detLq;
	double Ikq2 = (mLmq*mLlkq1*mPsiq + mLmq*mLl*mPsikq1 - (mLmq*(mLlkq1 + mLl) + mLl*mLlkq1)*mPsikq2) / detLq;
	double I0 = -mPsi0 / mLl;



	//DPSMatrix dtFluxes = mVoltages - mResistanceMat * currents - mOmMech * mOmegaFluxMat * mFluxes;
	
	double dtPsid = mVd - mRs*Id + mPsiq*mOmMech;
	double dtPsiq = mVq - mRs*Iq - mPsid*mOmMech;
	double dtPsi0 = mV0 - mRs*I0;
	double dtPsifd = mVfd - mRfd*Ifd;
	double dtPsikd = -mRkd*Ikd;
	double dtPsikq1 = -mRkq1*Ikq1;
	double dtPsikq2 = -mRkq2*Ikq2;
	
	//mFluxes = mFluxes + dt * mBase_OmElec * dtFluxes;

	mPsid = mPsid + dt*mBase_OmElec*dtPsid;
	mPsiq = mPsiq + dt*mBase_OmElec*dtPsiq;
	mPsi0 = mPsi0 + dt*mBase_OmElec*dtPsi0;
	mPsifd = mPsifd + dt*mBase_OmElec*dtPsifd;
	mPsikd = mPsikd + dt*mBase_OmElec*dtPsikd;
	mPsikq1 = mPsikq1 + dt*mBase_OmElec*dtPsikq1;
	mPsikq2 = mPsikq2 + dt*mBase_OmElec*dtPsikq2;

	//mCurrents = mReverseCurrents * mReactanceMat * mFluxes;

	//Calculation of currents based on inverse of inductance matrix
	mId = ((mLlfd*mLlkd + mLmd*(mLlfd + mLlkd))*mPsid - mLmd*mLlkd*mPsifd - mLlfd*mLmd*mPsikd) / detLd;
	mIfd = (mLlkd*mLmd*mPsid - (mLl*mLlkd + mLmd*(mLl + mLlkd))*mPsifd + mLmd*mLl*mPsikd) / detLd;
	mIkd = (mLmd*mLlfd*mPsid + mLmd*mLl*mPsifd - (mLmd*(mLlfd + mLl) + mLl*mLlfd)*mPsikd) / detLd;
	mIq = ((mLlkq1*mLlkq2 + mLmq*(mLlkq1 + mLlkq2))*mPsiq - mLmq*mLlkq2*mPsikq1 - mLmq*mLlkq1*mPsikq2) / detLq;
	mIkq1 = (mLmq*mLlkq2*mPsiq - (mLmq*(mLlkq2 + mLl) + mLl*mLlkq2)*mPsikq1 + mLmq*mLl*mPsikq2) / detLq;
	mIkq2 = (mLmq*mLlkq1*mPsiq + mLmq*mLl*mPsikq1 - (mLmq*(mLlkq1 + mLl) + mLl*mLlkq1)*mPsikq2) / detLq;
	mI0 = -mPsi0 / mLl;


	//// inverse dq-transform
	//mDq0Currents(0, 0) = mCurrents(0, 0);
	//mDq0Currents(1, 0) = mCurrents(1, 0);
	//mDq0Currents(2, 0) = mCurrents(2, 0);
	//mAbcsCurrents = dq0ToAbcTransform(mThetaMech, mDq0Currents);
	//mAbcsCurrents = mBase_i * mAbcsCurrents;

	mIaRe = mBase_i * dq0ToAbcTransform2(mThetaMech, mId, mIq, mI0)(0);
	mIbRe = mBase_i * dq0ToAbcTransform2(mThetaMech, mId, mIq, mI0)(1);
	mIcRe = mBase_i * dq0ToAbcTransform2(mThetaMech, mId, mIq, mI0)(2);
	mIaIm = mBase_i * dq0ToAbcTransform2(mThetaMech, mId, mIq, mI0)(3);
	mIbIm = mBase_i * dq0ToAbcTransform2(mThetaMech, mId, mIq, mI0)(4);
	mIcIm = mBase_i * dq0ToAbcTransform2(mThetaMech, mId, mIq, mI0)(5);


	// Update mechanical rotor angle with respect to electrical angle
	mThetaMech = mThetaMech + dt * ((mOmMech - 1) * mBase_OmMech);


	mCurrents << mIq,
		mId,
		mI0,
		mIkq1,
		mIkq2,
		mIfd,
		mIkd;

	mVoltages << mVq,
		mVd,
		mV0,
		mVkq1,
		mVkq2,
		mVfd,
		mVkd;

	mFluxes << mVq,
		mPsid,
		mPsi0,
		mPsikq1,
		mPsikq2,
		mPsifd,
		mPsikd;

}

void SynchronGenerator::postStep(SystemModel& system) {
	//if (mNode1 >= 0) {
	//	mAbcsVoltages(0, 0) = system.getRealFromLeftSideVector(mNode1);
	//	mAbcsVoltages(3, 0) = system.getImagFromLeftSideVector(mNode1);
	//}
	//else {
	//	mAbcsVoltages(0, 0) = 0;
	//	mAbcsVoltages(3, 0) = 0;
	//}
	//if (mNode2 >= 0) {
	//	mAbcsVoltages(1, 0) = system.getRealFromLeftSideVector(mNode2);
	//	mAbcsVoltages(4, 0) = system.getImagFromLeftSideVector(mNode2);
	//}
	//else {
	//	mAbcsVoltages(1, 0) = 0;
	//	mAbcsVoltages(4, 0) = 0;
	//}
	//if (mNode3 >= 0) {
	//	mAbcsVoltages(2, 0) = system.getRealFromLeftSideVector(mNode3);
	//	mAbcsVoltages(5, 0) = system.getImagFromLeftSideVector(mNode3);
	//}
	//else {
	//	mAbcsVoltages(2, 0) = 0;
	//	mAbcsVoltages(5, 0) = 0;
	//}

	if (mNode1 >= 0) {
		mVaRe = system.getRealFromLeftSideVector(mNode1);
		mVaIm = system.getImagFromLeftSideVector(mNode1);
	}
	else {
		mVaRe = 0;
		mVaIm = 0;
	}
	if (mNode2 >= 0) {
		mVbRe = system.getRealFromLeftSideVector(mNode2);
		mVbIm = system.getImagFromLeftSideVector(mNode2);
	}
	else {
		mVbRe = 0;
		mVbIm = 0;
	}
	if (mNode3 >= 0) {
		mVcRe = system.getRealFromLeftSideVector(mNode3);
		mVcIm = system.getImagFromLeftSideVector(mNode3);
	}
	else {
		mVcRe = 0;
		mVcIm = 0;
	}

}

//DPSMatrix SynchronGenerator::abcToDq0Transform(Real theta, DPSMatrix& in) {
//	// Balanced case
//	Complex alpha(cos(2. / 3. * PI), sin(2. / 3. * PI));	
//	Complex thetaCompInv(cos(-theta), sin(-theta));
//	MatrixComp AbcToPnz(3, 3);
//	AbcToPnz <<
//		1, 1, 1,
//		1, alpha, pow(alpha, 2),
//		1, pow(alpha, 2), alpha;
//	AbcToPnz = (1. / 3.) * AbcToPnz;
//
//	MatrixComp abcVector(3, 1);
//	abcVector <<
//		Complex(in(0, 0), in(3, 0)),
//		Complex(in(1, 0), in(4, 0)),
//		Complex(in(2, 0), in(5, 0));
//
//	MatrixComp pnzVector(3, 1);
//	pnzVector = AbcToPnz * abcVector * thetaCompInv;
//
//	DPSMatrix dq0Vector(3, 1);
//	dq0Vector <<
//		pnzVector(1, 0).imag(),
//		pnzVector(1, 0).real(),
//		0;
//	
//	return dq0Vector;
//}
//
//DPSMatrix SynchronGenerator::dq0ToAbcTransform(Real theta, DPSMatrix& in) {
//	// Balanced case
//	Complex alpha(cos(2. / 3. * PI), sin(2. / 3. * PI));
//	Complex thetaComp(cos(theta), sin(theta));
//	MatrixComp PnzToAbc(3, 3);
//	PnzToAbc <<
//		1, 1, 1,
//		1, pow(alpha, 2), alpha,
//		1, alpha, pow(alpha, 2);
//
//	MatrixComp pnzVector(3, 1);
//	pnzVector <<
//		0,
//		Complex(in(1, 0), in(0, 0)),
//		Complex(0, 0);
//
//	MatrixComp abcCompVector(3, 1);
//	abcCompVector = PnzToAbc * pnzVector * thetaComp;
//	
//	Matrix abcVector(6, 1);
//	abcVector <<
//		abcCompVector(0, 0).real(),
//		abcCompVector(1, 0).real(),
//		abcCompVector(2, 0).real(),
//		abcCompVector(0, 0).imag(),
//		abcCompVector(1, 0).imag(),
//		abcCompVector(2, 0).imag();
//
//	return abcVector;
//}

DPSMatrix SynchronGenerator::abcToDq0Transform2(Real theta, Real aRe, Real bRe, Real cRe, Real aIm, Real bIm, Real cIm) {
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

	DPSMatrix dq0Vector(3, 1);
	dq0Vector <<
		pnzVector(1, 0).imag(),
		pnzVector(1, 0).real(),
		0;

	return dq0Vector;
}

DPSMatrix SynchronGenerator::dq0ToAbcTransform2(Real theta, Real d, Real q, Real zero) {
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