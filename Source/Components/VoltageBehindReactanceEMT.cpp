#include "VoltageBehindReactanceEMT.h"

using namespace DPsim;

VoltageBehindReactanceEMT::VoltageBehindReactanceEMT(std::string name, int node1, int node2, int node3,
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

	// steady state per unit initial value
	initWithPerUnitParam(Rs, Ll, Lmd, Lmd0, Lmq, Lmq0, Rfd, Llfd, Rkd, Llkd, Rkq1, Llkq1, Rkq2, Llkq2, inertia);
	
}
void VoltageBehindReactanceEMT::initWithPerUnitParam(
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
		

	//Dynamic mutual inductances
	mDLmd = 1 / (1 / mLmd + 1 / mLlfd + 1 / mLlkd);
	mDLmq = 1 / (1 / mLmq + 1 / mLlkq1 + 1 / mLlkq2);

	mLa = (mDLmq + mDLmd) / 3;
	mLb = (mDLmd - mDLmq) / 3;
	


}


void VoltageBehindReactanceEMT::init(Real om, Real dt,
	Real initActivePower, Real initReactivePower, Real initTerminalVolt, Real initVoltAngle) {

	mResistanceMat <<
		mRs, 0, 0,
		0, mRs, 0,
		0, 0, mRs;

	// steady state per unit initial value
	initStatesInPerUnit(initActivePower, initReactivePower, initTerminalVolt, initVoltAngle);

	mVa = inverseParkTransform(mThetaMech, mVq, mVd, mV0)(0);
	mVb = inverseParkTransform(mThetaMech, mVq, mVd, mV0)(1);
	mVc = inverseParkTransform(mThetaMech, mVq, mVd, mV0)(2);

	mIa = inverseParkTransform(mThetaMech, mIq, mId, mI0)(0);
	mIb = inverseParkTransform(mThetaMech, mIq, mId, mI0)(1);
	mIc = inverseParkTransform(mThetaMech, mIq, mId, mI0)(2);
}

void VoltageBehindReactanceEMT::initStatesInPerUnit(Real initActivePower, Real initReactivePower,
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


	mIq = init_iq;
	mId = init_id;
	mI0 = 0;
	mIfd = init_ifd;
	mIkd = 0;
	mIkq1 = 0;
	mIkq2 = 0;

	mVd = init_vd;
	mVq = init_vq;
	mV0 = 0;
	mVfd = init_vfd;
	mVkd = 0;
	mVkq1 = 0;
	mVkq2 = 0;

	mPsiq = init_psiq;
	mPsid = init_psid;
	mPsi0 = 0;
	mPsifd = init_psifd;
	mPsikd = init_psid1;
	mPsikq1 = init_psiq1;
	mPsikq2 = init_psiq2;

	mDPsiq = mDLmq*(mPsikq1 / mLlkq1) + mDLmq*(mPsikq2 / mLlkq2);
	mDPsid = mDLmd*(mPsifd / mLlfd) + mDLmd*(mPsikd / mLlkd);

	mDVq = mOmMech*mDPsid + mDLmq*mRkq1*(mDPsiq - mPsikq1) / (mLlkq1*mLlkq1) +
		mDLmq*mRkq2*(mDPsiq - mPsikq2) / (mLlkq2*mLlkq2) + (mRkq1 / (mLlkq1*mLlkq1) + mRkq2 / (mLlkq2*mLlkq2))*mDLmq*mDLmq*mIq;
	mDVd = -mOmMech*mDPsiq + mDLmd*mRkd*(mDPsid - mPsikd) / (mLlkd*mLlkd) + (mDLmd / mLlfd)*mVfd +
		mDLmd*mRfd*(mDPsid - mPsifd) / (mLlfd*mLlfd) + (mRfd / (mLlfd*mLlfd) + mRkd / (mLlkd*mLlkd))*mDLmd*mDLmd*mId;

	// Initialize mechanical angle
	mThetaMech = initVoltAngle + init_delta - M_PI / 2;

	mDVa = inverseParkTransform(mThetaMech, mDVq, mDVd, 0)(0);
	mDVb = inverseParkTransform(mThetaMech, mDVq, mDVd, 0)(1);
	mDVc = inverseParkTransform(mThetaMech, mDVq, mDVd, 0)(2);

	//Initial inductance matrix
	mDInductanceMat <<
	mLl + mLa - mLb*cos(2 * mThetaMech), -mLa / 2 - mLb*cos(2 * mThetaMech - 2 * PI / 3), -mLa / 2 - mLb*cos(2 * mThetaMech + 2 * PI / 3),
	-mLa / 2 - mLb*cos(2 * mThetaMech - 2 * PI / 3), mLl + mLa - mLb*cos(2 * mThetaMech - 4 * PI / 3), -mLa / 2 - mLb*cos(2 * mThetaMech),
	-mLa / 2 - mLb*cos(2 * mThetaMech + 2 * PI / 3), -mLa / 2 - mLb*cos(2 * mThetaMech), mLl + mLa - mLb*cos(2 * mThetaMech + 4 * PI / 3);

}


void VoltageBehindReactanceEMT::step(SystemModel& system, Real fieldVoltage, Real mechPower, Real time) {

	stepInPerUnit(system.getOmega(), system.getTimeStep(), fieldVoltage, mechPower, time, system.getNumMethod());

	mVa = mVabc(0);
	mVb = mVabc(1);
	mVc = mVabc(2);

	mIa = (mVa / 1037.8378)*mBase_Z;
	mIb = (mVb / 1037.8378)*mBase_Z;
	mIc = (mVc / 1037.8378)*mBase_Z;

}

void VoltageBehindReactanceEMT::stepInPerUnit(Real om, Real dt, Real fieldVoltage, Real mechPower, Real time, NumericalMethod numMethod) {
	
	mVabc <<
		mVa,
		mVb,
		mVc;

	mIabc <<
		mIa,
		mIb,
		mIc;

	mDVabc <<
		mDVa,
		mDVb,
		mDVc;


	mV_hist = (mResistanceMat - (2. / dt)*mDInductanceMat)*mIabc + mDVabc - mVabc;

	mId = parkTransform(mThetaMech, mIa, mIb, mIc)(0);
	mIq = parkTransform(mThetaMech, mIa, mIb, mIc)(1);
	mI0 = parkTransform(mThetaMech, mIa, mIb, mIc)(2);	

	Real mPsimq = mDLmq*(mPsikq1 / mLlkq1 + mPsikq2 / mLlkq2 + mIq);
	Real mPsimd = mDLmd*(mPsifd / mLlfd + mPsikd / mLlkd + mId);

	mPsikq1 = mPsikq1 - dt*mBase_OmElec*(mRkq1 / mLlkq1)*(mPsikq1 - mPsimq);
	mPsikq2 = mPsikq2 - dt*mBase_OmElec*(mRkq2 / mLlkq2)*(mPsikq2 - mPsimq);
	mPsifd = mPsifd - dt*mBase_OmElec*((mRfd / mLlfd)*(mPsifd - mPsimd) + mVfd);
	mPsikd = mPsikd - dt*mBase_OmElec*(mRkd / mLlkd)*(mPsikd - mPsimd);

	// Calculate dynamic flux likages

	mDPsiq = mDLmq*(mPsikq1 / mLlkq1) + mDLmq*(mPsikq2 / mLlkq2);
	mDPsid = mDLmd*(mPsifd / mLlfd) + mDLmd*(mPsikd / mLlkd);
		
	mDVq = mOmMech*mDPsid + mDLmq*mRkq1*(mDPsiq - mPsikq1) / (mLlkq1*mLlkq1) +
		mDLmq*mRkq2*(mDPsiq - mPsikq2) / (mLlkq2*mLlkq2) + (mRkq1 / (mLlkq1*mLlkq1) + mRkq2 / (mLlkq2*mLlkq2))*mDLmq*mDLmq*mIq;
	mDVd = -mOmMech*mDPsiq + mDLmd*mRkd*(mDPsid - mPsikd) / (mLlkd*mLlkd) + (mDLmd / mLlfd)*mVfd +
		mDLmd*mRfd*(mDPsid - mPsifd) / (mLlfd*mLlfd) + (mRfd / (mLlfd*mLlfd) + mRkd / (mLlkd*mLlkd))*mDLmd*mDLmd*mId;

	mDVa = inverseParkTransform(mThetaMech, mDVq, mDVd, 0)(0);
	mDVb = inverseParkTransform(mThetaMech, mDVq, mDVd, 0)(1);
	mDVc = inverseParkTransform(mThetaMech, mDVq, mDVd, 0)(2);

	mThetaMech = mThetaMech + dt * (mOmMech* mBase_OmMech);
	//mThetaMech_hist2 = mThetaMech_hist1;
	//mThetaMech_hist1 = mThetaMech;
	//mThetaMech = 2 * mThetaMech - mThetaMech_hist2;

	//mOmMech_hist2 = mOmMech_hist1;
	//mOmMech_hist1 = mOmMech;
	//mOmMech = 2 * mOmMech - mOmMech_hist2;

	mVabc = (mResistanceMat + (2 / dt)*mDInductanceMat)*mIabc + mDVabc + mV_hist;

}

void VoltageBehindReactanceEMT::postStep(SystemModel& system) {
	

}


DPSMatrix VoltageBehindReactanceEMT::parkTransform(Real theta, double a, double b, double c) {

	DPSMatrix dq0vector(3, 1);

	// Park transform according to Kundur
	double q, d, zero;

	q = 2. / 3. * cos(theta) * a + 2. / 3. * cos(theta - 2. * M_PI / 3.)*b + 2. / 3. * cos(theta + 2. * M_PI / 3.)*c;
	d = 2. / 3. * sin(theta)*a + 2. / 3. * sin(theta - 2. * M_PI / 3.)*b + 2. / 3. * sin(theta + 2. * M_PI / 3.)*c;
	zero = 1. / 3. * a, 1. / 3. * b, 1. / 3. * c;

	dq0vector << q,
		d,
		0;

	return dq0vector;
}





DPSMatrix VoltageBehindReactanceEMT::inverseParkTransform(Real theta, double q, double d, double zero) {

	DPSMatrix abcVector(3, 1);

	double a, b, c;

	// Park transform according to Kundur
	a = cos(theta)*d + sin(theta)*q + 1.*zero;
	b = cos(theta - 2. * M_PI / 3.)*d + sin(theta - 2. * M_PI / 3.)*q + 1.*zero;
	c = cos(theta + 2. * M_PI / 3.)*d + sin(theta + 2. * M_PI / 3.)*q + 1.*zero;

	abcVector << a,
		b,
		c;

	return abcVector;
}