#include "SynchronGeneratorEMT.h"

using namespace DPsim;

SynchronGeneratorEMT::SynchronGeneratorEMT(std::string name, int node1, int node2, int node3,
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

void SynchronGeneratorEMT::initWithPerUnitParam(
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



	

}

void SynchronGeneratorEMT::init(Real om, Real dt,
	Real initActivePower, Real initReactivePower, Real initTerminalVolt, Real initVoltAngle) {

	// Create matrices for state space representation 
	mInductanceMat << 
		mLl + mLmq, 0, 0, mLmq, mLmq, 0, 0,
		0, mLl + mLmd, 0, 0, 0, mLmd, mLmd,
		0, 0, mLl, 0, 0, 0, 0,
		mLmq, 0, 0, mLlkq1 + mLmq, mLmq, 0, 0,
		mLmq, 0, 0, mLmq, mLlkq2 + mLmq, 0, 0,
		0, mLmd, 0, 0, 0, mLlfd + mLmd, mLmd,
		0, mLmd, 0, 0, 0, mLmd, mLlkd + mLmd;

	mResistanceMat << 
		mRs, 0, 0, 0, 0, 0, 0,
		0, mRs, 0, 0, 0, 0, 0,
		0, 0, mRs, 0, 0, 0, 0,
		0, 0, 0, mRkq1, 0, 0, 0,
		0, 0, 0, 0, mRkq2, 0, 0,
		0, 0, 0, 0, 0, mRfd, 0,
		0, 0, 0, 0, 0, 0, mRkd;

	mOmegaFluxMat << 
		0, 1, 0, 0, 0, 0, 0,
		-1, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0;

	mReverseCurrents <<	
		-1, 0, 0, 0, 0, 0, 0,
		0, -1, 0, 0, 0, 0, 0,
		0, 0, -1, 0, 0, 0, 0,
		0, 0, 0, 1, 0, 0, 0,
		0, 0, 0, 0, 1, 0, 0,
		0, 0, 0, 0, 0, 1, 0,
		0, 0, 0, 0, 0, 0, 1;

	mReactanceMat = mInductanceMat.inverse();
	
	// steady state per unit initial value
	initStatesInPerUnit(initActivePower, initReactivePower, initTerminalVolt, initVoltAngle);
	

	// I STOPED HERE VERIFYING THE RESULTS MATRIX x VARIABBLES - IT IS NOT WORKING YET!!!
	mDq0Voltages(0, 0) = mVoltages(0, 0);
	mDq0Voltages(1, 0) = mVoltages(1, 0);
	mDq0Voltages(2, 0) = mVoltages(2, 0);	
	mDq0Voltages = mDq0Voltages * mBase_v;
	mAbcsVoltages = inverseParkTransform(mThetaMech, mDq0Voltages);

	mVa = inverseParkTransform2(mThetaMech, mVd, mVq, mV0)(0);
	mVb = inverseParkTransform2(mThetaMech, mVd, mVq, mV0)(1);
	mVc = inverseParkTransform2(mThetaMech, mVd, mVq, mV0)(2);

	mDq0Currents(0, 0) = mCurrents(0, 0);
	mDq0Currents(1, 0) = mCurrents(1, 0);
	mDq0Currents(2, 0) = mCurrents(2, 0);
	mDq0Currents = mDq0Currents * mBase_i;
	mAbcsCurrents = inverseParkTransform(mThetaMech, mDq0Currents);
}

void SynchronGeneratorEMT::initStatesInPerUnit(Real initActivePower, Real initReactivePower,
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

	mVoltages(0, 0) = init_vq;
	mVoltages(1, 0) = init_vd;
	mVoltages(2, 0) = 0;
	mVoltages(3, 0) = 0;
	mVoltages(4, 0) = 0;
	mVoltages(5, 0) = init_vfd;
	mVoltages(6, 0) = 0;


	mCurrents(0, 0) = init_iq;
	mCurrents(1, 0) = init_id;
	mCurrents(2, 0) = 0;
	mCurrents(3, 0) = 0;
	mCurrents(4, 0) = 0;
	mCurrents(5, 0) = init_ifd;
	mCurrents(6, 0) = 0;

	mFluxes(0, 0) = init_psiq;
	mFluxes(1, 0) = init_psid;
	mFluxes(2, 0) = 0;
	mFluxes(3, 0) = init_psiq1;
	mFluxes(4, 0) = init_psiq2;
	mFluxes(5, 0) = init_psifd;
	mFluxes(6, 0) = init_psid1;

	mVq = init_vq;
	mVd = init_vd;
	mV0 = 0;
	mVfd = init_vfd;
	mVkd = 0;
	mVkq1 = 0;
	mVkq2 = 0;

	mIq = init_iq;
	mId = init_id;
	mI0 = 0;
	mIfd = init_ifd;
	mIkd = 0;
	mIkq1 = 0;
	mIkq2 = 0;

	mPsiq = init_psiq;
	mPsid = init_psid;
	mPsi0 = 0;
	mPsifd = init_psifd;
	mPsikd = init_psid1;
	mPsikq1 = init_psiq1;
	mPsikq2 = init_psiq2;

	// Initialize mechanical angle
	//mThetaMech = initVoltAngle + init_delta;
	mThetaMech = initVoltAngle + init_delta - M_PI/90;
}

void SynchronGeneratorEMT::step(SystemModel& system, Real fieldVoltage, Real mechPower) {

	stepInPerUnit(system.getOmega(), system.getTimeStep(), fieldVoltage, mechPower);
	
	// Update current source accordingly
	if (mNode1 >= 0) {
		system.addRealToRightSideVector(mNode1, mIa);
	}
	if (mNode2 >= 0) {
		system.addRealToRightSideVector(mNode2, mIb);
	}
	if (mNode3 >= 0) {
		system.addRealToRightSideVector(mNode3, mIc);
	}
}

void SynchronGeneratorEMT::stepInPerUnit(Real om, Real dt, Real fieldVoltage, Real mechPower) {
	
	// retrieve voltages
	mAbcsVoltages = (1 / mBase_v) * mAbcsVoltages;
	mAbcsCurrents = (1 / mBase_i) * mAbcsCurrents;


	mVa = (1 / mBase_v) * mVa;
	mVb = (1 / mBase_v) * mVb;
	mVc = (1 / mBase_v) * mVc;

	mIa = (1 / mBase_i) * mIa;
	mIb = (1 / mBase_i) * mIb;
	mIc = (1 / mBase_i) * mIc;

	mVoltages(5, 0) = fieldVoltage / mBase_v;
	// TODO calculate effect of changed field voltage

	// dq-transform of interface voltage
	mVd = parkTransform2(mThetaMech, mVa, mVb, mVc)(0);
	mVq = parkTransform2(mThetaMech, mVa, mVb, mVc)(1);
	mV0 = parkTransform2(mThetaMech, mVa, mVb, mVc)(2);

	mDq0Voltages = parkTransform(mThetaMech, mAbcsVoltages);
	mVoltages(0, 0) = mDq0Voltages(0, 0);
	mVoltages(1, 0) = mDq0Voltages(1, 0);
	mVoltages(2, 0) = mDq0Voltages(2, 0);

	// calculate mechanical states
	mMechPower = mechPower / mNomPower;
	mMechTorque = mMechPower / mOmMech;
	mElecTorque = (mPsid*mIq - mPsiq*mId);
	//mElecTorque = (mFluxes(1, 0)*mCurrents(0, 0) - mFluxes(0, 0)*mCurrents(1, 0));

	// Euler step forward	
	mOmMech = mOmMech + dt * (1 / (2 * mH) * (mMechTorque - mElecTorque));


	// Determinant of Ld_eq (inductance matrix of d axis)
	double detLd = (mLmd + mLl)*(-mLlfd*mLlkd - mLlfd*mLmd - mLmd*mLlkd) + mLmd*mLmd*(mLlfd + mLlkd);

	//Calculation of currents based on inverse of inductance matrix
	mId = ((mLlfd*mLlkd + mLmd*(mLlfd + mLlkd))*mPsid - mLmd*mLlkd*mPsifd - mLlfd*mLmd*mPsikd) / detLd;
	mIfd = (mLlkd*mLmd*mPsid - (mLl*mLlkd + mLmd*(mLl + mLlkd))*mPsifd + mLmd*mLl) / detLd;
	mIkd = (mLmd*mLlfd*mPsid + mLmd*mLl*mPsifd - (mLmd*(mLlfd + mLl) + mLl*mLlfd)*mPsikd) / detLd;



	DPSMatrix currents = mReverseCurrents * mReactanceMat * mFluxes;


	DPSMatrix dtFluxes = mVoltages - mResistanceMat * currents - mOmMech * mOmegaFluxMat * mFluxes;
	
	for (int i = 0; i < dtFluxes.size(); i++)
	{
		if (dtFluxes(i, 0) < 0.000001)
			dtFluxes(i, 0) = 0;
	}
	mFluxes = mFluxes + dt * mBase_OmElec * dtFluxes;
	
	mCurrents = mReverseCurrents * mReactanceMat * mFluxes;

	// Update mechanical rotor angle with respect to electrical angle
	mThetaMech = mThetaMech + dt * (mOmMech * mBase_OmMech);

	// inverse dq-transform
	mDq0Currents(0, 0) = mCurrents(0, 0);
	mDq0Currents(1, 0) = mCurrents(1, 0);
	mDq0Currents(2, 0) = mCurrents(2, 0);
	mAbcsCurrents = inverseParkTransform(mThetaMech, mDq0Currents);
	mAbcsCurrents = mBase_i * mAbcsCurrents;
}

void SynchronGeneratorEMT::postStep(SystemModel& system) {
	if (mNode1 >= 0) {
		mAbcsVoltages(0,0) = system.getRealFromLeftSideVector(mNode1);		
	}
	else {
		mAbcsVoltages(0, 0) = 0;		
	}
	if (mNode2 >= 0) {
		mAbcsVoltages(1, 0) = system.getRealFromLeftSideVector(mNode2);
	}
	else {
		mAbcsVoltages(1, 0) = 0;		
	}
	if (mNode3 >= 0) {
		mAbcsVoltages(2, 0) = system.getRealFromLeftSideVector(mNode3);
	}
	else {
		mAbcsVoltages(2, 0) = 0;
	}
}

DPSMatrix SynchronGeneratorEMT::parkTransform(Real theta, DPSMatrix& in) {
	DPSMatrix ParkMat(3,3);
	// Park transform according to Krause
	//ParkMat << 
	//	2. / 3. * cos(theta), 2. / 3. * cos(theta - 2. * M_PI / 3.), 2. / 3. * cos(theta + 2. * M_PI / 3.),
	//	2. / 3. * sin(theta), 2. / 3. * sin(theta - 2. * M_PI / 3.), 2. / 3. * sin(theta + 2. * M_PI / 3.),
	//	1. / 3., 1. / 3., 1. / 3.;

	// Park transform according to Kundur
	 ParkMat << 2. / 3. * cos(theta), 2. / 3. * cos(theta - 2. * M_PI / 3.), 2. / 3. * cos(theta + 2. * M_PI / 3.),
		- 2. / 3. * sin(theta), - 2. / 3. * sin(theta - 2. * M_PI / 3.), - 2. / 3. * sin(theta + 2. * M_PI / 3.),
		1. / 3., 1. / 3., 1. / 3.;

	return ParkMat * in;
}

DPSMatrix SynchronGeneratorEMT::parkTransform2(Real theta, double a, double b, double c) {
	DPSMatrix ParkMat(3, 1);
	// Park transform according to Krause
	//ParkMat << 
	//	2. / 3. * cos(theta), 2. / 3. * cos(theta - 2. * M_PI / 3.), 2. / 3. * cos(theta + 2. * M_PI / 3.),
	//	2. / 3. * sin(theta), 2. / 3. * sin(theta - 2. * M_PI / 3.), 2. / 3. * sin(theta + 2. * M_PI / 3.),
	//	1. / 3., 1. / 3., 1. / 3.;

	// Park transform according to Kundur
	double d, q, zero;

	d = 2. / 3. * cos(theta) * a + 2. / 3. * cos(theta - 2. * M_PI / 3.)*b + 2. / 3. * cos(theta + 2. * M_PI / 3.)*c;
	q = -2. / 3. * sin(theta)*a - 2. / 3. * sin(theta - 2. * M_PI / 3.)*b - 2. / 3. * sin(theta + 2. * M_PI / 3.)*c;
	zero = 1. / 3. * a, 1. / 3. * b, 1. / 3. * c;

	ParkMat << d,
		q,
		0;

	return ParkMat;
}

DPSMatrix SynchronGeneratorEMT::inverseParkTransform(Real theta, DPSMatrix& in) {
	DPSMatrix InverseParkMat(3,3);
	//// Park transform according to Krause
	//InverseParkMat << 
	//	cos(theta), sin(theta), 1,
	//	cos(theta - 2. * M_PI / 3.), sin(theta - 2. * M_PI / 3.), 1,
	//	cos(theta + 2. * M_PI / 3.), sin(theta + 2. * M_PI / 3.), 1;

	// Park transform according to Krause
	InverseParkMat <<
		cos(theta), -sin(theta), 1. / sqrt(3),
		cos(theta - 2. * M_PI / 3.), -sin(theta - 2. * M_PI / 3.), 1. / sqrt(3),
		cos(theta + 2. * M_PI / 3.), -sin(theta + 2. * M_PI / 3.), 1. / sqrt(3);

	return InverseParkMat * in;
}

DPSMatrix SynchronGeneratorEMT::inverseParkTransform2(Real theta, double d, double q, double zero) {
	DPSMatrix InverseParkMat(3, 1);

	double a, b, c;

		// Park transform according to Krause
	a = cos(theta)*d - sin(theta)*q + (1. / sqrt(3))*zero;
	b = cos(theta - 2. * M_PI / 3.)*d - sin(theta - 2. * M_PI / 3.)*q + (1. / sqrt(3))*zero;
	c=	cos(theta + 2. * M_PI / 3.)*d - sin(theta + 2. * M_PI / 3.)*q + (1. / sqrt(3))*zero;

	InverseParkMat << a,
		b,
		c;

	return InverseParkMat;
}