#include "SynchronousGenerator.h"

SynchronousGenerator::SynchronousGenerator(std::string name, int node1, int node2, int node3, 
	bool usePerUnit, double nomPower, double nomVolt, double nomFreq, double statorRes, double leakInd, double mutInd_d, double mutInd_q, 
	double fieldRes, double fieldLeakInd, double dampRes_d, double dampLeakInd_d, double dampRes1_q, double dampLeakInd1_q,
	double dampRes2_q, double dampLeakInd2_q, double inertia, int poleNumber) {

	this->node1 = node1 - 1;
	this->node2 = node2 - 1;
	this->node3 = node3 - 1;

	mUsePerUnit = usePerUnit;
	mNomPower = nomPower;
	mNomVolt = nomVolt;
	mNomFreq = nomFreq;
	mPoleNumber = poleNumber;
	mInertia = inertia;

	mBaseVoltage = mNomVolt * sqrt(2) / sqrt(3);
	mBaseCurrent = mNomPower * sqrt(2) / (mNomVolt * sqrt(3));
	mBaseImpedance = mBaseVoltage / mBaseCurrent;
	mBaseAngFreq = 2 * DPS_PI * mNomFreq;
	mBaseInductance = mBaseImpedance / mBaseAngFreq;	
	mBaseTorque = mNomPower * mPoleNumber / (2 * mBaseAngFreq);
	double test = pow((2. / mPoleNumber), 2);
	mInertiaCoeff = 0.5 * pow((2. / mPoleNumber), 2) * mInertia * pow(mBaseAngFreq, 2) / mNomPower;
	mOmega_r = 2 * DPS_PI * mNomFreq;

	if (mUsePerUnit) {
		mRs = statorRes / mBaseImpedance;
		mLl = leakInd / mBaseInductance;
		mLmd = mutInd_d / mBaseInductance;
		mLmq = mutInd_q / mBaseInductance;
		mRf = fieldRes / mBaseImpedance;
		mLlkd = dampLeakInd_d / mBaseInductance;
		mLlfd = fieldLeakInd / mBaseInductance;
		mRkd = dampRes_d / mBaseImpedance;
		mRkq1 = dampRes1_q / mBaseImpedance;
		mRkq2 = dampRes2_q / mBaseImpedance;
		mLlkq1 = dampLeakInd1_q / mBaseInductance;
		mLlkq2 = dampLeakInd2_q / mBaseInductance;
			
	} else {
		mRs = statorRes;
		mLl = leakInd;
		mLmd = mutInd_d;
		mLmq = mutInd_q;
		mRf = fieldRes;
		mLlkd = dampLeakInd_d;
		mLlfd = fieldLeakInd;
		mRkd = dampRes_d;
		mRkq1 = dampRes1_q;
		mRkq2 = dampRes2_q;
		mLlkq1 = dampLeakInd1_q;
		mLlkq2 = dampLeakInd2_q;	
	}
	
	mLaq = 1 / (1 / mLmq + 1 / mLl + 1 / mLlkq1 + 1 / mLlkq2);
	mLad = 1 / (1 / mLmd + 1 / mLl + 1 / mLlkd + 1 / mLlfd);
}

void SynchronousGenerator::applyMatrixStamp(DPSMatrix& g, DPSMatrix& j, int compOffset, double om, double dt) {
	// Compensated current source
}

void SynchronousGenerator::Init(DPSMatrix& g, DPSMatrix& j, int compOffset, double om, double dt,
	DPSMatrix initAbcsCurrents, DPSMatrix initAbcsVoltages, double initFieldVoltage, double initFieldCurrent, double initTheta_r) {

	// Create matrices for state space representation 
	mInductanceMat << mLl + mLmq, 0, 0, mLmq, mLmq, 0, 0,
		0, mLl + mLmd, 0, 0, 0, mLmd, mLmd,
		0, 0, mLl, 0, 0, 0, 0,
		mLmq, 0, 0, mLlkq1 + mLmq, mLmq, 0, 0,
		mLmq, 0, 0, mLmq, mLlkq2 + mLmq, 0, 0,
		0, mLmd, 0, 0, 0, mLlfd + mLmd, mLmd,
		0, mLmd, 0, 0, 0, mLmd, mLlkd + mLmd;

	mResistanceMat << mRs, 0, 0, 0, 0, 0, 0,
		0, mRs, 0, 0, 0, 0, 0,
		0, 0, mRs, 0, 0, 0, 0,
		0, 0, 0, mRkq1, 0, 0, 0,
		0, 0, 0, 0, mRkq2, 0, 0,
		0, 0, 0, 0, 0, mRf, 0,
		0, 0, 0, 0, 0, 0, mRkd;

	mOmegaFluxMat << 0, 1, 0, 0, 0, 0, 0,
		-1, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0;

	mReactanceMat = mInductanceMat.inverse();

	// Initialize currents, fluxes, voltages and angle	
	mTheta_r = initTheta_r;
	mAbcsCurrents = initAbcsCurrents;
	mAbcsVoltages = initAbcsVoltages;
	mVoltages(5, 0) = initFieldVoltage;
	mCurrents(5, 0) = initFieldCurrent;

	if (mUsePerUnit) {
		mAbcsCurrents = (1 / mBaseCurrent) * mAbcsCurrents;
		mAbcsVoltages = (1 / mBaseVoltage) * mAbcsVoltages;
		mVoltages(5, 0) = mVoltages(5, 0) / mBaseVoltage;
		mCurrents(5, 0) = mCurrents(5, 0) / mBaseCurrent;
	}

	mDq0Currents = ParkTransform(mTheta_r, mAbcsCurrents);
	mCurrents(0, 0) = -mDq0Currents(0, 0);
	mCurrents(1, 0) = -mDq0Currents(1, 0);
	mCurrents(2, 0) = -mDq0Currents(2, 0);

	mFluxes = mInductanceMat * mCurrents;
	double imq = (mFluxes(0, 0) / mLl + mFluxes(3, 0) / mLlkq1 + mFluxes(4, 0) / mLlkq2) * mLaq;
	double imd = (mFluxes(1, 0) / mLl + mFluxes(5, 0) / mLlfd + mFluxes(6, 0) / mLlkd) * mLad;

	mDq0Voltages = ParkTransform(mTheta_r, mAbcsVoltages);
	mVoltages(0, 0) = mDq0Voltages(0, 0);
	mVoltages(1, 0) = mDq0Voltages(1, 0);
	mVoltages(2, 0) = mDq0Voltages(2, 0);

	if (mUsePerUnit) {
		mAbcsCurrents = initAbcsCurrents;
		mAbcsVoltages = initAbcsVoltages;
	}

	DPSMatrix correctedVoltages = mOmega_r * mOmegaFluxMat * mFluxes + mResistanceMat * mCurrents;
	DPSMatrix test1 = mVoltages;
	DPSMatrix test2 = mOmega_r * mOmegaFluxMat * mFluxes;
	DPSMatrix test3 = mResistanceMat * mCurrents;//mReactanceMat * mFluxes;
	mVoltages(1,0) = correctedVoltages(1,0);
	mDq0Voltages(0, 0) = mVoltages(0, 0);
	mDq0Voltages(1, 0) = mVoltages(1, 0);
	mDq0Voltages(2, 0) = mVoltages(2, 0);
	mAbcsVoltages = InverseParkTransform(mTheta_r, mDq0Voltages);
	double testvolt4 = mAbcsCurrents(0, 0) * mBaseImpedance * 1000;
	double testvolt5 = mAbcsCurrents(1, 0) * mBaseImpedance * 1000;
	double testvolt6 = mAbcsCurrents(2, 0) * mBaseImpedance * 1000;
	double phimq = 0;
	double phimd = 0;

}

/// Performs an Euler forward step with the state space model of a synchronous generator 
/// to calculate the flux and current from the voltage vector.
void SynchronousGenerator::Step(DPSMatrix& g, DPSMatrix& j, int compOffset, double om, double dt, double t,
	double fieldVoltage, double fieldCurrent, double mechPower) {	

	if (mUsePerUnit) {
		// retrieve voltages
		mAbcsVoltages = (1 / mBaseVoltage) * mAbcsVoltages;
		mAbcsCurrents = (1 / mBaseCurrent) * mAbcsCurrents;
		mVoltages(5, 0) = fieldVoltage / mBaseVoltage;
		mCurrents(5, 0) = fieldCurrent / mBaseCurrent;

		// dq-transform of interface voltage
		mDq0Voltages = ParkTransform(mTheta_r, mAbcsVoltages);
		mVoltages(0, 0) = mDq0Voltages(0, 0);
		mVoltages(1, 0) = mDq0Voltages(1, 0);
		mVoltages(2, 0) = mDq0Voltages(2, 0);

		// calculate mechanical states
		mMechPower = mechPower / mNomPower;
		mMechTorque = mechPower / mOmega_r / mBaseTorque;			
		mElecTorque = mBaseAngFreq * (mFluxes(1, 0)*mCurrents(0, 0) - mFluxes(0, 0)*mCurrents(1, 0));		

		// Euler step forward
		mTheta_r = mTheta_r + dt * mOmega_r;
		mOmega_r = mOmega_r + dt * (mBaseAngFreq / (2 * mInertiaCoeff) * ( mMechTorque - mElecTorque ) );		
		//mFluxes = mFluxes + dt * (mVoltages - mResistanceMat * mReactanceMat * mFluxes - mOmega_r * mOmegaFluxMat * mFluxes);

		DPSMatrix test = mVoltages - mResistanceMat * mReactanceMat * mFluxes - mOmega_r / mBaseAngFreq * mOmegaFluxMat * mFluxes;

		// inverse dq-transform
		mCurrents = mReactanceMat * mFluxes;
		mDq0Currents(0, 0) = mCurrents(0, 0);
		mDq0Currents(1, 0) = mCurrents(1, 0);
		mDq0Currents(2, 0) = mCurrents(2, 0);
		mAbcsCurrents = InverseParkTransform(mTheta_r, mDq0Currents);
		mAbcsCurrents = mBaseCurrent * mAbcsCurrents;
	}
	else {
		// retrieve voltages 
		mVoltages(5, 0) = fieldVoltage;
		mCurrents(5, 0) = fieldCurrent;

		// dq-transform of interface voltage
		mDq0Voltages = ParkTransform(mTheta_r, mAbcsVoltages);
		mVoltages(0, 0) = mDq0Voltages(0, 0);
		mVoltages(1, 0) = mDq0Voltages(1, 0);
		mVoltages(2, 0) = mDq0Voltages(2, 0);

		// calculate mechanical states
		mMechPower = mechPower;
		mMechTorque = mechPower / mOmega_r;
		
		// Euler step forward
		DPSMatrix test1 = mVoltages;
		DPSMatrix test2 = mOmega_r * mOmegaFluxMat * mFluxes;
		DPSMatrix test3 = mResistanceMat * mCurrents;//mReactanceMat * mFluxes;

		//mFluxes = mFluxes + dt * (mVoltages - mResistanceMat * mReactanceMat * mFluxes - mOmega_r * mOmegaFluxMat * mFluxes);
						
		

		// inverse dq-transform
		mCurrents = mReactanceMat * mFluxes;
		mDq0Currents(0, 0) = -mCurrents(0, 0);
		mDq0Currents(1, 0) = -mCurrents(1, 0);
		mDq0Currents(2, 0) = -mCurrents(2, 0);
		mAbcsCurrents = InverseParkTransform(mTheta_r, mDq0Currents);

		//mTheta_r = mTheta_r + dt * mOmega_r;
		mElecTorque = -3. * mPoleNumber / 4. * (mFluxes(1, 0)*mCurrents(0, 0) - mFluxes(0, 0)*mCurrents(1, 0));
		//mOmega_r = mOmega_r + dt * (mPoleNumber / (2 * mInertia) * (mMechTorque - mElecTorque));
		
	}

	// Update current source accordingly
	if (node1 >= 0) {
		j(node1, 0) = j(node1, 0) + mAbcsCurrents(0, 0);
		j(compOffset + node1, 0) = j(compOffset + node1, 0);
	}
	if (node2 >= 0) {
		j(node2, 0) = j(node2, 0) + mAbcsCurrents(1, 0);
		j(compOffset + node2, 0) = j(compOffset + node2, 0);
	}
	if (node3 >= 0) {
		j(node3, 0) = j(node3, 0) + mAbcsCurrents(2, 0);
		j(compOffset + node3, 0) = j(compOffset + node3, 0);
	}
}

/// Retrieves calculated voltage from simulation for next step
void SynchronousGenerator::PostStep(DPSMatrix& g, DPSMatrix& j, DPSMatrix& vt, int compOffset, double om, double dt, double t) {
	if (node1 >= 0) {
		mAbcsVoltages(0,0) = vt(node1, 0);		
	}
	else {
		mAbcsVoltages(0, 0) = 0;		
	}
	if (node2 >= 0) {
		mAbcsVoltages(1, 0) = vt(node2, 0);		
	}
	else {
		mAbcsVoltages(1, 0) = 0;		
	}
	if (node3 >= 0) {
		mAbcsVoltages(2, 0) = vt(node3, 0);		
	}
	else {
		mAbcsVoltages(2, 0) = 0;
	}
}

DPSMatrix SynchronousGenerator::ParkTransform(double theta, DPSMatrix& in) {
	DPSMatrix ParkMat(3,3);
	ParkMat << 2. / 3. * cos(theta), 2. / 3. * cos(theta - 2. * M_PI / 3.), 2. / 3. * cos(theta + 2. * M_PI / 3.),
		2. / 3. * sin(theta), 2. / 3. * sin(theta - 2. * M_PI / 3.), 2. / 3. * sin(theta + 2. * M_PI / 3.),
		1. / 3., 1. / 3., 1. / 3.;

	return ParkMat * in;
}

DPSMatrix SynchronousGenerator::InverseParkTransform(double theta, DPSMatrix& in) {
	DPSMatrix InverseParkMat(3,3);
	InverseParkMat << cos(theta), sin(theta), 1,
		cos(theta - 2. * M_PI / 3.), sin(theta - 2. * M_PI / 3.), 1,
		cos(theta + 2. * M_PI / 3.), sin(theta + 2. * M_PI / 3.), 1;

	return InverseParkMat * in;
}