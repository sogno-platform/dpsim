#include "SynchronousGenerator.h"

SynchronousGenerator::SynchronousGenerator(std::string name, int node1, int node2, int node3, double nomVolt, double statorRes, double leakInd, double mutInd_d,
	double mutInd_q, double fieldRes, double fieldLeakInd, double dampRes_d, double dampLeakInd_d, double dampRes1_q, double dampLeakInd1_q,
	double dampRes2_q, double dampLeakInd2_q, double inertia, int poleNumber) {

	this->node1 = node1 - 1;
	this->node2 = node2 - 1;
	this->node3 = node3 - 1;
	mNomVolt = nomVolt;
	mStatorRes = statorRes;
	mLeakInd = leakInd;
	mMutInd_d = mutInd_d;
	mMutInd_q = mutInd_q;
	mFieldRes = fieldRes;
	mFieldLeakInd = fieldLeakInd;
	mDampRes_d = dampRes_d;
	mDampRes1_q = dampRes1_q;
	mDampRes2_q = dampRes2_q;
	mDampLeakInd1_q = dampLeakInd1_q;
	mDampLeakInd2_q = dampLeakInd2_q;
	mInertia = inertia;
	mPoleNumber = poleNumber;
}

void SynchronousGenerator::applyMatrixStamp(DPSMatrix& g, DPSMatrix& j, int compOffset, double om, double dt) {
	// Compensated current source
}

void SynchronousGenerator::Init(DPSMatrix& g, DPSMatrix& j, int compOffset, double om, double dt, DPSMatrix initAbcsCurrents, DPSMatrix initAbcsVoltages, double initFieldVoltage, double initFieldCurrent, double initTheta_r) {
	
	// Create matrices for state space representation 
	mInductanceMat << mLeakInd + mMutInd_q, 0, 0, mMutInd_q, mMutInd_q, 0, 0,
		0, mLeakInd + mMutInd_d, 0, 0, 0, mMutInd_d, mMutInd_d,
		0, 0, mLeakInd, 0, 0, 0, 0,
		mMutInd_q, 0, 0, mDampLeakInd1_q + mMutInd_q, mMutInd_q, 0, 0,
		mMutInd_q, 0, 0, mMutInd_q, mDampLeakInd2_q + mMutInd_q, 0, 0,
		0, mMutInd_d, 0, 0, 0, mFieldLeakInd, mMutInd_d,
		0, mMutInd_d, 0, 0, 0, mMutInd_d, mDampLeakInd_d;

	mResistanceMat << mStatorRes, 0, 0, 0, 0, 0, 0,
		0, mStatorRes, 0, 0, 0, 0, 0,
		0, 0, mStatorRes, 0, 0, 0, 0,
		0, 0, 0, mDampRes1_q, 0, 0, 0,
		0, 0, 0, 0, mDampRes2_q, 0, 0,
		0, 0, 0, 0, 0, mFieldRes, 0,
		0, 0, 0, 0, 0, 0, mDampRes_d;

	mOmegaFluxMat << 0, 1, 0, 0, 0, 0, 0,
		-1, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0;

	mReactanceMat = mInductanceMat.inverse();

	// initialize currents and voltages
	mTheta_r = initTheta_r;
	mOmega_r = om;
	mAbcsCurrents = initAbcsCurrents;
	mDq0Currents = ParkTransform(mTheta_r, mAbcsCurrents);
	mCurrents(0, 0) = mDq0Currents(0, 0);
	mCurrents(1, 0) = mDq0Currents(1, 0);
	mCurrents(2, 0) = mDq0Currents(2, 0);
	mCurrents(5, 0) = initFieldCurrent;
	mFluxes = mInductanceMat * mCurrents;
	mCurrents = mReactanceMat * mFluxes;
	mAbcsVoltages = initAbcsVoltages;
	mDq0Voltages = ParkTransform(mTheta_r, mAbcsVoltages);
	mVoltages(0, 0) = mDq0Voltages(0, 0);
	mVoltages(1, 0) = mDq0Voltages(1, 0);
	mVoltages(2, 0) = mDq0Voltages(2, 0);
	mVoltages(5, 0) = initFieldVoltage;
	
}

/// Performs an Euler forward step with the state space model of a synchronous generator 
/// to calculate the flux and current from the voltage vector.
void SynchronousGenerator::Step(DPSMatrix& g, DPSMatrix& j, int compOffset, double om, double dt, double t, double fieldVoltage, double fieldCurrent, double mechPower) {

	mVoltages(5, 0) = fieldVoltage;
	mCurrents(5, 0) = fieldCurrent;

	mMechPower = mechPower;
	mMechTorque = mMechPower / mOmega_r;
	mElecTorque = 3. * mPoleNumber / 4. * (mFluxes(1, 0)*mCurrents(0, 0) - mFluxes(0, 0)*mCurrents(1, 0));

	// dq-transform of interface voltage
	mDq0Voltages = ParkTransform(mTheta_r, mAbcsVoltages);
	mVoltages(0, 0) = mDq0Voltages(0, 0);
	mVoltages(1, 0) = mDq0Voltages(1, 0);
	mVoltages(2, 0) = mDq0Voltages(2, 0);

	// Euler step forward
	mTheta_r = mTheta_r + dt * mOmega_r;
	mOmega_r = mOmega_r + dt * (mPoleNumber / 2 * mInertia*(mElecTorque - mMechTorque));
	
	mFluxes = mFluxes + dt * (mVoltages - mResistanceMat * mReactanceMat * mFluxes - mOmega_r * mOmegaFluxMat * mFluxes);

	// inverse dq-transform
	mCurrents = mReactanceMat * mFluxes;
	mDq0Currents(0, 0) = mCurrents(0, 0);
	mDq0Currents(1, 0) = mCurrents(1, 0);
	mDq0Currents(2, 0) = mCurrents(2, 0);
	mAbcsCurrents = InverseParkTransform(mTheta_r, mDq0Currents);

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

void SynchronousGenerator::PostStep(DPSMatrix& g, DPSMatrix& j, DPSMatrix& vt, int compOffset, double om, double dt, double t) {
	// Calculate actual voltage for next step

	// extract solution
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