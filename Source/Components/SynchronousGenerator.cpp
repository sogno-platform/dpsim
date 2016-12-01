#include "SynchronousGenerator.h"

SynchronousGenerator::SynchronousGenerator(std::string name, int node1, int node2, int node3, double statorRes, double leakInd, double mutInd_d,
	double mutInd_q, double fieldRes, double fieldLeakInd, double dampRes_d, double dampLeakInd_d, double dampRes1_q, double dampLeakInd1_q,
	double dampRes2_q, double dampLeakInd2_q, double inertia, int poleNumber) {

	this->node1 = node1;
	this->node2 = node2;
	this->node3 = node3;
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

void SynchronousGenerator::Init(DPSMatrix& g, DPSMatrix& j, int compOffset, double om, double dt) {
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

}

void SynchronousGenerator::Step(DPSMatrix& g, DPSMatrix& j, int compOffset, double om, double dt, double t) {
	// Euler forward step with state space equations
	mFluxes = mFluxes + mVoltages - mResistanceMat * mReactanceMat * mFluxes - mOmega_r * mOmegaFluxMat * mFluxes;
}

void SynchronousGenerator::PostStep(DPSMatrix& g, DPSMatrix& j, DPSMatrix& vt, int compOffset, double om, double dt, double t) {
	// Calculate actual voltage for next step
	double vposr, vnegr;
	double vposi, vnegi;

	// extract solution
	if (node1 >= 0) {
		vposr = vt(node1, 0);
		vposi = vt(compOffset + node1, 0);
	}
	else {
		vposr = 0;
		vposi = 0;
	}

	if (node2 >= 0) {
		vnegr = vt(node2, 0);
		vnegi = vt(compOffset + node2, 0);
	}
	else {
		vnegr = 0;
		vnegi = 0;
	}
	voltageRe = vposr - vnegr;
	voltageIm = vposi - vnegi;
}

void SynchronousGenerator::ParkTransform(double theta, DPSMatrix& in, DPSMatrix& out) {
	DPSMatrix ParkMat;
	ParkMat << 2 / 3 * cos(theta), 2 / 3 * cos(theta - 2 * M_PI / 3), 2 / 3 * cos(theta + 2 * M_PI / 3),
		2 / 3 * sin(theta), 2 / 3 * sin(theta - 2 * M_PI / 3), 2 / 3 * sin(theta + 2 * M_PI / 3),
		1 / 3, 1 / 3, 1 / 3;

	out = ParkMat * in;
}

void SynchronousGenerator::InverseParkTransform(double theta, DPSMatrix& in, DPSMatrix& out) {
	DPSMatrix InverseParkMat;
	InverseParkMat << cos(theta), sin(theta), 1,
		cos(theta - 2 * M_PI / 3), sin(theta - 2 * M_PI / 3), 1,
		cos(theta + 2 * M_PI / 3), sin(theta + 2 * M_PI / 3), 1;

	out = InverseParkMat * in;
}