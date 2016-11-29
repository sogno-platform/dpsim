#include "SynchronousGenerator.h"

SynchronousGenerator::SynchronousGenerator(std::string name, int src, int dest, double statorRes, double leakInd, double mutInd_d,
	double mutInd_q, double fieldRes, double fieldLeakInd, double dampRes_d, double dampLeakInd_d, double dampRes1_q, double dampLeakInd1_q,
	double dampRes2_q, double dampLeakInd2_q, double inertia, int poleNumber) : BaseComponent(name, src, dest) {

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
	
}

void SynchronousGenerator::Step(DPSMatrix& g, DPSMatrix& j, int compOffset, double om, double dt, double t) {
	// Euler forward step with state space equations
}

void SynchronousGenerator::PostStep(DPSMatrix& g, DPSMatrix& j, DPSMatrix& vt, int compOffset, double om, double dt, double t) {
	// Calculate actual voltage fir next step
}