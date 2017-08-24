#include "PQLoad.h"

using namespace DPsim;

PQLoad::PQLoad(std::string name, int src, int dest, Real p, Real q, Real volt, Real angle) : RxLine(name, src, dest, 1, 1) {
	// we need the system frequency to calculate the impedance, so we initialize
	// it with the dummy value of 1+j1 here for now
	mActivePower = p;
	mReactivePower = q;
	mSvVoltage = volt;
}

void PQLoad::init(Real om, Real dt) {
	Real abs = mActivePower*mActivePower+mReactivePower*mReactivePower;
	mResistance = mSvVoltage*mSvVoltage*mActivePower/abs;
	mConductance = 1.0 / mResistance;
	Real reactance = mSvVoltage*mSvVoltage*mReactivePower/abs;
	mInductance = reactance/om;
	RxLine::init(om, dt);
}
