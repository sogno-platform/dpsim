#include "PQLoad.h"

using namespace DPsim;

PQLoad::PQLoad(std::string name, int src, int dest, Real p, Real q, Real volt, Real angle) : RxLine(name, src, dest, 1, 1) {
	// we need the system frequency to calculate the impedance, so we initialize
	// it with the dummy value of 1+j1 here for now
	mActivePower = p;
	mReactivePower = q;
	mSvVoltage = volt;
	// the parameters of the RxLine shouldn't be modified directly; the face that
	// this component inherits from RxLine is just an implementation details that
	// may change
	attrMap.erase(attrMap.find("resistance"));
	attrMap.erase(attrMap.find("inductance"));
	attrMap["activePower"] = {AttrReal, &this->mActivePower};
	attrMap["reactivePower"] = {AttrReal, &this->mReactivePower};
	attrMap["svVoltage"] = {AttrReal, &this->mSvVoltage};
}

void PQLoad::init(Real om, Real dt) {
	Real abs = mActivePower*mActivePower+mReactivePower*mReactivePower;
	mResistance = mSvVoltage*mSvVoltage*mActivePower/abs;
	mConductance = 1.0 / mResistance;
	Real reactance = mSvVoltage*mSvVoltage*mReactivePower/abs;
	mInductance = reactance/om;
	RxLine::init(om, dt);
}

void PQLoad::applySystemMatrixStamp(SystemModel& system) {
	// powers / svvoltage might have changed, so update them
	Real abs = mActivePower*mActivePower+mReactivePower*mReactivePower;
	mResistance = mSvVoltage*mSvVoltage*mActivePower/abs;
	mConductance = 1.0 / mResistance;
	Real reactance = mSvVoltage*mSvVoltage*mReactivePower/abs;
	mInductance = reactance/system.getOmega();
	RxLine::applySystemMatrixStamp(system);
}
