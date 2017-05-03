
#include "ExternalInterface.h"

using namespace DPsim;

void ExternalInterface::registerVoltageSource(ExternalVoltageSource *evs, int num) {
	int sz = mExtComponents.size();
	if (num >= sz)
		mExtComponents.resize(num+1, NULL);
	mExtComponents[num] = evs;
}

void ExternalInterface::registerCurrentSource(ExternalCurrentSource *ecs, int num) {
	int sz = mExtComponents.size();
	if (num >= sz)
		mExtComponents.resize(num+1, NULL);
	mExtComponents[num] = ecs;
}

void ExternalInterface::registerExportedVoltage(VoltDiff vd, int num) {
	int sz = mExportedVoltages.size();
	if (num >= sz)
		mExportedVoltages.resize(num+1, {-1, -1});
	mExportedVoltages[num] = vd;
}
