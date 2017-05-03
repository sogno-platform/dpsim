
#include "ExternalInterface.h"

using namespace DPsim;

void ExternalInterface::registerVoltageSource(ExternalVoltageSource *evs, int num) {
	int sz = mExtComponents.size();
	if (num >= sz) {
		mExtComponents.reserve(num+1);
		for (int i = sz; i < num; i++)
			mExtComponents[i] = NULL;
	}
	mExtComponents[num] = evs;
}

void ExternalInterface::registerCurrentSource(ExternalCurrentSource *ecs, int num) {
	int sz = mExtComponents.size();
	if (num >= sz) {
		mExtComponents.reserve(num+1);
		for (int i = sz; i < num; i++)
			mExtComponents[i] = NULL;
	}
	mExtComponents[num] = ecs;
}

