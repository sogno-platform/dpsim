
#include "ExternalInterface.h"

using namespace DPsim;

void ExternalInterface::registerVoltageSource(ExternalVoltageSource *evs, Int realIdx, Int imagIdx) {
	mExtComponents.push_back({evs, realIdx, imagIdx});
}

void ExternalInterface::registerCurrentSource(ExternalCurrentSource *ecs, Int realIdx, Int imagIdx) {
	mExtComponents.push_back({ecs, realIdx, imagIdx});
}

void ExternalInterface::registerExportedVoltage(Int from, Int to, Int realIdx, Int imagIdx) {
	mExportedVoltages.push_back({from, to, realIdx, imagIdx});
}

void ExternalInterface::registerExportedCurrent(BaseComponent *comp, Int realIdx, Int imagIdx) {
	mExportedCurrents.push_back({comp, realIdx, imagIdx});
}
