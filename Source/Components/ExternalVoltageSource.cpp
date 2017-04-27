#include "ExternalVoltageSource.h"

using namespace DPsim;

ExternalVoltageSource::ExternalVoltageSource(std::string name, int src, int dest, Real voltage, Real phase, int num): IdealVoltageSource(name, src, dest, voltage, phase, num) {
	this->mPhase = phase;
}

void ExternalVoltageSource::setVoltage(Real voltage) {
	this->mVoltageDiffr = voltage*cos(mPhase);
	this->mVoltageDiffi = voltage*sin(mPhase);
}
