#include "ExternalVoltageSource.h"

using namespace DPsim;

ExternalVoltageSource::ExternalVoltageSource(std::string name, int src, int dest, Real initVoltage, Real initPhase, int num) :
	IdealVoltageSource(name, src, dest, initVoltage, initPhase, num) {
}

void ExternalVoltageSource::setVoltage(Real real, Real imag) {
	this->mVoltageDiffr = real;
	this->mVoltageDiffi = imag;
}
