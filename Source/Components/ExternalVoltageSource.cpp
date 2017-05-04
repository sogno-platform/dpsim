#include "ExternalVoltageSource.h"

using namespace DPsim;

ExternalVoltageSource::ExternalVoltageSource(std::string name, int src, int dest, int num): IdealVoltageSource(name, src, dest, 0, 0, num) {
}

void ExternalVoltageSource::setVoltage(Real real, Real imag) {
	this->mVoltageDiffr = real;
	this->mVoltageDiffi = imag;
}
