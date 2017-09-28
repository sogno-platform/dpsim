#include "ExternalVoltageSource.h"

using namespace DPsim;

ExternalVoltageSource::ExternalVoltageSource(std::string name, int src, int dest, Complex voltage, int num) :
	IdealVoltageSource(name, src, dest, voltage, num) {
}

void ExternalVoltageSource::setVoltage(Real real, Real imag) {
	this->mVoltage = Complex(real, imag);
}
