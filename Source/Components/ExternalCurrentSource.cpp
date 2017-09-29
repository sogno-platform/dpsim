#include "ExternalCurrentSource.h"

using namespace DPsim;

ExternalCurrentSource::ExternalCurrentSource(std::string name, int src, int dest, Complex initCurrent) :
	CurrentSource(name, src, dest, initCurrent) {
}

void ExternalCurrentSource::setCurrent(Real real, Real imag) {
	this->mCurrent = Complex(real, imag);
}
