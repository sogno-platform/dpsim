#include "ExternalCurrentSource.h"

using namespace DPsim;

ExternalCurrentSource::ExternalCurrentSource(std::string name, int src, int dest) : CurrentSource(name, src, dest, 0, 0) {
}

void ExternalCurrentSource::setCurrent(Real real, Real imag) {
	this->currentr = real;
	this->currenti = imag;
}
