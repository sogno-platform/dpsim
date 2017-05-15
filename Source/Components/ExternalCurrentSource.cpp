#include "ExternalCurrentSource.h"

using namespace DPsim;

ExternalCurrentSource::ExternalCurrentSource(std::string name, int src, int dest, Real initCurrent, Real initPhase) :
	CurrentSource(name, src, dest, initCurrent, initPhase) {
}

void ExternalCurrentSource::setCurrent(Real real, Real imag) {
	this->currentr = real;
	this->currenti = imag;
}
