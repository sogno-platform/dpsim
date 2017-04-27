#include "ExternalCurrentSource.h"

using namespace DPsim;

ExternalCurrentSource::ExternalCurrentSource(std::string name, int src, int dest, Real current, Real phase) : CurrentSource(name, src, dest, current, phase) {
	this->mPhase = phase;
}

void ExternalCurrentSource::setCurrent(Real current) {
	this->currentr = current*cos(this->mPhase);
	this->currenti = current*sin(this->mPhase);
}
