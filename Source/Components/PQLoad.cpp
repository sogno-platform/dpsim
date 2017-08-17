#include "PQLoad.h"

using namespace DPsim;

PQLoad::PQLoad(std::string name, int src, int dest, Real p, Real q) : BaseComponent(name, src, dest) {
	this->mActivePower = p;
	this->mReactivePower = q;
}
