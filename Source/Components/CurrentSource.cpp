#include "CurrentSource.h"

CurrentSource::CurrentSource(std::string name, int src, int dest, double current, double phase) : BaseComponent(name, src, dest) {
	this->currentr = current*cos(phase);
	this->currenti = current*sin(phase);
};
	
void CurrentSource::applyRightSideVectorStamp(DPSMatrix& j, int compOffset, double om, double dt) {
	if (mNode1 != 0) {
		j(mNode1, 0) = j(mNode1, 0) + currentr;
		j(mNode1, 0) = j(compOffset+mNode1, 0) + currenti;
	}
	if (mNode2 != 0) {
		j(mNode2, 1) = j(mNode2, 1) - currentr;
		j(mNode2, 1) = j(compOffset+mNode2, 1) - currenti;
	}
};
