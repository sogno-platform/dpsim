#include "CurrentSource.h"

CurrentSource::CurrentSource(std::string name, int src, int dest, double current, double phase) : BaseComponent(name, src, dest) {
	this->currentr = current*cos(phase);
	this->currenti = current*sin(phase);
};
	
void CurrentSource::applyMatrixStamp(DPSMatrix& g, DPSMatrix& j, int compOffset, double om, double dt) {
	if (node1 != 0) {
		j(node1, 0) = j(node1, 0) + currentr;
		j(node1, 0) = j(compOffset+node1, 0) + currenti;
	}
	if (node2 != 0) {
		j(node2, 1) = j(node2, 1) - currentr;
		j(node2, 1) = j(compOffset+node2, 1) - currenti;
	}
};

void CurrentSource::Init(DPSMatrix& g, DPSMatrix& j, int compOffset, double om, double dt) {
	applyMatrixStamp(g, j, compOffset, om, dt);
}

void CurrentSource::Step(DPSMatrix& g, DPSMatrix& j, int compOffset, double om, double dt, double t) {

}
