#include "LinearResistor.h"

LinearResistor::LinearResistor(std::string name, int src, int dest, double resistance) : BaseComponent(name, src, dest) {
	this->resistance = resistance;
	this->conductance = 1.0 / resistance;
}	
		
void LinearResistor::applySystemMatrixStamp(DPSMatrix& g, int compOffset, double om, double dt) {

	// Set diagonal entries
	if (node1 >= 0) {
		g(node1,node1) = g(node1,node1) + conductance;
		g(compOffset+node1,compOffset+node1) = g(compOffset+node1,compOffset+node1) + conductance;
	}
	
	if (node2 >= 0) {
		g(node2,node2) = g(node2,node2) + conductance;
		g(compOffset+node2,compOffset+node2) = g(compOffset+node2,compOffset+node2) + conductance;
	}

	// Set off diagonal entries
	if (node1 >= 0 && node2 >= 0) {
		g(node1, node2) = g(node1, node2) - conductance;
		g(compOffset + node1, compOffset + node2) = g(compOffset + node1, compOffset + node2) - conductance;

		g(node2, node1) = g(node2, node1) - conductance;
		g(compOffset + node2, compOffset + node1) = g(compOffset + node2, compOffset + node1) - conductance;
	}
}
	

