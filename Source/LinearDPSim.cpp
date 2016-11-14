#include "LinearDPSim.h"

LinearDPSim::LinearDPSim() {

}

LinearDPSim::LinearDPSim(std::vector<CircuitElement*> elements, double om, double dt, double tf) {
	this->dt = dt;
	this->tf = tf;
	this->om = om;

	addElements(elements);
	createSystemMatrix();
	initialize();
}


LinearDPSim::~LinearDPSim() {
	
}


void LinearDPSim::initialize() {
	// Initialize time variable
	t = 0;
	
	// Create the matrices
	for (std::vector<CircuitElement*>::iterator it = elements.begin(); it != elements.end(); ++it) {
		(*it)->Init(A, j, compOffset, om, dt);
	}

	std::cout << A << std::endl;
	luFactored = Eigen::PartialPivLU<DPSMatrix>(A);
}
	

void LinearDPSim::addElements(std::vector<CircuitElement*> newElements) {
	for (std::vector<CircuitElement*>::iterator it = newElements.begin(); it != newElements.end(); ++it) {
		elements.push_back((*it));
	}
}

void LinearDPSim::createSystemMatrix() {
	int maxNode = 0;
	for (std::vector<CircuitElement*>::iterator it = elements.begin(); it != elements.end(); ++it) {
		if ((*it)->getNode1() > maxNode)
			maxNode = (*it)->getNode1();
		if ((*it)->getNode2() > maxNode)
			maxNode = (*it)->getNode2();
	}
	
	numNodes = maxNode + 1;
	compOffset = numNodes;
	A = DPSMatrix::Zero(2 * numNodes, 2 * numNodes);
	j = DPSMatrix::Zero(2 * numNodes, 1);
}

double LinearDPSim::getTime() {
	return t;
}
	
int LinearDPSim::step()
{
	j.setZero();
	
	for (std::vector<CircuitElement*>::iterator it = elements.begin(); it != elements.end(); ++it) {
		(*it)->Step(A, j, compOffset, om, dt, t);
	}
	
	vt = luFactored.solve(j);
 
	for (std::vector<CircuitElement*>::iterator it = elements.begin(); it != elements.end(); ++it) {
		(*it)->PostStep(A, j, vt, compOffset, om, dt, t);
	}

	t += dt;

	if(t >= tf)
		return 0;
	else
		return 1;
}

DPSMatrix LinearDPSim::getVoltages() {
	return vt;
}
