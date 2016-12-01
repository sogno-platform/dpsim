#include "Simulation.h"

Simulation::Simulation() {

}

Simulation::Simulation(std::vector<BaseComponent*> elements, double om, double dt, double tf) {
	this->dt = dt;
	this->tf = tf;
	this->om = om;

	AddElements(elements);
	CreateSystemMatrix();
	Initialize();
}

Simulation::Simulation(std::vector<BaseComponent*> elements, double om, double dt, double tf, Logger& logger) : Simulation(elements, om, dt, tf) {
	for (std::vector<BaseComponent*>::iterator it = elements.begin(); it != elements.end(); ++it) {
		logger.Log(Logtype::INFO) << "Added " << (*it)->GetName() << " of type " << typeid(*(*it)).name() << " to simulation." << std::endl;
		(*it)->Init(A, j, compOffset, om, dt);
	}

	logger.Log(Logtype::INFO) << "System matrix A:" << std::endl;
	logger.Log() << A << std::endl;
	logger.Log(Logtype::INFO) << "Known variables matrix j:" << std::endl;
	logger.Log() << j << std::endl;
}


Simulation::~Simulation() {
	
}


void Simulation::Initialize() {
	// Initialize time variable
	t = 0;
	
	// Create the matrices
	for (std::vector<BaseComponent*>::iterator it = elements.begin(); it != elements.end(); ++it) {
		(*it)->Init(A, j, compOffset, om, dt);
	}

	luFactored = Eigen::PartialPivLU<DPSMatrix>(A);
}
	

void Simulation::AddElements(std::vector<BaseComponent*> newElements) {
	for (std::vector<BaseComponent*>::iterator it = newElements.begin(); it != newElements.end(); ++it) {
		elements.push_back((*it));

	}
}

void Simulation::CreateSystemMatrix() {
	int maxNode = 0;
	for (std::vector<BaseComponent*>::iterator it = elements.begin(); it != elements.end(); ++it) {
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

double Simulation::GetTime() {
	return t;
}
	
int Simulation::Step()
{
	j.setZero();
	
	for (std::vector<BaseComponent*>::iterator it = elements.begin(); it != elements.end(); ++it) {
		(*it)->Step(A, j, compOffset, om, dt, t);
	}
	
	vt = luFactored.solve(j);
 
	for (std::vector<BaseComponent*>::iterator it = elements.begin(); it != elements.end(); ++it) {
		(*it)->PostStep(A, j, vt, compOffset, om, dt, t);
	}

	t += dt;

	if(t >= tf)
		return 0;
	else
		return 1;
}

DPSMatrix Simulation::GetVoltages() {
	return vt;
}

std::ostringstream Simulation::GetVoltageDataLine() {
	std::ostringstream output;
	output << std::scientific << t;
	for (int i = 0; i < vt.rows(); i++) {
		output << ", " << vt(i, 0);
	}
	output << std::endl;
	return output;
}

std::ostringstream Simulation::GetCurrentDataLine() {
	std::ostringstream output;
	output << std::scientific << t;
	for (int i = 0; i < j.rows(); i++) {
		output << ", " << j(i, 0);
	}
	output << std::endl;
	return output;
}