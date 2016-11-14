#ifndef LINEARDPSIM_H
#define LINEARDPSIM_H

#include <iostream>
#include <vector>
#include "MathLibrary.h"
#include "Components.h"

class LinearDPSim {
protected:
	/// Number of nodes
	int numNodes; 
	/// Index offset for imaginary part
	int compOffset;
	/// Angular frequency of the phasor
	double om;
	/// Final time of the simulation
	double tf;
	/// Simulation time step
	double dt;
	/// Time variable that is incremented at every step
	double t;
	/// Stores a list of circuit elements that are used to generate the system matrix
	std::vector<CircuitElement*> elements;
	/// System matrix that is modified by matrix stamps 
	DPSMatrix A;
	/// Vector of known quantities
	DPSMatrix j;
	/// Vector of unknown quantities
	DPSMatrix vt;
	/// LU decomposition of system matrix A
	Eigen::PartialPivLU<DPSMatrix> luFactored;

	void addElements(std::vector<CircuitElement*> elements);
	void createSystemMatrix();
	void initialize();

public:
	LinearDPSim();
	LinearDPSim(std::vector<CircuitElement*> elements, double om, double dt, double tf);
	~LinearDPSim();
	
	double getTime();
	int step();
	DPSMatrix getVoltages();
};

#endif






