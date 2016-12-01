#ifndef Simulation_H
#define Simulation_H

#include <iostream>
#include <vector>
#include "MathLibrary.h"
#include "Components.h"
#include "Logger.h"

class Simulation {
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
	std::vector<BaseComponent*> elements;
	/// System matrix that is modified by matrix stamps 
	DPSMatrix A;
	/// Vector of known quantities
	DPSMatrix j;
	/// Vector of unknown quantities
	DPSMatrix vt;
	/// LU decomposition of system matrix A
	Eigen::PartialPivLU<DPSMatrix> luFactored;

	void AddElements(std::vector<BaseComponent*> elements);
	void CreateSystemMatrix();
	void Initialize();

public:
	Simulation();
	Simulation(std::vector<BaseComponent*> elements, double om, double dt, double tf);
	Simulation(std::vector<BaseComponent*> elements, double om, double dt, double tf, Logger& logger);
	~Simulation();
	
	double GetTime();
	int Step();
	DPSMatrix GetVoltages();
	std::ostringstream GetVoltageDataLine();
	std::ostringstream GetCurrentDataLine();

};

#endif






