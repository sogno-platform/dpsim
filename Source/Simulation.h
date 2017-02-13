#ifndef Simulation_H
#define Simulation_H

#include <iostream>
#include <vector>
#include "MathLibrary.h"
#include "Components.h"
#include "Logger.h"

class Simulation {
protected:
	

public:
	/// Number of nodes
	int mNumNodes;
	/// Index offset for imaginary part
	int mCompOffset;
	/// Angular frequency of the phasor
	double mSystemOmega;
	/// Final time of the simulation
	double mFinalTime;
	/// Simulation time step
	double mTimeStep;
	/// Time variable that is incremented at every step
	double mTime;
	/// Stores a list of circuit elements that are used to generate the system matrix
	std::vector<BaseComponent*> mElements;
	/// Circuit list vector
	std::vector<std::vector<BaseComponent*> > mElementsVector;
	/// LU decomposition of system matrix A
	Eigen::PartialPivLU<DPSMatrix> mLuFactored;
	/// LU decomposition of system matrix A
	std::vector<Eigen::PartialPivLU<DPSMatrix> > mLuFactoredVector;		
	/// System matrix A that is modified by matrix stamps 
	DPSMatrix mSystemMatrix;
	/// System matrices list for swtiching events
	std::vector<DPSMatrix> mSystemMatrixVector;
	/// Vector of known quantities
	DPSMatrix mRightSideVector;
	/// Vector of unknown quantities
	DPSMatrix mLeftSideVector;

	Simulation();
	Simulation(std::vector<BaseComponent*> elements, double om, double dt, double tf);
	Simulation(std::vector<BaseComponent*> elements, double om, double dt, double tf, Logger& logger);
	~Simulation();

	void CreateSystemMatrix(std::vector<BaseComponent*> elements);
	void Initialize();
	
	double getTime() { return mTime; }
	int Step();
	DPSMatrix getLeftSideVector() { return mLeftSideVector; }
	DPSMatrix getRightSideVector() { return mRightSideVector; }
};

#endif






