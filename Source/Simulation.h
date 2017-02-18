#ifndef Simulation_H
#define Simulation_H

#include <iostream>
#include <vector>
#include "MathLibrary.h"
#include "Components.h"
#include "Logger.h"

namespace DPsim {

	struct switchConfiguration {
		Real switchTime;
		UInt systemIndex;
	};

	enum class SimulationType{DynPhasor, EMT};

	class Simulation {

	protected:
		/// Simulation type
		SimulationType mSimType;
		/// Index of the next switching
		UInt mCurrentSwitchTimeIndex;
		/// Vector of switch times
		std::vector<switchConfiguration> mSwitchEventVector;

	public:
		/// Number of nodes
		int mNumNodes;
		/// Index offset for imaginary part
		int mCompOffset;
		/// Angular frequency of the phasor
		Real mSystemOmega;
		/// Final time of the simulation
		Real mFinalTime;
		/// Simulation time step
		Real mTimeStep;
		/// Time variable that is incremented at every step
		Real mTime;
		
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
		Simulation(std::vector<BaseComponent*> elements, Real om, Real dt, Real tf);
		Simulation(std::vector<BaseComponent*> elements, Real om, Real dt, Real tf, Logger& logger);
		Simulation(std::vector<BaseComponent*> elements, Real om, Real dt, Real tf, Logger& logger, SimulationType simType);
		~Simulation();

		/// TODO: check that every system matrix has the same dimensions
		void CreateSystemMatrix(std::vector<BaseComponent*> elements);
		void Initialize();
		/// Solve system A * x = z for x and current time
		int step(Logger& logger);
		/// Solve system A * x = z for x and current time. Log current values of both vectors.
		int step(Logger& logger, Logger& leftSideVectorLog, Logger& rightSideVectorLog);
		void switchSystemMatrix(int systemMatrixIndex);
		void setSwitchTime(Real switchTime, Int systemIndex);
		void increaseByTimeStep();

		double getTime() { return mTime; }
		double getFinalTime() { return mFinalTime; }
		DPSMatrix getLeftSideVector() { return mLeftSideVector; }
		DPSMatrix getRightSideVector() { return mRightSideVector; }
	};

}

#endif






