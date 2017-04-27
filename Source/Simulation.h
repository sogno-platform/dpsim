#ifndef Simulation_H
#define Simulation_H

#include <iostream>
#include <vector>
#include "MathLibrary.h"
#include "Components.h"
#include "Logger.h"
#include "SystemModel.h"
#include "ExternalInterface.h"

namespace DPsim {

	struct switchConfiguration {
		Real switchTime;
		UInt systemIndex;
	};

	class Simulation {

	private:
		/// Final time of the simulation
		Real mFinalTime;
		/// Time variable that is incremented at every step
		Real mTime;
		/// Index of the next switching
		UInt mCurrentSwitchTimeIndex;
		/// Vector of switch times
		std::vector<switchConfiguration> mSwitchEventVector;
		/// Structure that holds all system information.
		SystemModel mSystemModel;
		
		/// Circuit list vector
		std::vector<std::vector<BaseComponent*> > mElementsVector;

		/// Vector of ExternalInterfaces
		std::vector<ExternalInterface*> mExternalInterfaces;

		/// TODO: check that every system matrix has the same dimensions		
		void initialize(std::vector<BaseComponent*> elements);

	public:				
		/// Stores a list of circuit elements that are used to generate the system matrix
		std::vector<BaseComponent*> mElements;

		/// Sets parameters to default values.
		Simulation();
		/// Creates system matrix according to 
		Simulation(std::vector<BaseComponent*> elements, Real om, Real dt, Real tf);
		Simulation(std::vector<BaseComponent*> elements, Real om, Real dt, Real tf, Logger& logger);
		Simulation(std::vector<BaseComponent*> elements, Real om, Real dt, Real tf, Logger& logger, SimulationType simType);
		~Simulation();

		
		/// Solve system A * x = z for x and current time
		int step(Logger& logger);
		/// Solve system A * x = z for x and current time. Log current values of both vectors.
		int step(Logger& logger, Logger& leftSideVectorLog, Logger& rightSideVectorLog);
		void switchSystemMatrix(int systemMatrixIndex);
		void setSwitchTime(Real switchTime, Int systemIndex);
		void increaseByTimeStep();
		void addExternalInterface(ExternalInterface*);

		double getTime() { return mTime; }
		double getFinalTime() { return mFinalTime; }
		Matrix getLeftSideVector() { return mSystemModel.getLeftSideVector(); }
		Matrix getRightSideVector() { return mSystemModel.getRightSideVector(); }
		Matrix getSystemMatrix() { return mSystemModel.getCurrentSystemMatrix(); }
		int stepGeneratorTest(Logger& logger, Logger& leftSideVectorLog, Logger& rightSideVectorLog, 
			BaseComponent* generator, Logger& synGenLogFlux, Logger& synGenLogVolt, Logger& synGenLogCurr, Real fieldVoltage, Real mechPower, 
			Real logTimeStep, Real& lastLogTime);

		void addSystemTopology(std::vector<BaseComponent*> newElements);
	};

}

#endif






