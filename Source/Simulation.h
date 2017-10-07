/** The simulation
 *
 * @file
 * @author Markus Mirz <mmirz@eonerc.rwth-aachen.de>
 * @copyright 2017, Institute for Automation of Complex Power Systems, EONERC
 * @license GNU General Public License (version 3)
 *
 * DPsim
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *********************************************************************************/

#ifndef Simulation_H
#define Simulation_H

#include <signal.h>
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

	/* Possible methods to achieve execution in real time. */
	enum RTMethod {
		RTExceptions, // use a normal timer and throw an exception in the signal handler if the timestep wasn't completed yet
		RTTimerFD,    // read on a timerfd after every step
	};

	class TimerExpiredException {
	};

	class Simulation {

	private:
		/// Final time of the simulation
		Real mFinalTime;
		/// Time variable that is incremented at every step
		Real mTime;
		/// Last simulation time step when log was updated
		Int mLastLogTimeStep;
		/// Down sampling rate
		Int mDownSampleRate;
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

		uint64_t mRtTimerCount = 0;

		/// TODO: check that every system matrix has the same dimensions		
		void initialize(std::vector<BaseComponent*> elements);


	public:				
		/// Stores a list of circuit elements that are used to generate the system matrix
		std::vector<BaseComponent*> mElements;

		/// Sets parameters to default values.
		Simulation();
		/// Creates system matrix according to 
		Simulation(std::vector<BaseComponent*> elements, Real om, Real dt, Real tf, SimulationType simType = SimulationType::DynPhasor);
		Simulation(std::vector<BaseComponent*> elements, Real om, Real dt, Real tf, Logger& logger, SimulationType simType = SimulationType::DynPhasor);
		Simulation(std::vector<BaseComponent*> elements, Real om, Real dt, Real tf, Logger& logger, Int downSampleRate, SimulationType simType = SimulationType::DynPhasor);
		~Simulation();

		
		/// Solve system A * x = z for x and current time
		int step(Logger& logger, bool blocking = true);
		/// Solve system A * x = z for x and current time. Log current values of both vectors.
		int step(Logger& logger, Logger& leftSideVectorLog, Logger& rightSideVectorLog, bool blocking = true);
		void switchSystemMatrix(int systemMatrixIndex);
		void setSwitchTime(Real switchTime, Int systemIndex);
		void increaseByTimeStep();
		void addExternalInterface(ExternalInterface*);

		void setNumericalMethod(NumericalMethod numMethod);

		double getTime() { return mTime; }
		double getFinalTime() { return mFinalTime; }
		double getTimeStep() { return mSystemModel.getTimeStep(); }
		Matrix & getLeftSideVector() { return mSystemModel.getLeftSideVector(); }
		Matrix & getRightSideVector() { return mSystemModel.getRightSideVector(); }
		Matrix & getSystemMatrix() { return mSystemModel.getCurrentSystemMatrix(); }
		int stepGeneratorTest(Logger& logger, Logger& leftSideVectorLog, Logger& rightSideVectorLog, 
			BaseComponent* generator, Real time);		
		int stepGeneratorVBR(Logger& logger, BaseComponent* generator,
			Logger& synGenLogVolt, Logger& synGenLogCurr, Logger& synGenLogElecTorque, Logger& synGenLogOmega, Logger& synGenLogTheta,
			Real fieldVoltage, Real mechPower, Real time);

		void addSystemTopology(std::vector<BaseComponent*> newElements);

#ifdef __linux__
		/* Perform the main simulation loop in real time.
		 *
		 * @param rtMethod The method with which the realtime execution is achieved.
		 * @param startSynch If true, the simulation waits for the first external value before starting the timing.
		 * @param logger Logger which is used to log general information.
		 * @param llogger Logger which is used to log the left-side (solution).
		 * @param rlogger Logger which is used to log the right-side vector.
		 */
		void runRT(RTMethod rtMethod, bool startSynch, Logger& logger, Logger& llogger, Logger &rlogger);
		static void alarmHandler(int, siginfo_t*, void*);
#endif
	};

}

#endif






