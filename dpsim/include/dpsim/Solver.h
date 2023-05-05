/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <iostream>
#include <typeinfo>
#include <vector>
#include <list>

#include <dpsim/Definitions.h>
#include <dpsim/SolverParameters.h> 
#include <dpsim/Config.h>
#include <dpsim/DirectLinearSolverConfiguration.h>
#include <dpsim-models/Logger.h>
#include <dpsim-models/SystemTopology.h>
#include <dpsim-models/Task.h>

using namespace std;

namespace DPsim {
	/// Holds switching time and which system should be activated.
	struct SwitchConfiguration {
		Real switchTime;
		UInt systemIndex;
	};

	/// Base class for more specific solvers such as MNA, ODE or IDA.
	class Solver {
	public:
		typedef std::shared_ptr<Solver> Ptr;
		typedef std::vector<Ptr> List;

		enum Behaviour { Initialization, Simulation };

	protected:
		/// Name for logging
		String mName;
		/// Logging level
		CPS::Logger::Level mLogLevel;
		/// Logger
		CPS::Logger::Log mSLog;
		/// Solver Parameters
		std::shared_ptr<SolverParameters> mSolverParams;
		/// Time step for fixed step solvers
		Real mTimeStep;
		
		// #### Initialization ####
		/// Enable recomputation of system matrix during simulation
		Bool mSystemMatrixRecomputation = false;
		/// Determines if solver is in initialization phase, which requires different behavior
		Bool mIsInInitialization = false;

	public:

		Solver(String name, CPS::Logger::Level logLevel) :
			mName(name),
			mLogLevel(logLevel),
			mSLog(CPS::Logger::get(name + "_Solver", logLevel, CPS::Logger::Level::warn)) {
		}

		virtual ~Solver() { }

		
		// #### Solver settings ####
		/// Solver types:
		/// Modified Nodal Analysis, Differential Algebraic, Newton Raphson
		enum class Type { MNA, DAE, NRP };
		///
		///
		virtual void setSystem(const CPS::SystemTopology &system) {}
		///
		void setTimeStep(Real timeStep) { mTimeStep = timeStep; }
		///
		void doSystemMatrixRecomputation(Bool value) { mSystemMatrixRecomputation = value; }

		void setSolverParameters(std::shared_ptr<SolverParameters> &solverParameters){ mSolverParams = solverParameters; }
		
		// #### Initialization ####
		///
		virtual void initialize() {}
		/// activate steady state initialization
		
		/// log LU decomposition times, if applicable
		virtual void logLUTimes()
		{
			// no default implementation for all types of solvers
		}

		// #### Simulation ####
		/// Get tasks for scheduler
		virtual CPS::Task::List getTasks() = 0;
		/// Log results
		virtual void log(Real time, Int timeStepCount) { };

		/// ### SynGen Interface ###
		int mMaxIterations = 10;
		void setMaxNumberOfIterations(int maxIterations) {mMaxIterations = maxIterations;}
	};
}
