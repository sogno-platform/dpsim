/* Copyright 2017-2022 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <iostream>
#include <vector>
#include <list>

#include <dpsim/Solver.h>
#include <dpsim/DataLogger.h>
#include <dpsim-models/Solver/DAEInterface.h>
#include <dpsim/Scheduler.h>
#include <dpsim-models/SimPowerComp.h>
#include <dpsim-models/Logger.h>
#include <dpsim-models/AttributeList.h>


#include <ida/ida.h>
//#include <ida/ida_direct.h>
#include <nvector/nvector_serial.h>
#include <sunlinsol/sunlinsol_dense.h>
#include <sundials/sundials_types.h>

namespace DPsim {

	/// Solver class which uses Differential Algebraic Equation(DAE) systems
	template <typename VarType>
	class DAESolver : public Solver {
	protected:
		// General simulation parameters
        CPS::SystemTopology mSystem;
		/// Offsets vector for adding new equations to the residual vector
		std::vector<Int> mOffsets;
		/// Initial sim time
		Real mInitTime;
		/// Current simulation time
		Real mSimTime;
		/// Constant time step
		Real mTimestep;
		/// Number of equations in problem
		Int mNEQ;
		/// Components of the Problem
		CPS::DAEInterface::List mDAEComponents;
		/// Nodes of the Problem
		typename CPS::SimNode<VarType>::List mNodes;

		// IDA simulation variables
		/// Reference to a common simulation context object
		SUNContext mSunctx {nullptr};
		/// Memory block allocated by IDA
		void *mIDAMemoryBlock = NULL;
		/// Vector of problem variables
		N_Vector mStateVector = NULL;
		/// Derivates of the state vector with respect to time
		N_Vector mDerivativeStateVector = NULL;
		/// Vector to indicate differential(0.0) and algebraic(1.0) variable
		N_Vector mStateIDsVector = NULL;
		/// Time IDA reached while solving
		realtype mTimeReachedSolver;
		/// vector of absolute tolerances
		N_Vector mAbsoluteTolerances;
		/// Relative tolerance
		realtype mRelativeTolerance = 1e-6;
		/// Template Jacobian Matrix
		SUNMatrix mJacobianMatrix = NULL;
		/// Linear solver object
		SUNLinearSolver mLinearSolver = NULL;
        /// number of steps taken by ida
		long int mNumberStepsIDA = 0;
		/// number of calls to the user's res function
        long int mNumberCallsResidualFunctions=0;
		/// cumulative number of nonlinear iterations performed
		long int mNonLinearIters;
		/// integration step size taken on the last internal step
		realtype mLastIntegrationStep;
		/// Actual initial step size
		realtype mActualInitialStepSize;
		/// Step size to be attempted on the next step
		realtype mNextStepSize;
		/// vector of solution error weights at the current time
		N_Vector mErrorWeights;
		/// vector of estimated load errors at the current time
		N_Vector mEstLocalErrors;
		///
        std::vector<CPS::DAEInterface::ResFn> mResidualFunctions;

		/// Residual Function of entire System
		static int residualFunctionWrapper(realtype step_time, N_Vector state, N_Vector dstate_dt, N_Vector resid, void *user_data);
		int residualFunction(realtype step_time, N_Vector state, N_Vector dstate_dt, N_Vector resid);

	public:
		/// Create solve object with given parameters
		DAESolver(const String& name, 
			CPS::SystemTopology system, Real dt, Real t0, 
			CPS::Logger::Level logLevel = CPS::Logger::Level::info, 
			Real relTol=1e-4);
		/// Deallocate all memory
		~DAESolver();
		/// Set relative Tolerance
		void setRelativeTolerance(Real relTol) {
			mRelativeTolerance = relTol;
		}
		/// Initialization of individual components
		void initializeComponents();
		/// Initialization 
		void initialize();
		/// Solve system for the current time
		Real step(Real time);
		/// log weighted errors
		std::string printWeightedErrors();


		CPS::Task::List getTasks();

		class SolveStep : public CPS::Task {
		public:
			SolveStep(DAESolver& solver) :
				Task(solver.mName + ".SolveStep"), mSolver(solver) {
				mModifiedAttributes.push_back(Scheduler::external);
			}
			void execute(Real time, Int timeStepCount) {
    			mSolver.step(time);
			}
		private:
			DAESolver& mSolver;
		};

		///
		class LogTask : public CPS::Task {
		public:
			LogTask(DAESolver& solver) :
				Task(solver.mName + ".Log"), mSolver(solver) {
				mModifiedAttributes.push_back(Scheduler::external);
			}

			void execute(Real time, Int timeStepCount) {
				mSolver.log(time, timeStepCount);
			}

		private:
			DAESolver& mSolver;
		};
	};
}
