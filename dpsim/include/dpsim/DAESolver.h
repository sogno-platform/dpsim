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
#include <nvector/nvector_serial.h>
#include <sunlinsol/sunlinsol_dense.h>
#include <sunmatrix/sunmatrix_sparse.h>    /* access to sparse SUNMatrix           */
// #include <sunlinsol/sunlinsol_superlumt.h> /* access to SuperLUMT linear solver */
#include <sundials/sundials_types.h>

namespace DPsim {

	/// Solver class which uses Differential Algebraic Equation(DAE) systems
	template <typename VarType>
	class DAESolver : public Solver {
	private:
		// General simulation parameters/variables
		///
        CPS::SystemTopology mSystem;
		/// Offsets vector for adding new equations to the residual vector
		std::vector<Int> mOffsets;
		/// Initial sim time
		Real mInitTime;
		/// Current simulation time
		Real mSimTime;
		/// time intervat at which a computed solution is desired
		Real mTimestep;
		/// Number of equations in problem
		Int mNEQ;
		/// Components of the Problem
		CPS::DAEInterface::List mDAEComponents;
		/// Nodes of the Problem
		typename CPS::SimNode<VarType>::List mNodes;

		// IDA simulator variables
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
		/// Template Jacobian Matrix
		SUNMatrix mJacobianMatrix = NULL;
		/// Linear solver object
		SUNLinearSolver mLinearSolver = NULL;
		///
        std::vector<CPS::DAEInterface::ResFn> mResidualFunctions;
		///
        std::vector<CPS::DAEInterface::JacobianFn> mJacobianFunctions;

		// Solver parameters
		/// Relative tolerance
		realtype mRelativeTolerance = 1e-4;

		// ### nonlinear solver interface optional input parameters ###
		/// specifies maximum no. of nonlinear iterations
		int mIDAMaxNonlinIters = 4;
		/// specifies the maximum number of nonlinear solver convergence failures in one step
		int mIDAMaxConvFails = 10;
		/// Coeff. in the nonlinear convergence test
		realtype mIDANonlinConvCoef = 0.33;
			
		// ### Solver optional input parameters ###
		/// Maximum order for BDF method (0<mIDAMaxBDFOrder<=5)
		unsigned int mIDAMaxBDFOrder = 5;
		/// Maximum number of steps to be taken by the solver in its attempt to reach the next output time (-1 = unlimited)
		int mIDAMaxNumSteps = -1;
		/// indicates whether or not to use variable integration step size
		bool mVariableStepSize = true;
		/// min integration time
		realtype mMinStepSize = 1e-6;
		/// max integration time
		realtype mMaxStepSize = 1e-3;
		/// Maximum number of error test failures in attempting one step
		unsigned int mIDAMaxErrTestFails = 100;
		/// indicates whether or not to suppress algebraic variables in the local error test
		bool mIDASetSuppressAlg = SUNTRUE;

		// Variables to store ida statistics
        /// number of steps taken by ida
		long int mNumberStepsIDA = 0;
		/// number of calls to the user's res function
        long int mNumberCallsResidualFunctions = 0;
		/// cumulative number of nonlinear iterations performed
		long int mNonLinearIters;
		/// cumulative number of calls made to the linear solver’s setup function (total so far)
		long int mNumLinSolvSetups = 0;
		/// cumulative number of local error test failures that have occurred (total so far)
		long int mNumErrTestFails = 0;
		/// number of failed steps due to a nonlinear solver failure
		long int mNumStepSolveFails = 0;
		/// integration step size taken on the last internal step
		realtype mLastIntegrationStep;
		/// Step size to be attempted on the next step
		realtype mNextIntegrationStep;
		/// Actual initial step size
		realtype mActualInitialStepSize;
		/// Current internal time reached by the solver
		realtype mCurrentInternalTime;
		/// Order used during the last step
		int mOrderLastStep;
		/// Order to be attempted on the next step
		int mOrderNextStep;
		/// vector of solution error weights at the current time
		N_Vector mErrorWeights;
		/// vector of estimated load errors at the current time
		N_Vector mEstLocalErrors;
		
		

		/// Residual Function of entire System
		static int residualFunctionWrapper(realtype current_time, N_Vector state, 
									N_Vector dstate_dt, N_Vector resid, void *user_data);
		int residualFunction(realtype current_time, N_Vector state, N_Vector dstate_dt, 
							 N_Vector resid, void *user_data);

		/// Jacobian matrix calculation
		static int jacobianFunctionWrapper(realtype current_time, realtype cj, N_Vector state, 
								N_Vector dstate_dt, N_Vector res_vec, SUNMatrix JJ, void *user_data,
    							N_Vector tempv1, N_Vector tempv2, N_Vector tempv3);
		int calculateJacobianMatrix(realtype current_time, realtype cj, N_Vector state, 
								N_Vector dstate_dt, SUNMatrix JJ);

		/// Initialization of individual components
		void initializeComponents();

		// print and get errors
		/// log weighted errors
		void printEstimatedAndWeightedErrors(std::string& ret);

	public:
		/// Create solve object with given parameters
		DAESolver(const String& name, 
			CPS::SystemTopology system, Real dt, Real t0, 
			CPS::Logger::Level logLevel = CPS::Logger::Level::info);
		/// Deallocate all memory
		~DAESolver();
		
		/// Initialization 
		void initialize();
		/// Solve system for the current time
		Real step(Real time);
		///
		void updateVoltageAndCurrents();
		/// log IDA statistics
		void logStatistics(CPS::Logger::Level minLogLevel = CPS::Logger::Level::debug);
		

		/// ### Setters ###
		/// Set relative Tolerance
		void setRelativeTolerance(Real relTol) { mRelativeTolerance = relTol; }

		// ### Setters for nonlinear solver interface optional input functions ###
		/// Set maximum no. of convergence failures
		void setIDAMaxConvFails(int IDAMaxConvFails) { mIDAMaxConvFails = IDAMaxConvFails; }
		/// Set the safety factor in the nonlinear convergence test
		void setIDANonlinConvCoef(Real IDANonlinConvCoef) { mIDANonlinConvCoef = IDANonlinConvCoef; }
		/// Set the maximum number of nonlinear solver iterations in one solve attempt
		void setIDAMaxNonlinIters(int IDAMaxNonlinIters) { mIDAMaxNonlinIters = IDAMaxNonlinIters; }

		// ### Setters for main solver optional input functions ###
		/// Set maximum order for BDF method
		void setIDAMaxBDFOrder(unsigned int IDAMaxBDFOrder) {
			if (IDAMaxBDFOrder>5 || IDAMaxBDFOrder==0)
				IDAMaxBDFOrder = 5;
			mIDAMaxBDFOrder = IDAMaxBDFOrder;
		}
		/// Set maximum no. of internal steps before tout
		void setIDAMaxNumSteps(int IDAMaxNumSteps) { mIDAMaxNumSteps = IDAMaxNumSteps; }
		/// Set flag to specifi whether or not to use variable integration step size
		void setVariableStepSize(bool variableStepSize) { mVariableStepSize = variableStepSize; }
		/// Set min integration time
		void setMinStepSize(Real minStepSize) { mMinStepSize = minStepSize; }
		/// Set max integration time 
		void setMaxStepSize(Real maxStepSize) { mMaxStepSize = maxStepSize; }
		/// Set max integration time
		void setMaxErrTestFails(unsigned int IDAMaxErrTestFails) { mIDAMaxErrTestFails = IDAMaxErrTestFails; }
		/// Set Suppress alg. vars. from error test
		void setIDASetSuppressAlg(bool IDASetSuppressAlg) { mIDASetSuppressAlg = IDASetSuppressAlg; }


		// ### Solver tasks ###
		CPS::Task::List getTasks();

		class SolveStep : public CPS::Task {
		public:
			SolveStep(DAESolver& solver) :
				Task(solver.mName + ".SolveStep"), mSolver(solver) {
				mModifiedAttributes.push_back(Scheduler::external);
			}
			void execute(Real time, Int timeStepCount) {
				if (time == mSolver.mInitTime)
					// when time==initTime only log the initial values
					mSolver.updateVoltageAndCurrents();
				else
					
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
