
#pragma once 
#include <iostream>
#include <vector>
#include <list>
#include "Solver.h"
#include "cps/SystemTopology.h"
//#include "Logger.h"
#include <ida/ida.h>
#include <ida/ida_direct.h>
#include <sunlinsol/sunlinsol_dense.h>
#include <sundials/sundials_types.h>
#include <nvector/nvector_serial.h>

#define NVECTOR_DATA(vec) NV_DATA_S (vec) // Returns pointer to the first element of array vec

using namespace DPsim ;

	/// Solver class which uses Differential Algebraic Equation(DAE) systems
	class DAESolver : public Solver{
	protected:
		// General simulation parameters
		/// Local copy of the SystemTopology 
		SystemTopology DAESys;
		/// Offsets vector for adding new equations to the residual vector
		std::vector<int> offsets; 
		/// Constant time step
		Real mTimestep;
		/// Number of equations in problem
		int NEQ;

		//IDA simulation variables
		/// Memory block allocated by IDA
		void *mem = NULL;
		/// Vector of problem variables
		N_Vector state = NULL, 
		///  Derivates of the state vector with respect to time
		N_Vector dstate_dt = NULL; 
		/// Time IDA reached while solving
		realtype tret; 
		/// Scalar absolute tolerance
		realtype abstol;
		/// Relative tolerance
		realtype rtol;
		/// Template Jacobian Matrix
		SUNMatrix A = NULL;
		/// Linear solver object
		SUNLinearSolver LS = NULL;
	

	public:
		/// Create solve object with given parameters
		DAESolver(String name,  SystemTopology system, Real dt);
		virtual ~DAESolver();
		/// Initialize Components & Nodes with inital values
		void initialize(Component::List comps);
		/// Residual Function of entire System
		int DAE_residualFunction(realtype ttime, N_Vector state, N_Vector dstate_dt, N_Vector resid, void *user_data);
		/// Solve system for the current time
		Real step(Real time);
	};


