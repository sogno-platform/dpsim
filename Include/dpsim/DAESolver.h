/** DAE Solver
 * 
 * @file
 * @author Markus Mirz <mmirz@eonerc.rwth-aachen.de>
 * @author Steffen Vogel <stvogel@eonerc.rwth-aachen.de>
 * @copyright 2017-2018, Institute for Automation of Complex Power Systems, EONERC
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

#pragma once

#include <iostream>
#include <vector>
#include <list>

#include <dpsim/Solver.h>

#include <cps/SystemTopology.h>
#include <cps/Solver/DAEInterface.h>
#include <cps/Logger.h>

#include <ida/ida.h>
#include <ida/ida_direct.h>
#include <sunlinsol/sunlinsol_dense.h>
#include <sundials/sundials_types.h>
#include <nvector/nvector_serial.h>


namespace DPsim {

	/// Solver class which uses Differential Algebraic Equation(DAE) systems
	class DAESolver : public Solver {
	protected:
		// General simulation parameters
        CPS::SystemTopology mSystem;
		/// Offsets vector for adding new equations to the residual vector
		std::vector<Int> mOffsets;
		/// Constant time step
		Real mTimestep;
		/// Number of equations in problem
		Int mNEQ;
		/// Components of the solver
        CPS::Component::List mComponents;
		///
        CPS::Node<Complex>::List mNodes;

		// IDA simulation variables
		/// Memory block allocated by IDA
		void *mem = NULL;
		/// Vector of problem variables
		N_Vector state = NULL;
		/// Derivates of the state vector with respect to time
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

        std::vector<CPS::DAEInterface::ResFn> mResidualFunctions;

		/// Residual Function of entire System
		static int residualFunctionWrapper(realtype ttime, N_Vector state, N_Vector dstate_dt, N_Vector resid, void *user_data);
		int residualFunction(realtype ttime, N_Vector state, N_Vector dstate_dt, N_Vector resid);
	
	public:
		/// Create solve object with given parameters
        DAESolver(String name, CPS::SystemTopology system, Real dt, Real t0);
		/// Deallocate all memory
		~DAESolver();
		/// Initialize Components & Nodes with inital values
		void initialize(Real t0);
		/// Solve system for the current time
		Real step(Real time);
	};
}
