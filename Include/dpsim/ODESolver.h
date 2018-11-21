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

#ifdef WITH_SUNDIALS

#include <dpsim/Solver.h>
#include <cps/SystemTopology.h>
#include <cps/Logger.h>

#include <arkode/arkode.h>              // prototypes for ARKode fcts., consts. and includes sundials_types.h
#include <nvector/nvector_serial.h>     // access to serial N_Vector
#include <sunmatrix/sunmatrix_dense.h>  // access to dense SUNMatrix
#include <sunlinsol/sunlinsol_dense.h>  // access to dense SUNLinearSolver
#include <arkode/arkode_direct.h>       // access to ARKDls interface

#if defined(SUNDIALS_EXTENDED_PRECISION)
#define GSYM "Lg"
#define ESYM "Le"
#define FSYM "Lf"
#else
#define GSYM "g"
#define ESYM "e"
#define FSYM "f"
#endif // defined(SUNDIALS_EXTENDED_PRECISION)

using namespace CPS;

namespace DPsim {
	/// Solver class for ODE (Ordinary Differential Equation) systems
	class ODESolver: public Solver {
	protected:
		/// Constant time step
		Real mTimestep;
		/// Components of the solver
		Component::List mComponents;

		std::vector<ODEInterface::StSpFn> mStSpFunctions; // Really needed?

		// ### General problem variables ###
		// For every component we get an additional entry in the vector
		std::vector<sunindextype> mProbDims;
		std::vector<N_Vector> mStates;
		std::vector<void*> mArkode_mems;

		/// Dimension of differential variables
  	//	sunindextype mProbDim;
		// reusable error-checking flag
		int flag;
		// vector for differential variables
	/*	N_Vector y = NULL;
		// ARKode memory structure
		void *arkode_mem = NULL;*/
		/// Template Jacobian Matrix (implicit solver)
	/*	SUNMatrix A = NULL;
		/// Linear solver object (implicit solver)
		SUNLinearSolver LS = NULL; */
		// Keep tolerance for each component equal
		/// Relative tolerance
		realtype reltol = RCONST(1.0e-6);
		/// Scalar absolute tolerance
		realtype abstol = RCONST(1.0e-10);

	public:
		/// Create solve object with given parameters
		ODESolver(String name, SystemTopology system, Real dt, Real t0);
		/// Deallocate all memory
		~ODESolver();

		/// use wrappers similar to DAE_Solver
    	static int StateSpaceWrapper(realtype t, N_Vector y, N_Vector ydot, void *user_data);
    	// neeeded for implicit solve:
    	static int JacobianWrapper(realtype t, N_Vector y, N_Vector fy, SUNMatrix J, void *user_data,
            N_Vector tmp1, N_Vector tmp2, N_Vector tmp3);
    	/// ARKode- standard error detection function; in DAE-solver not detection function is used -> for efficiency purposes?
		static int check_flag(void *flagvalue, const std::string funcname, int opt);
		///
    	Real step(Real initial_time);
	};
}
#endif // WITH_SUNDIALS
