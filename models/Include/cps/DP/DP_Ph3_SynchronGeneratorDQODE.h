/**
 * @file
 * @author Georg Reinke <georg.reinke@rwth-aachen.de>
 * @copyright 2017-2019, Institute for Automation of Complex Power Systems, EONERC
 *
 * CPowerSystems
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

#include <cps/DP/DP_Ph3_SynchronGeneratorDQ.h>

#ifdef WITH_SUNDIALS
  #include <arkode/arkode.h> // Prototypes for ARKode fcts., consts
  #include <nvector/nvector_serial.h> // Serial N_Vector types, fcts., macros
  #include <sundials/sundials_types.h>		/* def. of type 'realtype' */
  #include <sunmatrix/sunmatrix_dense.h>  /* access to dense SUNMatrix						*/
  #include <sunlinsol/sunlinsol_dense.h>  /* access to dense SUNLinearSolver		  */
  #include <arkode/arkode_direct.h>		   /* access to ARKDls interface				   */

  #include <cps/Solver/ODEInterface.h>
  // Needed to print the computed to the console
  #if defined(SUNDIALS_EXTENDED_PRECISION)
		#define GSYM "Lg"
		#define ESYM "Le"
		#define FSYM "Lf"
  #else
		#define GSYM "g"
		#define ESYM "e"
		#define FSYM "f"
  #endif
#endif

namespace CPS {
namespace DP {
namespace Ph3 {
	class SynchronGeneratorDQODE :
		public SynchronGeneratorDQ,
		public ODEInterface,
		public SharedFactory<SynchronGeneratorDQODE> {
	public:
		SynchronGeneratorDQODE(String uid, String name, Logger::Level loglevel = Logger::Level::off);
		SynchronGeneratorDQODE(String name, Logger::Level loglevel = Logger::Level::off);

		// #### MNA Section ####
		void mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector);

		class MnaPreStep : public Task {
		public:
			MnaPreStep(SynchronGeneratorDQODE& synGen)
				: Task(synGen.mName + ".MnaPreStep"), mSynGen(synGen) {
				mAttributeDependencies.push_back(synGen.attribute("ode_post_state"));
				mModifiedAttributes.push_back(synGen.attribute("right_vector"));
				mPrevStepDependencies.push_back(synGen.attribute("v_intf"));
			}

			void execute(Real time, Int timeStepCount);

		private:
			SynchronGeneratorDQODE& mSynGen;
		};

		class ODEPreStep : public Task {
		public:
			ODEPreStep(SynchronGeneratorDQODE& synGen)
			: Task(synGen.mName + ".ODEPreStep"), mSynGen(synGen) {
				mModifiedAttributes.push_back(synGen.attribute("ode_pre_state"));
				mModifiedAttributes.push_back(synGen.attribute("i_intf"));
			}

			void execute(Real time, Int timeStepCount);

		private:
			SynchronGeneratorDQODE& mSynGen;
		};

	protected:
		// #### ODE Section ####

		/// Defines the ODE-System which shall be solved
		void odeStateSpace(double t, const double y[], double ydot[]); // ODE-Class approach

		/// Jacobian corresponding to the StateSpace system, needed for implicit solve
		void odeJacobian(double t, const double y[], double fy[], double J[],
		                 double tmp1[], double tmp2[], double tmp3[]);

		void odePreStep();
		void odePostStep();

		// ### Variables for ODE-Solver interaction ###
		/// Number of differential variables
		int mDim;
	};
}
}
}
