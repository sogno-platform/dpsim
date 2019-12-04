/**
 * @file
 * @author Markus Mirz <mmirz@eonerc.rwth-aachen.de>
 * @copyright 2017-2018, Institute for Automation of Complex Power Systems, EONERC
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

#include <cps/PowerComponent.h>
#include <cps/Solver/MNATearInterface.h>
#include <cps/Base/Base_Ph1_Inductor.h>

namespace CPS {
namespace DP {
namespace Ph3 {
	/// \brief Inductor
	///
	/// The inductor is represented by a DC equivalent circuit which corresponds to
	/// one iteration of the trapezoidal integration method.
	/// The equivalent DC circuit is a resistance in parallel with a current source.
	/// The resistance is constant for a defined time step and system
	/// frequency and the current source changes for each iteration.
	class Inductor :
		public Base::Ph1::Inductor,
		public MNATearInterface,
		public PowerComponent<Complex>,
		public SharedFactory<Inductor> {
	protected:
		/// DC equivalent current source [A]
		MatrixComp mEquivCurrent;
		/// Equivalent conductance [S]
		MatrixComp mEquivCond;
		/// Coefficient in front of previous current value
		MatrixComp mPrevCurrFac;

		void initVars(Real omega, Real timeStep);
	public:
		/// Defines UID, name, component parameters and logging level
		Inductor(String uid, String name, Logger::Level logLevel = Logger::Level::off);
		/// Defines name, component parameters and logging level
		Inductor(String name, Logger::Level logLevel = Logger::Level::off)
			: Inductor(name, name, logLevel) { }
		/// Defines name, component parameters and logging level
		Inductor(String name, Real inductance,
			Logger::Level logLevel = Logger::Level::off);

		PowerComponent<Complex>::Ptr clone(String name);

		// #### General ####
		/// Initializes component from power flow data
		void initializeFromPowerflow(Real frequency);
		// #### MNA section ####
		/// Initializes internal variables of the component
		void mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector);
		/// Stamps system matrix
		void mnaApplySystemMatrixStamp(Matrix& systemMatrix);
		/// Stamps right side (source) vector
		void mnaApplyRightSideVectorStamp(Matrix& rightVector);
		/// Upgrade values in the source vector and maybe system matrix before MNA solution
		void mnaStep(Matrix& systemMatrix, Matrix& rightVector, Matrix& leftVector, Real time);
		/// Upgrade internal variables after MNA solution
		void mnaMnaPostStep(Matrix& rightVector, Matrix& leftVector, Real time);
		/// Update interface voltage from MNA system result
		void mnaUpdateVoltage(const Matrix& leftVector);
		/// Update interface current from MNA system result
		void mnaUpdateCurrent(const Matrix& leftVector);

		class MnaPreStep : public Task {
		public:
			MnaPreStep(Inductor& inductor) :
				Task(inductor.mName + ".MnaPreStep"), mInductor(inductor) {
				// actually depends on L, but then we'd have to modify the system matrix anyway
				mModifiedAttributes.push_back(inductor.attribute("right_vector"));
				mPrevStepDependencies.push_back(inductor.attribute("v_intf"));
				mPrevStepDependencies.push_back(inductor.attribute("i_intf"));
			}

			void execute(Real time, Int timeStepCount);

		private:
			Inductor& mInductor;
		};

		class MnaPostStep : public Task {
		public:
			MnaPostStep(Inductor& inductor, Attribute<Matrix>::Ptr leftVector) :
				Task(inductor.mName + ".MnaPostStep"), mInductor(inductor), mLeftVector(leftVector) {
				mAttributeDependencies.push_back(mLeftVector);
				mModifiedAttributes.push_back(mInductor.attribute("v_intf"));
				mModifiedAttributes.push_back(mInductor.attribute("i_intf"));
			}

			void execute(Real time, Int timeStepCount);

		private:
			Inductor& mInductor;
			Attribute<Matrix>::Ptr mLeftVector;
		};

		void mnaTearInitialize(Real omega, Real timestep);
		void mnaTearApplyMatrixStamp(Matrix& tearMatrix);
		void mnaTearApplyVoltageStamp(Matrix& voltageVector);
		void mnaTearPostStep(Complex voltage, Complex current);
	};
}
}
}
