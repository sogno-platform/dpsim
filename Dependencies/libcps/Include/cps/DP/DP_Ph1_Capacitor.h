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
#include <cps/Solver/MNAInterface.h>
#include <cps/Base/Base_Ph1_Capacitor.h>

namespace CPS {
namespace DP {
namespace Ph1 {
	/// \brief Capacitor model
	///
	/// The capacitor is represented by a DC equivalent circuit which
	/// corresponds to one iteration of the trapezoidal integration method.
	/// The equivalent DC circuit is a resistance in parallel with a current source.
	/// The resistance is constant for a defined time step and system
	/// frequency and the current source changes for each iteration.
	class Capacitor :
		public Base::Ph1::Capacitor,
		public MNAInterface,
		public PowerComponent<Complex>,
		public SharedFactory<Capacitor> {
	protected:
		/// DC equivalent current source for harmonics [A]
		MatrixComp mEquivCurrent;
		/// Equivalent conductance for harmonics [S]
		MatrixComp mEquivCond;
		/// Coefficient in front of previous voltage value for harmonics
		MatrixComp mPrevVoltCoeff;
	public:
		/// Defines UID, name and logging level
		Capacitor(String uid, String name, Logger::Level logLevel = Logger::Level::off);
		/// Defines name, component parameters and logging level
		Capacitor(String name, Logger::Level logLevel = Logger::Level::off)
			: Capacitor(name, name, logLevel) { }

		PowerComponent<Complex>::Ptr clone(String name);

		// #### General ####
		/// Initializes component from power flow data
		void initializeFromPowerflow(Real frequency);
		///
		void initialize(Matrix frequencies);

		// #### MNA section ####
		/// Initializes internal variables of the component
		void mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector);
		void mnaInitializeHarm(Real omega, Real timeStep, std::vector<Attribute<Matrix>::Ptr> leftVector);
		/// Stamps system matrix
		void mnaApplySystemMatrixStamp(Matrix& systemMatrix);
		void mnaApplySystemMatrixStampHarm(Matrix& systemMatrix, Int freqIdx);
		/// Stamps right side (source) vector
		void mnaApplyRightSideVectorStamp(Matrix& rightVector);
		void mnaApplyRightSideVectorStampHarm(Matrix& rightVector);
		/// Update interface voltage from MNA system result
		void mnaUpdateVoltage(const Matrix& leftVector);
		void mnaUpdateVoltageHarm(const Matrix& leftVector, Int freqIdx);
		void mnaApplyRightSideVectorStampHarm(Matrix& sourceVector, Int freqIdx);
		/// Update interface current from MNA system result
		void mnaUpdateCurrent(const Matrix& leftVector);
		void mnaUpdateCurrentHarm();

		class MnaPreStep : public Task {
		public:
			MnaPreStep(Capacitor& capacitor)
				: Task(capacitor.mName + ".MnaPreStep"), mCapacitor(capacitor) {
				// actually depends on C, but then we'd have to modify the system matrix anyway
				mModifiedAttributes.push_back(capacitor.attribute("right_vector"));
				mPrevStepDependencies.push_back(capacitor.attribute("i_intf"));
				mPrevStepDependencies.push_back(capacitor.attribute("v_intf"));
			}
			void execute(Real time, Int timeStepCount);
		private:
			Capacitor& mCapacitor;
		};

		class MnaPostStep : public Task {
		public:
			MnaPostStep(Capacitor& capacitor, Attribute<Matrix>::Ptr leftVector)
				: Task(capacitor.mName + ".MnaPostStep"), mCapacitor(capacitor), mLeftVector(leftVector) {
				mAttributeDependencies.push_back(mLeftVector);
				mModifiedAttributes.push_back(mCapacitor.attribute("v_intf"));
				mModifiedAttributes.push_back(mCapacitor.attribute("i_intf"));
			}
			void execute(Real time, Int timeStepCount);
		private:
			Capacitor& mCapacitor;
			Attribute<Matrix>::Ptr mLeftVector;
		};

		class MnaPreStepHarm : public CPS::Task {
		public:
			MnaPreStepHarm(Capacitor& capacitor)
				: Task(capacitor.mName + ".MnaPreStepHarm"),
				mCapacitor(capacitor) {
				// actually depends on C, but then we'd have to modify the system matrix anyway
				mModifiedAttributes.push_back(capacitor.attribute("right_vector"));
				mPrevStepDependencies.push_back(capacitor.attribute("i_intf"));
				mPrevStepDependencies.push_back(capacitor.attribute("v_intf"));
			}
			void execute(Real time, Int timeStepCount);
		private:
			Capacitor& mCapacitor;
		};

		class MnaPostStepHarm : public CPS::Task {
		public:
			MnaPostStepHarm(Capacitor& capacitor, std::vector<Attribute<Matrix>::Ptr> leftVectors)
				: Task(capacitor.mName + ".MnaPostStepHarm"),
				mCapacitor(capacitor), mLeftVectors(leftVectors) {
				for (UInt i = 0; i < mLeftVectors.size(); i++)
					mAttributeDependencies.push_back(mLeftVectors[i]);
				mModifiedAttributes.push_back(mCapacitor.attribute("v_intf"));
				mModifiedAttributes.push_back(mCapacitor.attribute("i_intf"));
			}
			void execute(Real time, Int timeStepCount);
		private:
			Capacitor& mCapacitor;
			std::vector< Attribute<Matrix>::Ptr > mLeftVectors;
		};
	};
}
}
}
