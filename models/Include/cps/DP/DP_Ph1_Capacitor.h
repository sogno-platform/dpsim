/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <cps/SimPowerComp.h>
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
		public SimPowerComp<Complex>,
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

		SimPowerComp<Complex>::Ptr clone(String name);

		// #### General ####
		/// Initializes component from power flow data
		void initializeFromNodesAndTerminals(Real frequency);
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
		/// MNA pre step operations
		void mnaPreStep(Real time, Int timeStepCount);
		/// MNA post step operations
		void mnaPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector);
		/// Add MNA pre step dependencies
		void mnaAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes);
		/// Add MNA post step dependencies
		void mnaAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector);

		class MnaPreStep : public Task {
		public:
			MnaPreStep(Capacitor& capacitor)
				: Task(capacitor.mName + ".MnaPreStep"), mCapacitor(capacitor) {
					mCapacitor.mnaAddPreStepDependencies(mPrevStepDependencies, mAttributeDependencies, mModifiedAttributes);
			}
			void execute(Real time, Int timeStepCount) { mCapacitor.mnaPreStep(time, timeStepCount); };
		private:
			Capacitor& mCapacitor;
		};

		class MnaPostStep : public Task {
		public:
			MnaPostStep(Capacitor& capacitor, Attribute<Matrix>::Ptr leftVector)
				: Task(capacitor.mName + ".MnaPostStep"), mCapacitor(capacitor), mLeftVector(leftVector) {
					mCapacitor.mnaAddPostStepDependencies(mPrevStepDependencies, mAttributeDependencies, mModifiedAttributes, mLeftVector);
			}
			void execute(Real time, Int timeStepCount) { mCapacitor.mnaPostStep(time, timeStepCount, mLeftVector); };
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
			MnaPostStepHarm(Capacitor& capacitor, const std::vector<Attribute<Matrix>::Ptr> &leftVectors)
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
