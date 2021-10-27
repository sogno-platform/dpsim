/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <cps/SimPowerComp.h>
#include <cps/Solver/MNATearInterface.h>
#include <cps/Base/Base_Ph1_Inductor.h>

namespace CPS {
namespace DP {
namespace Ph1 {
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
		public SimPowerComp<Complex>,
		public SharedFactory<Inductor> {
	protected:
		/// DC equivalent current source for harmonics [A]
		MatrixComp mEquivCurrent;
		/// Equivalent conductance for harmonics [S]
		MatrixComp mEquivCond;
		/// Coefficient in front of previous current value for harmonics
		MatrixComp mPrevCurrFac;
		///
		void initVars(Real timeStep);
	public:
		/// Defines UID, name and log level
		Inductor(String uid, String name, Logger::Level logLevel = Logger::Level::off);
		/// Defines name and log level
		Inductor(String name, Logger::Level logLevel = Logger::Level::off)
			: Inductor(name, name, logLevel) { }

		// #### General ####
		/// Return new instance with the same parameters
		SimPowerComp<Complex>::Ptr clone(String name);
		/// Initializes state variables considering the number of frequencies
		void initialize(Matrix frequencies);
		/// Initializes states from power flow data
		void initializeFromNodesAndTerminals(Real frequency);

		// #### MNA section ####
		/// Initializes MNA specific variables
		void mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector);
		void mnaInitializeHarm(Real omega, Real timeStep, std::vector<Attribute<Matrix>::Ptr> leftVectors);
		/// Stamps system matrix
		void mnaApplySystemMatrixStamp(Matrix& systemMatrix);
		void mnaApplySystemMatrixStampHarm(Matrix& systemMatrix, Int freqIdx);
		/// Stamps right side (source) vector
		void mnaApplyRightSideVectorStamp(Matrix& rightVector);
		void mnaApplyRightSideVectorStampHarm(Matrix& rightVector);
		/// Update interface voltage from MNA system results
		void mnaUpdateVoltage(const Matrix& leftVector);
		void mnaUpdateVoltageHarm(const Matrix& leftVector, Int freqIdx);
		/// Update interface current from MNA system results
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

		// #### Tearing methods ####
		void mnaTearInitialize(Real omega, Real timestep);
		void mnaTearApplyMatrixStamp(Matrix& tearMatrix);
		void mnaTearApplyVoltageStamp(Matrix& voltageVector);
		void mnaTearPostStep(Complex voltage, Complex current);

		class MnaPreStep : public Task {
		public:
			MnaPreStep(Inductor& inductor) :
				Task(inductor.mName + ".MnaPreStep"), mInductor(inductor) {
					mInductor.mnaAddPreStepDependencies(mPrevStepDependencies, mAttributeDependencies, mModifiedAttributes);
			}
			void execute(Real time, Int timeStepCount) { mInductor.mnaPreStep(time, timeStepCount); };
		private:
			Inductor& mInductor;
		};

		class MnaPostStep : public Task {
		public:
			MnaPostStep(Inductor& inductor, Attribute<Matrix>::Ptr leftVector) :
				Task(inductor.mName + ".MnaPostStep"),
				mInductor(inductor), mLeftVector(leftVector) {
					mInductor.mnaAddPostStepDependencies(mPrevStepDependencies, mAttributeDependencies, mModifiedAttributes, mLeftVector);
			}
			void execute(Real time, Int timeStepCount) { mInductor.mnaPostStep(time, timeStepCount, mLeftVector); };
		private:
			Inductor& mInductor;
			Attribute<Matrix>::Ptr mLeftVector;
		};

		class MnaPreStepHarm : public Task {
		public:
			MnaPreStepHarm(Inductor& inductor)
				: Task(inductor.mName + ".MnaPreStepHarm"),
				mInductor(inductor) {
				// actually depends on L, but then we'd have to modify the system matrix anyway
				mModifiedAttributes.push_back(inductor.attribute("right_vector"));
				mPrevStepDependencies.push_back(inductor.attribute("v_intf"));
				mPrevStepDependencies.push_back(inductor.attribute("i_intf"));
			}
			void execute(Real time, Int timeStepCount);
		private:
			Inductor& mInductor;
		};

		class MnaPostStepHarm : public Task {
		public:
			MnaPostStepHarm(Inductor& inductor, const std::vector<Attribute<Matrix>::Ptr> &leftVectors)
				: Task(inductor.mName + ".MnaPostStepHarm"),
				mInductor(inductor), mLeftVectors(leftVectors) {
				for (UInt i = 0; i < mLeftVectors.size(); i++)
					mAttributeDependencies.push_back(mLeftVectors[i]);
				mModifiedAttributes.push_back(mInductor.attribute("v_intf"));
				mModifiedAttributes.push_back(mInductor.attribute("i_intf"));
			}
			void execute(Real time, Int timeStepCount);
		private:
			Inductor& mInductor;
			std::vector< Attribute<Matrix>::Ptr > mLeftVectors;
		};


	};
}
}
}
