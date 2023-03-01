/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim-models/MNASimPowerComp.h>
#include <dpsim-models/Base/Base_Ph1_Capacitor.h>

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
		public MNASimPowerComp<Complex>,
		public Base::Ph1::Capacitor,
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
		void mnaCompInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) override;
		void mnaCompInitializeHarm(Real omega, Real timeStep, std::vector<Attribute<Matrix>::Ptr> leftVector) override;
		/// Stamps system matrix
		void mnaCompApplySystemMatrixStamp(SparseMatrixRow& systemMatrix);
		void mnaCompApplySystemMatrixStampHarm(SparseMatrixRow& systemMatrix, Int freqIdx);
		/// Stamps right side (source) vector
		void mnaCompApplyRightSideVectorStamp(Matrix& rightVector) override;
		void mnaCompApplyRightSideVectorStampHarm(Matrix& rightVector) override;
		void mnaCompApplyRightSideVectorStampHarm(Matrix& sourceVector, Int freqIdx) override;
		/// Update interface voltage from MNA system result
		void mnaCompUpdateVoltage(const Matrix& leftVector) override;
		void mnaCompUpdateVoltageHarm(const Matrix& leftVector, Int freqIdx);
		/// Update interface current from MNA system result
		void mnaCompUpdateCurrent(const Matrix& leftVector) override;
		void mnaCompUpdateCurrentHarm();
		/// MNA pre step operations
		void mnaCompPreStep(Real time, Int timeStepCount) override;
		/// MNA post step operations
		void mnaCompPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) override;
		/// Add MNA pre step dependencies
		void mnaCompAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) override;
		/// Add MNA post step dependencies
		void mnaCompAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector) override;

		class MnaPreStepHarm : public CPS::Task {
		public:
			MnaPreStepHarm(Capacitor& capacitor)
				: Task(**capacitor.mName + ".MnaPreStepHarm"),
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
				: Task(**capacitor.mName + ".MnaPostStepHarm"),
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
