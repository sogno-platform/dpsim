/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim-models/MNASimPowerComp.h>
#include <dpsim-models/Solver/MNAInterface.h>
#include <dpsim-models/Base/Base_Ph1_Transformer.h>

namespace CPS {
namespace DP {
namespace Ph1 {
	/// Transformer that includes an inductance and resistance
	class Transformer :
		public MNASimPowerComp<Complex>,
		public SharedFactory<Transformer>,
		public Base::Ph1::Transformer {

	public:
		/// Defines UID, name and logging level
		Transformer(String uid, String name,
			Logger::Level logLevel = Logger::Level::off);
		/// Defines name and logging level
		Transformer(String name, Logger::Level logLevel = Logger::Level::off)
			: Transformer(name, name, logLevel) { }

		/// DC equivalent current source for harmonics [A]
		MatrixComp mEquivCurrent;
		/// Equivalent conductance for harmonics [S]
		MatrixComp mEquivCond;
		/// Coefficient in front of previous current value for harmonics
		MatrixComp mPrevCurrFac;

	public:
		// #### General ####
		/// Defines component parameters
		void setParameters(Real nomVoltageEnd1, Real nomVoltageEnd2, Real ratioAbs, Real ratioPhase, Real resistance, Real inductance);
		/// Initializes component from power flow data
		void initializeFromNodesAndTerminals(Real frequency);
		/// Initializes state variables considering the number of frequencies
		void initialize(Matrix frequencies);
		/// 
		void initVars(Real timeStep);


		// #### MNA section ####
		/// Initializes internal variables of the component
		void mnaCompInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) override;
		///
		void mnaCompInitializeHarm(Real omega, Real timeStep, std::vector<Attribute<Matrix>::Ptr> leftVectors) override;
		/// Stamps system matrix
		void mnaCompApplySystemMatrixStamp(SparseMatrixRow& systemMatrix) override;
		/// Stamps system matrix
		void mnaCompApplySystemMatrixStampHarm(SparseMatrixRow& systemMatrix , int freqIdx) override;
		/// Updates internal current variable of the component
		void mnaCompUpdateCurrent(const Matrix& leftVector) override;
		/// Updates internal current variable of the component
		void mnaCompUpdateCurrentHarm();
		/// Updates internal voltage variable of the component
		void mnaCompUpdateVoltage(const Matrix& leftVector) override;
		/// Updates internal voltage variable of the component for harmonics
		void mnaCompUpdateVoltageHarm(const Matrix& leftVector, int freqIdx);
		/// MNA pre step operations
		void mnaCompPreStep(Real time, Int timeStepCount) override;
		/// MNA pre step operations
		void mnaCompPreStepHarm(Real time, Int timeStepCount);
		/// MNA post step operations
		void mnaCompPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) override;
		/// MNA post step operations
		void mnaPostStepHarm(Real time, Int timeStepCount) ;
		/// Stamps right side (source) vector
		void mnaCompApplyRightSideVectorStamp(Matrix& rightVector) override;
		/// Stamps right side (source) vector
		void mnaCompApplyRightSideVectorStampHarm(Matrix& rightVector) override;
		/// Add MNA pre step dependencies
		void mnaCompAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) override;
		/// Add MNA post step dependencies
		void mnaCompAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector) override;

		//
		class MnaPreStepHarm : public Task {
		public:
			MnaPreStepHarm(Transformer& transformer)
				: Task(**transformer.mName + ".MnaPreStepHarm"),
				mTransformer(transformer) {
				// actually depends on L, but then we'd have to modify the system matrix anyway
				mModifiedAttributes.push_back(mTransformer.attribute("right_vector"));
				mPrevStepDependencies.push_back(mTransformer.attribute("v_intf"));
				mPrevStepDependencies.push_back(mTransformer.attribute("i_intf"));
			}
			void execute(Real time, Int timeStepCount);
		private:
			Transformer& mTransformer;
		};

		//
		class MnaPostStepHarm : public Task {
		public:
			MnaPostStepHarm(Transformer& transformer, const std::vector<Attribute<Matrix>::Ptr> &leftVectors)
				: Task(**transformer.mName + ".MnaPostStepHarm"),
				mTransformer(transformer), mLeftVectors(leftVectors) {
				for (UInt i = 0; i < mLeftVectors.size(); i++)
					mAttributeDependencies.push_back(mLeftVectors[i]);
				mModifiedAttributes.push_back(mTransformer.attribute("v_intf"));
				mModifiedAttributes.push_back(mTransformer.attribute("i_intf"));
			}
			void execute(Real time, Int timeStepCount);
		private:
			Transformer& mTransformer;
			std::vector< Attribute<Matrix>::Ptr > mLeftVectors;
		};
	};
}
}
}
