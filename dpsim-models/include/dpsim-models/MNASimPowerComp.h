// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <dpsim-models/SimPowerComp.h>
#include <dpsim-models/Solver/MNAInterface.h>

namespace CPS {

	/// Base class for all MNA components that are transmitting power.
	template <typename VarType>
	class MNASimPowerComp : public SimPowerComp<VarType>, public MNAInterface {

	private:
		Bool mHasPreStep;
		Bool mHasPostStep;

	public:
		using Type = VarType;
		using Ptr = std::shared_ptr<MNASimPowerComp<VarType>>;
		using List = std::vector<Ptr>;

		/// This component's contribution ("stamp") to the right-side vector.
		/// TODO performance improvements from a sparse representation, at least during copying / summation?
		Attribute<Matrix>::Ptr mRightVector;

		/// List of tasks that relate to using MNA for this component (usually pre-step and/or post-step)
		Task::List mMnaTasks;

		/// Basic constructor that takes UID, name and log level
		MNASimPowerComp(String uid, String name, Logger::Level logLevel)
			: SimPowerComp<VarType>(uid, name, logLevel), mRightVector(IdentifiedObject::mAttributes->createDynamic<Matrix>("right_vector")) { };

		/// Basic constructor that takes name and log level and sets the UID to name as well
		explicit MNASimPowerComp(String name, Logger::Level logLevel = Logger::Level::off)
			: MNASimPowerComp<VarType>(name, name, logLevel) { };

		/// Destructor - does not do anything
		virtual ~MNASimPowerComp() = default;

		void mnaInitialize(Real omega, Real timeStep) final;
		void mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) override;
		void mnaApplySystemMatrixStamp(Matrix& systemMatrix) override;
		void mnaApplySparseSystemMatrixStamp(SparseMatrixRow& systemMatrix) override;
		void mnaApplyRightSideVectorStamp(Matrix& rightVector) override;
		void mnaUpdateVoltage(const Matrix& leftVector) override;
		void mnaUpdateCurrent(const Matrix& leftVector) override;
		void mnaPreStep(Real time, Int timeStepCount) override;
		void mnaPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) override;
		void mnaAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) override;
		void mnaAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector) override;
		void mnaInitializeHarm(Real omega, Real timeStep, std::vector<Attribute<Matrix>::Ptr> leftVector) override;
		void mnaApplySystemMatrixStampHarm(Matrix& systemMatrix, Int freqIdx) override;
		void mnaApplyRightSideVectorStampHarm(Matrix& sourceVector) override;
		void mnaApplyRightSideVectorStampHarm(Matrix& sourceVector, Int freqIdx) override;
		const Task::List& mnaTasks() final;
		const Attribute<Matrix>::Ptr getRightVector() final;

		class MnaPreStep : public CPS::Task {
		public:
			explicit MnaPreStep(MNASimPowerComp<VarType>& comp) :
				Task(**comp.mName + ".MnaPreStep"), mComp(comp) {
					mComp.mnaAddPreStepDependencies(mPrevStepDependencies, mAttributeDependencies, mModifiedAttributes);
			}
			void execute(Real time, Int timeStepCount) override {
				mComp.mnaPreStep(time, timeStepCount);
			};

		private:
			MNASimPowerComp<VarType>& mComp;
		};

		class MnaPostStep : public CPS::Task {
		public:
			MnaPostStep(MNASimPowerComp<VarType>& comp, Attribute<Matrix>::Ptr leftVector) :
				Task(**comp.mName + ".MnaPostStep"), mComp(comp), mLeftVector(leftVector) {
				mComp.mnaAddPostStepDependencies(mPrevStepDependencies, mAttributeDependencies, mModifiedAttributes, mLeftVector);
			}
			void execute(Real time, Int timeStepCount) override {
				mComp.mnaPostStep(time, timeStepCount, mLeftVector);
			};

		private:
			MNASimPowerComp<VarType>& mComp;
			Attribute<Matrix>::Ptr mLeftVector;
		};

	};
}
