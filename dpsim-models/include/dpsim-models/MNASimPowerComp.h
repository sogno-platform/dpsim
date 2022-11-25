// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <dpsim-models/SimPowerComp.h>
#include <dpsim-models/Solver/MNAInterface.h>

namespace CPS {
	/// Base class for all components that are transmitting power.
	template <typename VarType>
	class MNASimPowerComp : public SimPowerComp<VarType>, public MNAInterface {

	protected:
		MNAInterface::List mSubcomponentsBeforePreStep;
		MNAInterface::List mSubcomponentsAfterPreStep;
		MNAInterface::List mSubcomponentsBeforePostStep;
		MNAInterface::List mSubcomponentsAfterPostStep;

	public:
		typedef VarType Type;
		typedef std::shared_ptr<MNASimPowerComp<VarType>> Ptr;
		typedef std::vector<Ptr> List;

		enum class MNA_SUBCOMP_TASK_ORDER {
			NO_TASK,
			TASK_BEFORE_PARENT,
			TASK_AFTER_PARENT
		};

		/// Basic constructor that takes UID, name and log level
		MNASimPowerComp<VarType>::MNASimPowerComp(String uid, String name, Logger::Level logLevel)
			: SimPowerComp<VarType>(uid, name, logLevel) { }

		/// Basic constructor that takes name and log level and sets the UID to name as well
		MNASimPowerComp(String name, Logger::Level logLevel = Logger::Level::off)
			: SimPowerComp<VarType>(name, name, logLevel) { }

		/// Destructor - does not do anything
		virtual ~MNASimPowerComp() { }

		/// @brief Add a new subcomponent implementing MNA methods
		/// @param subc The new subcomponent
		/// @param preStepOrder When to execute the subcomponent's pre-step in relation to the parent
		/// @param postStepOrder When to execute the subcomponent's post-step in relation to the parent
		void addMNASubComponent(typename MNASimPowerComp<VarType>::Ptr subc, MNA_SUBCOMP_TASK_ORDER preStepOrder, MNA_SUBCOMP_TASK_ORDER postStepOrder);

		// #### MNA Interface Functions ####
		/// Initializes variables of components
		virtual void mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) override;
		/// Stamps system matrix
		virtual void mnaApplySystemMatrixStamp(Matrix& systemMatrix) override;
		/// Stamps right side (source) vector
		virtual void mnaApplyRightSideVectorStamp(Matrix& rightVector) override;
		/// MNA pre step operations
		virtual void mnaPreStep(Real time, Int timeStepCount) override;
		/// MNA post step operations
		virtual void mnaPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) override;
		/// Add MNA pre step dependencies
		virtual void mnaAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) override;
		/// Add MNA post step dependencies
		virtual void mnaAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector) override;

		// #### MNA Parent Functions ####
		virtual void mnaParentInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) { };
		virtual void mnaParentApplySystemMatrixStamp(Matrix& systemMatrix) { };
		virtual void mnaParentApplyRightSideVectorStamp(Matrix& rightVector) { };
		virtual void mnaParentPreStep(Real time, Int timeStepCount) { };
		virtual void mnaParentPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) { };
		virtual void mnaParentAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) { };
		virtual void mnaParentAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector) { };
	};
}
