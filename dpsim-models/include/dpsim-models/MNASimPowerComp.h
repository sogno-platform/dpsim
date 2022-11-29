// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <dpsim-models/SimPowerComp.h>
#include <dpsim-models/Solver/MNAInterface.h>

namespace CPS {

	/// Base class for all MNA components that are transmitting power.
	template <typename VarType>
	class MNASimPowerComp : public SimPowerComp<VarType>, public MNAInterface {

	public:
		typedef VarType Type;
		typedef std::shared_ptr<MNASimPowerComp<VarType>> Ptr;
		typedef std::vector<Ptr> List;

		/// Basic constructor that takes UID, name and log level
		MNASimPowerComp(String uid, String name, Logger::Level logLevel)
			: SimPowerComp<VarType>(uid, name, logLevel) { }

		/// Basic constructor that takes name and log level and sets the UID to name as well
		MNASimPowerComp(String name, Logger::Level logLevel = Logger::Level::off)
			: SimPowerComp<VarType>(name, name, logLevel) { }

		/// Destructor - does not do anything
		virtual ~MNASimPowerComp() = default;
		class MnaPreStep : public CPS::Task {
		public:
			MnaPreStep(MNASimPowerComp<VarType>& comp) :
				Task(**comp.mName + ".MnaPreStep"), mComp(comp) {
					mComp.mnaAddPreStepDependencies(mPrevStepDependencies, mAttributeDependencies, mModifiedAttributes);
			}
			void execute(Real time, Int timeStepCount) { mComp.mnaPreStep(time, timeStepCount); };

		private:
			MNASimPowerComp<VarType>& mComp;
		};

		class MnaPostStep : public CPS::Task {
		public:
			MnaPostStep(MNASimPowerComp<VarType>& comp, Attribute<Matrix>::Ptr leftVector) :
				Task(**comp.mName + ".MnaPostStep"), mComp(comp), mLeftVector(leftVector) {
				mComp.mnaAddPostStepDependencies(mPrevStepDependencies, mAttributeDependencies, mModifiedAttributes, mLeftVector);
			}
			void execute(Real time, Int timeStepCount) { mComp.mnaPostStep(time, timeStepCount, mLeftVector); };

		private:
			MNASimPowerComp<VarType>& mComp;
			Attribute<Matrix>::Ptr mLeftVector;
		};

	};
}
