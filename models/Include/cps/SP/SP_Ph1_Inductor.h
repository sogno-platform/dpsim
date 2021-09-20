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
namespace SP {
namespace Ph1 {
	/// Static phasor inductor model
	class Inductor :
		public Base::Ph1::Inductor,
		public MNATearInterface,
		public SimPowerComp<Complex>,
		public SharedFactory<Inductor> {
	protected:
		/// susceptance [S]
		Complex mSusceptance;

	public:
		/// Defines UID, name and log level
		Inductor(String uid, String name, Logger::Level logLevel = Logger::Level::off);
		/// Defines name and log level
		Inductor(String name, Logger::Level logLevel = Logger::Level::off)
			: Inductor(name, name, logLevel) { }

		SimPowerComp<Complex>::Ptr clone(String name);

		// #### General ####
		/// Initializes component from power flow data
		void initializeFromNodesAndTerminals(Real frequency);
		// #### MNA section ####
		/// Initializes internal variables of the component
		void mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector);
		/// Stamps system matrix
		void mnaApplySystemMatrixStamp(Matrix& systemMatrix);
		/// Update interface voltage from MNA system results
		void mnaUpdateVoltage(const Matrix& leftVector);
		/// Update interface current from MNA system results
		void mnaUpdateCurrent(const Matrix& leftVector);
		/// MNA post step operations
		void mnaPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector);
		/// Add MNA post step dependencies
		void mnaAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector);

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

		void mnaTearApplyMatrixStamp(Matrix& tearMatrix);
	};
}
}
}
