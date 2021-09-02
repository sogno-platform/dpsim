/* Copyright 2017-2020 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <cps/SimPowerComp.h>
#include <cps/Solver/MNAVariableCompInterface.h>
#include <cps/Solver/MNASwitchInterface.h>
// #include <cps/Solver/MNAInterface.h>
#include <cps/Base/Base_Ph1_Switch.h>

namespace CPS {
namespace DP {
namespace Ph1 {
	/// \brief
	///
	/// Switch with variable resistance to avoid numerical oscillations, when an inductive current is suddenly interrupted.
	/// It is useful as a fault switch especially on a faulted generator or transformer winding.

	/// The switch resistance changes at a defined fixed rate by multiplying previous resistance value with a factor for the rate of change
	/// The MNA variable component interface is used to recompute the system Matrix after each change.
	class varResSwitch :
		public Base::Ph1::Switch,
		public MNAVariableCompInterface,
		public MNASwitchInterface,
		public SimPowerComp<Complex>,
		public SharedFactory<varResSwitch> {
	
	protected:

		Bool mPrevState=false;
		Real mDeltaResClosed = 0; 
		Real mDeltaResOpen = 1.5;
		Real mPrevRes; // previous resistance value to multiply with rate of change
		// because we change the base value mClosedResistance & mOpenResistance to recompute the system Matrix
		// we need to save the initialisation values to use them as target values in the transition
		Real mInitClosedRes;
		Real mInitOpenRes;

		

	public:
		
		void setInitParameters(Real timestep);
		
		/// Defines UID, name and log level
		varResSwitch(String uid, String name, Logger::Level logLevel = Logger::Level::off);
		/// Defines name and log level
		varResSwitch(String name, Logger::Level logLevel = Logger::Level::off)
			: varResSwitch(name, name, logLevel) { }

		// varResSwitch(String name, Real openRes, Real closedRes , 
		// 		Logger::Level logLevel = Logger::Level::off);

		SimPowerComp<Complex>::Ptr clone(String name);

		// #### General ####
		/// Initializes states from power flow data
		void initializeFromNodesAndTerminals(Real frequency);

		// #### MNA section ####
		/// Initializes MNA specific variables
		virtual void mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector);
		/// Stamps system matrix
		virtual void mnaApplySystemMatrixStamp(Matrix& systemMatrix);
		/// Stamps right side (source) vector
		virtual void mnaApplyRightSideVectorStamp(Matrix& rightVector);
		/// Update interface voltage from MNA system results
		virtual void mnaUpdateVoltage(const Matrix& leftVector);
		/// Update interface current from MNA system results
		virtual void mnaUpdateCurrent(const Matrix& leftVector);
		/// MNA post step operations
		virtual void mnaPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector);
		/// add MNA post step dependencies
		virtual void mnaAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector);

		// #### MNA section for switch ####
		/// Check if switch is closed
		Bool mnaIsClosed() { return attribute<Bool>("is_closed")->get(); }
		/// Stamps system matrix considering the defined switch position
		void mnaApplySwitchSystemMatrixStamp(Bool closed, Matrix& systemMatrix, Int freqIdx);

		Bool hasParameterChanged();

		class MnaPostStep : public Task {
		public:
			MnaPostStep(varResSwitch& varresswitch, Attribute<Matrix>::Ptr leftVector) :
				Task(varresswitch.mName + ".MnaPostStep"), mvarResSwitch(varresswitch), mLeftVector(leftVector) {
				mvarResSwitch.mnaAddPostStepDependencies(mPrevStepDependencies, mAttributeDependencies, mModifiedAttributes, mLeftVector);
			}
			void execute(Real time, Int timeStepCount) { mvarResSwitch.mnaPostStep(time, timeStepCount, mLeftVector); };
		private:
			varResSwitch& mvarResSwitch;
			Attribute<Matrix>::Ptr mLeftVector;
		};
	};
}
}
}
