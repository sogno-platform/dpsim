/* Copyright 2017-2020 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim-models/Solver/MNAInterface.h>
#include <dpsim-models/MNASimPowerComp.h>
#include <dpsim-models/Solver/MNASwitchInterface.h>
#include <dpsim-models/Solver/MNAVariableCompInterface.h>
#include <dpsim-models/Definitions.h>
#include <dpsim-models/Logger.h>
#include <dpsim-models/Base/Base_Ph3_Switch.h>

namespace CPS {
namespace EMT {
namespace Ph3 {
	/// \brief
	///
	/// Switch with variable resistance to avoid numerical oscillations, when an inductive current is suddenly interrupted.
	/// It is useful as a fault switch especially on a faulted generator or transformer winding.

	/// The switch resistance changes at a defined fixed rate by multiplying previous resistance value with a factor for the rate of change
	/// The MNA variable component interface is used to recompute the system Matrix after each change.
	class varResSwitch :
		public MNASimPowerComp<Real>,
		public Base::Ph3::Switch,
		public MNAVariableCompInterface,
		public MNASwitchInterface,
		public SharedFactory<varResSwitch> {

	protected:

		Bool mPrevState=false;
		Real mDeltaResClosed = 0; 
		Real mDeltaResOpen = 1.5;
		Matrix mPrevRes; // previous resistance value to multiply with rate of change
		// because we change the base value mClosedResistance & mOpenResistance to recompute the system Matrix
		// we need to save the initialisation values to use them as target values in the transition
		Matrix mInitClosedRes;
		Matrix mInitOpenRes;



	public:
		/// Defines UID, name, component parameters and logging level
		varResSwitch(String uid, String name, Logger::Level logLevel = Logger::Level::off);
		/// Defines name, component parameters and logging level
		varResSwitch(String name, Logger::Level logLevel = Logger::Level::off)
			: varResSwitch(name, name, logLevel) { }

		SimPowerComp<Real>::Ptr clone(String name);

		// #### General ####
		void setInitParameters(Real timestep);
		/// Initializes component from power flow data
		void initializeFromNodesAndTerminals(Real frequency);

		// #### General MNA section ####
		void mnaCompInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector);
		/// Stamps system matrix
		void mnaCompApplySystemMatrixStamp(SparseMatrixRow& systemMatrix);
		/// Stamps right side (source) vector
		void mnaCompApplyRightSideVectorStamp(Matrix& rightVector);
		/// Update interface voltage from MNA system result
		void mnaCompUpdateVoltage(const Matrix& leftVector);
		/// Update interface current from MNA system result
		void mnaCompUpdateCurrent(const Matrix& leftVector);

		// #### MNA section for switches ####
		/// Check if switch is closed
		Bool mnaIsClosed() { return isClosed(); }
		/// Stamps system matrix considering the defined switch position
		void mnaCompApplySwitchSystemMatrixStamp(Bool closed, SparseMatrixRow& systemMatrix, Int freqIdx);

		void mnaCompPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) override;

		/// Add MNA post step dependencies
		void mnaCompAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector) override;
		
		Bool hasParameterChanged();
	};
}
}
}
