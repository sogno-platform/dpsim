/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim-models/CompositePowerComp.h>
#include <dpsim-models/Solver/MNAInterface.h>
#include <dpsim-models/EMT/EMT_Ph3_Capacitor.h>
#include <dpsim-models/EMT/EMT_Ph3_Resistor.h>

namespace CPS {
namespace EMT {
namespace Ph3 {

	class Shunt : 
		public CompositePowerComp<Real>, 
		public SharedFactory<Shunt>{
	private:
		/// Conductance [S]
		Matrix mConductance;
		/// Susceptance [S]
		Matrix mSusceptance;

		/// Capacitor between terminal and ground
		std::shared_ptr<Capacitor> mSubCapacitor;
		/// Resistor between terminal and ground
		std::shared_ptr<Resistor> mSubResistor;

	public:
		/// Defines UID, name, component parameters and logging level
		Shunt(String uid, String name, Logger::Level logLevel = Logger::Level::off);

		/// Defines name and logging level
		Shunt(String name, Logger::Level logLevel = Logger::Level::off)
			: Shunt(name, name, logLevel) { }

		// #### General ####
		/// Set shunt specific parameters
		void setParameters(Real conductance, Real susceptance);
		void setParameters(Matrix conductance, Matrix susceptance);

		// #### MNA section ####
		/// Initializes component from power flow data
		void initializeFromNodesAndTerminals(Real frequency) final;
		/// Add MNA pre step dependencies
		void mnaParentAddPreStepDependencies(AttributeBase::List &prevStepDependencies,
			AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) final;
		/// MNA pre step operations
		void mnaParentPreStep(Real time, Int timeStepCount) final;
		/// Updates internal current variable of the component
		void mnaCompUpdateCurrent(const Matrix& leftVector) final;
		/// Updates internal voltage variable of the component
		void mnaCompUpdateVoltage(const Matrix& leftVector) final;
		/// MNA post-step operations
		void mnaParentPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) final;
		/// add MNA post-step dependencies
		void mnaParentAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector) final;

	};
}
}
}
