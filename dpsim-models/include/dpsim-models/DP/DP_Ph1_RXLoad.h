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
#include <dpsim-models/DP/DP_Ph1_Capacitor.h>
#include <dpsim-models/DP/DP_Ph1_Inductor.h>
#include <dpsim-models/DP/DP_Ph1_Resistor.h>

namespace CPS {
namespace DP {
namespace Ph1 {
	/// Constant impedance load model consisting of RLC elements
	class RXLoad :
		public CompositePowerComp<Complex>,
		public SharedFactory<RXLoad> {
	protected:
		/// Resistance [Ohm]
		/// CHECK: This is only used for logging output
		Real mResistance;
		/// Reactance [Ohm]
		Real mReactance;
		/// Inductance [H]
		Real mInductance;
		/// Capacitance [F]
		Real mCapacitance;
		/// Internal inductor
		std::shared_ptr<DP::Ph1::Inductor> mSubInductor;
		/// Internal capacitor
		std::shared_ptr<DP::Ph1::Capacitor> mSubCapacitor;
		/// Internal resistance
		std::shared_ptr<DP::Ph1::Resistor> mSubResistor;
		/// Right side vectors of subcomponents
		std::vector<const Matrix*> mRightVectorStamps;

		/// ### Flags
		///
		Bool mInitPowerFromTerminal = true;
		///
		Bool mInitVoltageFromNode = true;

	public:
		/// Active power [Watt]
		const Attribute<Real>::Ptr mActivePower;
		/// Reactive power [VAr]
		const Attribute<Real>::Ptr mReactivePower;
		/// Nominal voltage [V]
		const Attribute<Real>::Ptr mNomVoltage;
		/// Defines UID, name and logging level
		RXLoad(String uid, String name,
			Logger::Level logLevel = Logger::Level::off);
		/// Defines name, component parameters and logging level
		RXLoad(String name,
			Logger::Level logLevel = Logger::Level::off);

		// #### General ####
		/// Initialize component from power flow data
		void initializeFromNodesAndTerminals(Real frequency);
		/// Set model specific parameters
		void setParameters(Real activePower, Real ReactivePower);
		/// Set model specific parameters
		void setParameters(Real activePower, Real ReactivePower, Real nominalVoltage);

		// #### MNA section ####
		/// Update interface current from MNA system result
		void mnaCompUpdateCurrent(const Matrix& leftVector) override;
		/// Update interface voltage from MNA system result
		void mnaCompUpdateVoltage(const Matrix& leftVector) override;
		/// MNA pre step operations
		void mnaParentPreStep(Real time, Int timeStepCount) override;
		/// MNA post step operations
		void mnaParentPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) override;
		/// Add MNA pre step dependencies
		void mnaParentAddPreStepDependencies(AttributeBase::List &prevStepDependencies,
			AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) override;
		/// Add MNA post step dependencies
		void mnaParentAddPostStepDependencies(AttributeBase::List &prevStepDependencies,
			AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes,
			Attribute<Matrix>::Ptr &leftVector) override;
	};
}
}
}
