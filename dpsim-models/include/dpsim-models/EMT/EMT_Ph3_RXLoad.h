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
#include <dpsim-models/EMT/EMT_Ph3_Inductor.h>
#include <dpsim-models/EMT/EMT_Ph3_Resistor.h>

namespace CPS {
	namespace EMT {
		namespace Ph3 {
			/// \brief
			/// TODO: currently modelled as an impedance, which obviously doesn't have a constant power characteristic
			/// Model as current source and read from CSV files
			class RXLoad :
				public CompositePowerComp<Real>,
				public SharedFactory<RXLoad> {
			protected:
				/// Power [Watt]
				MatrixComp mPower;
				/// Resistance [Ohm]
				Matrix mResistance;
				/// Reactance [Ohm]
				Matrix mReactance;
				/// Inductance [H]
				Matrix mInductance;
				/// Capacitance [F]
				Matrix mCapacitance;
				///
				Bool initPowerFromTerminal = true;
				/// Internal inductor
				std::shared_ptr<EMT::Ph3::Inductor> mSubInductor;
				/// Internal capacitor
				std::shared_ptr<EMT::Ph3::Capacitor> mSubCapacitor;
				/// Internal resistance
				std::shared_ptr<EMT::Ph3::Resistor> mSubResistor;
			public:
				/// Active power [Watt]
				const Attribute<Matrix>::Ptr mActivePower;
				/// Reactive power [VAr]
				const Attribute<Matrix>::Ptr mReactivePower;
				/// Nominal voltage [V]
				const Attribute<Real>::Ptr mNomVoltage;
				/// Defines UID, name and logging level
				RXLoad(String uid, String name,
					Logger::Level logLevel = Logger::Level::off);
				/// Defines name, component parameters and logging level
				RXLoad(String name,
					Logger::Level logLevel = Logger::Level::off);
				/// Defines name, component parameters and logging level
				RXLoad(String name,
					Matrix activePower, Matrix reactivePower, Real volt,
					Logger::Level logLevel = Logger::Level::off);

				// #### General ####
				///
				void setParameters(Matrix activePower, Matrix reactivePower, Real volt=0);
				/// Initializes component from power flow data
				void initializeFromNodesAndTerminals(Real frequency);

				// #### MNA section ####
				void mnaCompUpdateCurrent(const Matrix& leftVector) override;
				void mnaCompUpdateVoltage(const Matrix& leftVector) override;

				/// MNA pre and post step operations
				void mnaParentPreStep(Real time, Int timeStepCount) override;
				void mnaParentPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) override;

				void mnaParentAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) override;
				void mnaParentAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector) override;
			};
		}
	}
}
