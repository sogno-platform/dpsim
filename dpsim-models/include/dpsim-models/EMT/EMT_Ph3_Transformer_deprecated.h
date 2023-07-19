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
#include <dpsim-models/EMT/EMT_Ph3_Inductor.h>
#include <dpsim-models/EMT/EMT_Ph3_Resistor.h>
#include <dpsim-models/EMT/EMT_Ph3_Capacitor.h>
#include <dpsim-models/Base/Base_Ph3_Transformer.h>

namespace CPS {
	namespace EMT {
		namespace Ph3 {
			/// Transformer that includes an inductance and resistance
			class Transformer :
				public CompositePowerComp<Real>,
				public SharedFactory<Transformer>,
				public Base::Ph3::Transformer {
			private:
				/// Internal resistor to model losses
				std::shared_ptr<EMT::Ph3::Resistor> mSubResistor;
				/// Internal inductor to model losses
				std::shared_ptr<EMT::Ph3::Inductor> mSubInductor;

				/// Internal parallel resistance 1 as snubber
				std::shared_ptr<EMT::Ph3::Resistor> mSubSnubResistor1;
				/// Internal parallel resistance 2 as snubber
				std::shared_ptr<EMT::Ph3::Resistor> mSubSnubResistor2;
				/// Internal parallel capacitance 1 as snubber
				std::shared_ptr<EMT::Ph3::Capacitor> mSubSnubCapacitor1;
				/// Internal parallel capacitance 2 as snubber
				std::shared_ptr<EMT::Ph3::Capacitor> mSubSnubCapacitor2;

				/// Snubber resistance 1
				Matrix mSnubberResistance1;
				/// Snubber resistance 2
				Matrix mSnubberResistance2;
				/// Snubber capacitance 1
				Matrix mSnubberCapacitance1;
				/// Snubber capacitance 2
				Matrix mSnubberCapacitance2;

				/// Boolean for considering resistive losses with sub resistor
				Bool mWithResistiveLosses;
			public:
				/// Defines UID, name and logging level
				Transformer(String uid, String name,
					Logger::Level logLevel = Logger::Level::off, Bool withResistiveLosses = false);
				/// Defines name and logging level
				Transformer(String name, Logger::Level logLevel = Logger::Level::off)
					: Transformer(name, name, logLevel) { }

				SimPowerComp<Real>::Ptr clone(String name);

				// #### General ####
				/// Defines component parameters
				void setParameters(Real nomVoltageEnd1, Real nomVoltageEnd2, Real ratedPower, Real ratioAbs, Real ratioPhase, Matrix resistance, Matrix inductance);
				/// Initializes component from power flow data
				void initializeFromNodesAndTerminals(Real frequency);

				// #### MNA section ####
				/// Initializes internal variables of the component
				void mnaParentInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) override;
				/// Stamps system matrix
				void mnaCompApplySystemMatrixStamp(SparseMatrixRow& systemMatrix) override;
				/// Updates internal current variable of the component
				void mnaCompUpdateCurrent(const Matrix& leftVector) override;
				/// Updates internal voltage variable of the component
				void mnaCompUpdateVoltage(const Matrix& leftVector) override;
				/// MNA pre step operations
				void mnaParentPreStep(Real time, Int timeStepCount) override;
				/// MNA post step operations
				void mnaParentPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) override;
				/// Add MNA pre step dependencies
				void mnaParentAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) override;
				/// Add MNA post step dependencies
				void mnaParentAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector) override;
			};
		}
	}
}
