/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim-models/MNASimPowerComp.h>
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
				public MNASimPowerComp<Real>,
				public SharedFactory<Transformer>,
				public Base::Ph3::Transformer {
				
			public:
				/// Defines UID, name and logging level
				/// Defines UID, name and logging level
				Transformer(String uid, String name,
					Logger::Level logLevel = Logger::Level::off);
				/// Defines name and logging level
				Transformer(String name, Logger::Level logLevel = Logger::Level::off)
					: Transformer(name, name, logLevel) { }

            protected:
				/// DC equivalent current source [A]
				Matrix mEquivCurrent;
                /// Equivalent conductance [S]
				Matrix mEquivCond;
				/// Equivalent resistance scaling
				Matrix mResScaling ;
			public:
				/// Inductance [H]
				const Attribute<Matrix>::Ptr mInductance;
				///Resistance [ohm]
				const Attribute<Matrix>::Ptr mResistance;
				// #### General ####
				/// Defines component parameters
				void setParameters(Real nomVoltageEnd1, Real nomVoltageEnd2, Real ratioAbs, Real ratioPhase, Matrix resistance, Matrix inductance);
				/// Initializes component from power flow data
				void initializeFromNodesAndTerminals(Real frequency);
				/// Initializes auxiliar variables
				void initVars(Real timeStep);

				// #### MNA section ####
				/// Initializes internal variables of the component
				void mnaCompInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftSideVector) override;
				/// Stamps system matrix
				void mnaCompApplySystemMatrixStamp(SparseMatrixRow& systemMatrix) override;
				/// Stamps right side (source) vector
				void mnaCompApplyRightSideVectorStamp(Matrix& rightVector) override;
				/// Updates internal current variable of the component
				void mnaCompUpdateCurrent(const Matrix& leftVector) override;
				/// Updates internal voltage variable of the component
				void mnaCompUpdateVoltage(const Matrix& leftVector) override;
				/// MNA pre step operations
				void mnaCompPreStep(Real time, Int timeStepCount) override;
				/// MNA post step operations
				void mnaCompPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) override;
				/// Add MNA pre step dependencies
				void mnaCompAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) override;
				/// Add MNA post step dependencies
				void mnaCompAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector) override;
			};
		}
	}
}
