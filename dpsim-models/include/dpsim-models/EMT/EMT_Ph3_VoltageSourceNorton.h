/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/
#pragma once

#include <dpsim-models/Base/Base_Ph1_VoltageSource.h>
#include <dpsim-models/MNASimPowerComp.h>
#include <dpsim-models/Solver/MNAInterface.h>

namespace CPS {
	namespace EMT {
		namespace Ph3 {
			/// \brief Voltage source with Norton equivalent model
			class VoltageSourceNorton :
								public Base::Ph1::VoltageSource,
				public MNASimPowerComp<Real>,
				public SharedFactory<VoltageSourceNorton> {
			protected:
				void updateState(Real time);

				/// Equivalent current source [A]
				Matrix mEquivCurrent = Matrix::Zero(3, 1);

				//  ### Real Voltage source parameters ###
				/// conductance [S]
				Real mConductance;
			public:
				/// Resistance [ohm]
				const Attribute<Real>::Ptr mResistance;
				/// Defines UID, name and logging level
				VoltageSourceNorton(String uid, String name, Logger::Level logLevel = Logger::Level::off);
				///
				VoltageSourceNorton(String name, Logger::Level logLevel = Logger::Level::off)
					: VoltageSourceNorton(name, name, logLevel) { }

				// #### General ####
				void setParameters(Complex voltageRef, Real srcFreq, Real resistance);
				///
				void setVoltageRef(Complex voltage) const;

				SimPowerComp<Real>::Ptr clone(String name);
				/// Initializes component from power flow data
				void initializeFromNodesAndTerminals(Real frequency) { }

				// #### MNA section ####
				/// Initializes internal variables of the component
				void mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector);
				/// Stamps system matrix
				void mnaApplySystemMatrixStamp(Matrix& systemMatrix);
				/// Stamps right side (source) vector
				void mnaApplyRightSideVectorStamp(Matrix& rightVector);
				/// Update interface voltage from MNA system result
				void mnaUpdateVoltage(const Matrix& leftVector);
				/// Returns current through the component
				void mnaUpdateCurrent(const Matrix& leftVector);

				void mnaPreStep(Real time, Int timeStepCount) override;
				void mnaPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) override;

				/// Add MNA pre step dependencies
				void mnaAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) override;

				/// Add MNA post step dependencies
				void mnaAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector) override;
			};
		}
	}
}
