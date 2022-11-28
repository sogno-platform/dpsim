/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/
#pragma once

#include <dpsim-models/Base/Base_Ph1_VoltageSource.h>
#include <dpsim-models/SimPowerComp.h>
#include <dpsim-models/Solver/MNAInterface.h>

namespace CPS {
	namespace EMT {
		namespace Ph3 {
			/// \brief Voltage source with Norton equivalent model
			class VoltageSourceNorton :
				public MNAInterface,
				public Base::Ph1::VoltageSource,
				public SimPowerComp<Real>,
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

				class MnaPreStep : public CPS::Task {
				public:
					MnaPreStep(VoltageSourceNorton& VoltageSourceNorton) :
						Task(**VoltageSourceNorton.mName + ".MnaPreStep"), mVoltageSourceNorton(VoltageSourceNorton) {
						mAttributeDependencies.push_back(VoltageSourceNorton.attribute("V_ref"));
						mModifiedAttributes.push_back(mVoltageSourceNorton.attribute("right_vector"));
						mModifiedAttributes.push_back(mVoltageSourceNorton.attribute("v_intf"));
					}

					void execute(Real time, Int timeStepCount);

				private:
					VoltageSourceNorton& mVoltageSourceNorton;
				};

				class MnaPostStep : public CPS::Task {
				public:
					MnaPostStep(VoltageSourceNorton& VoltageSourceNorton, Attribute<Matrix>::Ptr leftVector) :
						Task(**VoltageSourceNorton.mName + ".MnaPostStep"), mVoltageSourceNorton(VoltageSourceNorton), mLeftVector(leftVector)
					{
						mAttributeDependencies.push_back(mLeftVector);
						mModifiedAttributes.push_back(mVoltageSourceNorton.attribute("i_intf"));
					}

					void execute(Real time, Int timeStepCount);

				private:
					VoltageSourceNorton& mVoltageSourceNorton;
					Attribute<Matrix>::Ptr mLeftVector;
				};
			};
		}
	}
}
