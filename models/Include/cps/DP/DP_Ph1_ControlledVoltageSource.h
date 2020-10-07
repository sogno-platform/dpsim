/* Copyright 2017-2020 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/
#pragma once

#include <cps/SimPowerComp.h>
#include <cps/Solver/MNAInterface.h>

namespace CPS {
	namespace DP {
		namespace Ph1 {
			class ControlledVoltageSource :
				public MNAInterface,
				public SimPowerComp<Complex>,
				public SharedFactory<ControlledVoltageSource> {
			protected:
				void updateVoltage(Real time);

			public:
				/// Defines UID, name and logging level
				ControlledVoltageSource(String uid, String name, Logger::Level logLevel = Logger::Level::off);
				///
				ControlledVoltageSource(String name, Logger::Level logLevel = Logger::Level::off)
					: ControlledVoltageSource(name, name, logLevel) { }

				void setParameters(MatrixComp voltageRefABC);

				SimPowerComp<Complex>::Ptr clone(String name);
				// #### General ####
				/// Initializes component from power flow data
				void initializeFromPowerflow(Real frequency) { }

				// #### MNA section ####
				/// Initializes internal variables of the component
				void mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector);
				/// Stamps system matrix
				void mnaApplySystemMatrixStamp(Matrix& systemMatrix);
				/// Stamps right side (source) vector
				void mnaApplyRightSideVectorStamp(Matrix& rightVector);
				/// Returns current through the component
				void mnaUpdateCurrent(const Matrix& leftVector);
				/// MNA pre and post step operations
				void mnaPreStep(Real time, Int timeStepCount);
				void mnaPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector);
				/// add MNA pre and post step dependencies
				void mnaAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes);
				void mnaAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector);

				class MnaPreStep : public CPS::Task {
				public:
					MnaPreStep(ControlledVoltageSource& ControlledVoltageSource) :
						Task(ControlledVoltageSource.mName + ".MnaPreStep"), mControlledVoltageSource(ControlledVoltageSource) {
							mControlledVoltageSource.mnaAddPreStepDependencies(mPrevStepDependencies, mAttributeDependencies, mModifiedAttributes);
				}
				void execute(Real time, Int timeStepCount) { mControlledVoltageSource.mnaPreStep(time, timeStepCount); };
				private:
					ControlledVoltageSource& mControlledVoltageSource;
				};

				class MnaPostStep : public CPS::Task {
				public:
					MnaPostStep(ControlledVoltageSource& ControlledVoltageSource, Attribute<Matrix>::Ptr leftVector) :
						Task(ControlledVoltageSource.mName + ".MnaPostStep"), mControlledVoltageSource(ControlledVoltageSource), mLeftVector(leftVector) {
							mControlledVoltageSource.mnaAddPostStepDependencies(mPrevStepDependencies, mAttributeDependencies, mModifiedAttributes, mLeftVector);
				}
				void execute(Real time, Int timeStepCount) { mControlledVoltageSource.mnaPostStep(time, timeStepCount, mLeftVector); };
				private:
					ControlledVoltageSource& mControlledVoltageSource;
					Attribute<Matrix>::Ptr mLeftVector;
				};
				class MnaPreStepHarm : public CPS::Task {
				public:
					MnaPreStepHarm(ControlledVoltageSource& voltageSource) :
						Task(voltageSource.mName + ".MnaPreStepHarm"),
						mControlledVoltageSource(voltageSource) {
						mAttributeDependencies.push_back(voltageSource.attribute("V_ref"));
						mModifiedAttributes.push_back(mControlledVoltageSource.attribute("right_vector"));
						mModifiedAttributes.push_back(mControlledVoltageSource.attribute("v_intf"));
					}
					void execute(Real time, Int timeStepCount);
				private:
					ControlledVoltageSource& mControlledVoltageSource;
				};

				class MnaPostStepHarm : public CPS::Task {
				public:
					MnaPostStepHarm(ControlledVoltageSource& voltageSource, std::vector<Attribute<Matrix>::Ptr> leftVectors) :
						Task(voltageSource.mName + ".MnaPostStepHarm"),
						mControlledVoltageSource(voltageSource), mLeftVectors(leftVectors) {
						for (UInt i = 0; i < mLeftVectors.size(); i++)
							mAttributeDependencies.push_back(mLeftVectors[i]);
						mModifiedAttributes.push_back(mControlledVoltageSource.attribute("i_intf"));
					}
					void execute(Real time, Int timeStepCount);
				private:
					ControlledVoltageSource& mControlledVoltageSource;
					std::vector< Attribute<Matrix>::Ptr > mLeftVectors;
				};
			};
		}
	}
}
