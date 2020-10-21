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
	namespace EMT {
		namespace Ph3 {
            class ControlledVoltageSource :
				public MNAInterface,
				public SimPowerComp<Real>,
				public SharedFactory<ControlledVoltageSource> {
			protected:
				void updateVoltage(Real time);

			public:
				/// Defines UID, name and logging level
				ControlledVoltageSource(String uid, String name, Logger::Level logLevel = Logger::Level::off);
				///
				ControlledVoltageSource(String name, Logger::Level logLevel = Logger::Level::off)
					: ControlledVoltageSource(name, name, logLevel) { }

				void setParameters(const Matrix& voltageRefABC);

				SimPowerComp<Real>::Ptr clone(String name);
				// #### General ####
				/// Initializes component from power flow data
				void initializeFromNodesAndTerminals(Real frequency) { }

				// #### MNA section ####
				/// Initializes internal variables of the component
				void mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector);
				/// Stamps system matrix
				void mnaApplySystemMatrixStamp(Matrix& systemMatrix);
				/// Stamps right side (source) vector
				void mnaApplyRightSideVectorStamp(Matrix& rightVector);
				/// Returns current through the component
				void mnaUpdateCurrent(const Matrix& leftVector);
				/// MNA pre step operations
				void mnaPreStep(Real time, Int timeStepCount);
				/// MNA post step operations
				void mnaPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector);
				/// Add MNA pre step dependencies
				void mnaAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes);
				/// Add MNA post step dependencies
				void mnaAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector);

				class MnaPreStep : public Task {
				public:
					MnaPreStep(ControlledVoltageSource& controlledVoltageSource) :
						Task(controlledVoltageSource.mName + ".MnaPreStep"), mControlledVoltageSource(controlledVoltageSource) {
							mControlledVoltageSource.mnaAddPreStepDependencies(mPrevStepDependencies, mAttributeDependencies, mModifiedAttributes);
						}
						void execute(Real time, Int timeStepCount) { mControlledVoltageSource.mnaPreStep(time, timeStepCount); };
				private:
					ControlledVoltageSource& mControlledVoltageSource;
				};

				class MnaPostStep : public Task {
				public:
					MnaPostStep(ControlledVoltageSource& controlledVoltageSource, Attribute<Matrix>::Ptr leftVector) :
						Task(controlledVoltageSource.mName + ".MnaPostStep"),			
						mControlledVoltageSource(controlledVoltageSource), mLeftVector(leftVector) {
							mControlledVoltageSource.mnaAddPostStepDependencies(mPrevStepDependencies, mAttributeDependencies, mModifiedAttributes, mLeftVector);
					}
					void execute(Real time, Int timeStepCount)  { mControlledVoltageSource.mnaPostStep(time, timeStepCount, mLeftVector); };
				private:
					ControlledVoltageSource& mControlledVoltageSource;
					Attribute<Matrix>::Ptr mLeftVector;
				};
			};
		}
	}
}
