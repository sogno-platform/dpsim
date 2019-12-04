/**
 * @file
 * @author Junjie Zhang <junjie.zhang@eonerc.rwth-aachen.de>
 * @copyright 2017-2019, Institute for Automation of Complex Power Systems, EONERC
 *
 * CPowerSystems
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *********************************************************************************/
#pragma once

#include <cps/PowerComponent.h>
#include <cps/Solver/MNAInterface.h>

namespace CPS {
	namespace DP {
		namespace Ph3 {
			class ControlledVoltageSource :
				public MNAInterface,
				public PowerComponent<Complex>,
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

				PowerComponent<Complex>::Ptr clone(String name);
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

				class MnaPreStep : public CPS::Task {
				public:
					MnaPreStep(ControlledVoltageSource& ControlledVoltageSource) :
						Task(ControlledVoltageSource.mName + ".MnaPreStep"), mControlledVoltageSource(ControlledVoltageSource) {
						mAttributeDependencies.push_back(ControlledVoltageSource.attribute("v_intf"));
						mModifiedAttributes.push_back(mControlledVoltageSource.attribute("right_vector"));
					}

					void execute(Real time, Int timeStepCount);

				private:
					ControlledVoltageSource& mControlledVoltageSource;
				};

				class MnaPostStep : public CPS::Task {
				public:
					MnaPostStep(ControlledVoltageSource& ControlledVoltageSource, Attribute<Matrix>::Ptr leftVector) :
						Task(ControlledVoltageSource.mName + ".MnaPostStep"), mControlledVoltageSource(ControlledVoltageSource), mLeftVector(leftVector)
					{
						mAttributeDependencies.push_back(mLeftVector);
						mModifiedAttributes.push_back(mControlledVoltageSource.attribute("i_intf"));
					}

					void execute(Real time, Int timeStepCount);

				private:
					ControlledVoltageSource& mControlledVoltageSource;
					Attribute<Matrix>::Ptr mLeftVector;
				};
			};
		}
	}
}
