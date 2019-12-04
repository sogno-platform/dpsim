/**
 * @file
 * @author Markus Mirz <mmirz@eonerc.rwth-aachen.de>
 * @copyright 2017-2018, Institute for Automation of Complex Power Systems, EONERC
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

#include <cps/Base/Base_Ph1_VoltageSource.h>
#include <cps/PowerComponent.h>
#include <cps/Solver/MNAInterface.h>

namespace CPS {
	namespace EMT {
		namespace Ph3 {
			/// \brief Voltage source with Norton equivalent model
			class VoltageSourceNorton :
				public MNAInterface,
				public PowerComponent<Real>,
				public Base::Ph1::VoltageSource,
				public SharedFactory<VoltageSourceNorton> {
			protected:
				void updateState(Real time);

				/// Equivalent current source [A]
				Matrix mEquivCurrent = Matrix::Zero(3, 1);

				//  ### Real Voltage source parameters ###
				/// Resistance [ohm]
				Real mResistance;
				/// conductance [S]
				Real mConductance;
			public:
				/// Defines UID, name and logging level
				VoltageSourceNorton(String uid, String name, Logger::Level logLevel = Logger::Level::off);
				///
				VoltageSourceNorton(String name, Logger::Level logLevel = Logger::Level::off)
					: VoltageSourceNorton(name, name, logLevel) { }

				// #### General ####
				void setParameters(Complex voltageRef, Real srcFreq, Real resistance);
				///
				void setVoltageRef(Complex voltage) { mVoltageRef = voltage; }

				PowerComponent<Real>::Ptr clone(String name);
				/// Initializes component from power flow data
				void initializeFromPowerflow(Real frequency) { }

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
						Task(VoltageSourceNorton.mName + ".MnaPreStep"), mVoltageSourceNorton(VoltageSourceNorton) {
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
						Task(VoltageSourceNorton.mName + ".MnaPostStep"), mVoltageSourceNorton(VoltageSourceNorton), mLeftVector(leftVector)
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
