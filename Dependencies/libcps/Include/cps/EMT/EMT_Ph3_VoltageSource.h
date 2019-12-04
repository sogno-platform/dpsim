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


#include <cps/PowerComponent.h>
#include <cps/Solver/MNAInterface.h>

namespace CPS {
	namespace EMT {
		namespace Ph3 {
			/// \brief Ideal Voltage source model
			///
			/// This model uses modified nodal analysis to represent an ideal voltage source.
			/// For a voltage source between nodes j and k, a new variable (current across the voltage source)
			/// is added to the left side vector
			/// as unkown and it is taken into account for the equation of node j as positve and for the equation
			/// of node k as negative. Moreover
			/// a new equation ej - ek = V is added to the problem.
			class VoltageSource :
				public MNAInterface,
				public PowerComponent<Real>,
				public SharedFactory<VoltageSource> {
			protected:
				void updateVoltage(Real time);
				void updateVoltage(Matrix vabc);

				Attribute<Complex>::Ptr mVoltageRef;
				Attribute<Real>::Ptr mSrcFreq;
			public:
				/// Defines UID, name and logging level
				VoltageSource(String uid, String name, Logger::Level logLevel = Logger::Level::off);
				///
				VoltageSource(String name, Logger::Level logLevel = Logger::Level::off)
					: VoltageSource(name, name, logLevel) { }

				void setParameters(Complex voltageRef, Real srcFreq = -1);

				PowerComponent<Real>::Ptr clone(String name);
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
					MnaPreStep(VoltageSource& voltageSource) :
						Task(voltageSource.mName + ".MnaPreStep"), mVoltageSource(voltageSource) {
						mAttributeDependencies.push_back(voltageSource.attribute("V_ref"));
						mModifiedAttributes.push_back(mVoltageSource.attribute("right_vector"));
						mModifiedAttributes.push_back(mVoltageSource.attribute("v_intf"));
					}

					void execute(Real time, Int timeStepCount);

				private:
					VoltageSource& mVoltageSource;
				};

				class MnaPostStep : public CPS::Task {
				public:
					MnaPostStep(VoltageSource& voltageSource, Attribute<Matrix>::Ptr leftVector) :
						Task(voltageSource.mName + ".MnaPostStep"), mVoltageSource(voltageSource), mLeftVector(leftVector)
					{
						mAttributeDependencies.push_back(mLeftVector);
						mModifiedAttributes.push_back(mVoltageSource.attribute("i_intf"));
					}

					void execute(Real time, Int timeStepCount);

				private:
					VoltageSource& mVoltageSource;
					Attribute<Matrix>::Ptr mLeftVector;
				};
			};
		}
	}
}
