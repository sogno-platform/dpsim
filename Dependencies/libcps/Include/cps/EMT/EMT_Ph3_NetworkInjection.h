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
	namespace EMT {
		namespace Ph3 {
			/// \brief Ideal Voltage source model
			///
			/// This model uses modified nodal analysis to represent an ideal voltage source.
			/// For a voltage source between nodes j and k, a new variable
			/// (current across the voltage source) is added to the left side vector
			/// as unkown and it is taken into account for the equation of node j as
			/// positve and for the equation of node k as negative. Moreover
			/// a new equation ej - ek = V is added to the problem.
			class NetworkInjection :
				public PowerComponent<Real>,
				public MNAInterface,
				public SharedFactory<NetworkInjection> {
			private:
				///
				void updateVoltage(Real time);
				///
				Attribute<MatrixComp>::Ptr mVoltageRef;
				///
				Attribute<Real>::Ptr mSrcFreq;
			public:
				/// Defines UID, name, component parameters and logging level
				NetworkInjection(String uid, String name, Logger::Level loglevel = Logger::Level::off);
				/// Defines UID, name, component parameters and logging level
				NetworkInjection(String name, Logger::Level logLevel = Logger::Level::off)
					: NetworkInjection(name, name, logLevel) { }
				/// Defines name, component parameters and logging level
				NetworkInjection(String name,
					Complex voltage, Logger::Level logLevel = Logger::Level::off);
				///
				PowerComponent<Real>::Ptr clone(String name);

				// #### General ####
				/// Initializes component from power flow data
				void initializeFromPowerflow(Real frequency);
				///
				void setSourceValue(MatrixComp voltage);
				///
				//void initialize(Matrix frequencies);
				///
				void setParameters(MatrixComp voltageRef, Real srcFreq = -1);

				// #### MNA Section ####
				/// Initializes internal variables of the component
				void mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector);
				/// Stamps system matrix
				void mnaApplySystemMatrixStamp(Matrix& systemMatrix);
				/// Stamps right side (source) vector
				void mnaApplyRightSideVectorStamp(Matrix& rightVector);
				/// Returns current through the component
				void mnaUpdateCurrent(const Matrix& leftVector);

				class MnaPreStep : public Task {
				public:
					MnaPreStep(NetworkInjection& NetworkInjection) :
						Task(NetworkInjection.mName + ".MnaPreStep"), mNetworkInjection(NetworkInjection) {
						mAttributeDependencies.push_back(NetworkInjection.attribute("V_ref"));
						mModifiedAttributes.push_back(mNetworkInjection.attribute("right_vector"));
						mModifiedAttributes.push_back(mNetworkInjection.attribute("v_intf"));
					}
					void execute(Real time, Int timeStepCount);
				private:
					NetworkInjection& mNetworkInjection;
				};

				class MnaPostStep : public Task {
				public:
					MnaPostStep(NetworkInjection& NetworkInjection, Attribute<Matrix>::Ptr leftVector) :
						Task(NetworkInjection.mName + ".MnaPostStep"),
						mNetworkInjection(NetworkInjection), mLeftVector(leftVector) {
						mAttributeDependencies.push_back(mLeftVector);
						mModifiedAttributes.push_back(mNetworkInjection.attribute("i_intf"));
					}
					void execute(Real time, Int timeStepCount);
				private:
					NetworkInjection& mNetworkInjection;
					Attribute<Matrix>::Ptr mLeftVector;
				};
				//// #### DAE Section ####
				///// Residual function for DAE Solver
				//void daeResidual(double ttime, const double state[], const double dstate_dt[], double resid[], std::vector<int>& off);
				/////Voltage Getter
				//Complex daeInitialize();
			};
		}
	}
}
