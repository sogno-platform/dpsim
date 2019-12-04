/**
 * @file
 * @author Markus Mirz <mmirz@eonerc.rwth-aachen.de>
 *		   Junjie Zhang <junjie.zhang@eonerc.rwth-aachen.de>
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
#include <cps/EMT/EMT_Ph3_RxLine.h>
#include <cps/EMT/EMT_Ph3_Inductor.h>
#include <cps/Base/Base_Ph3_Transformer.h>

namespace CPS {
	namespace EMT {
		namespace Ph3 {
			/// Transformer that includes an inductance and resistance
			class Transformer :
				public PowerComponent<Real>,
				public MNAInterface,
				public SharedFactory<Transformer>,
				public Base::Ph3::Transformer {
			private:
				/// Internal inductor to model losses
				std::shared_ptr<EMT::Ph3::Inductor> mSubInductor;
				/// Internal parallel resistance as snubber
				std::shared_ptr<EMT::Ph3::Resistor> mSubSnubResistor;
				std::shared_ptr<EMT::Ph3::Resistor> mSubResistor;
			public:
				/// Defines UID, name and logging level
				Transformer(String uid, String name,
					Logger::Level logLevel = Logger::Level::off);
				/// Defines name and logging level
				Transformer(String name, Logger::Level logLevel = Logger::Level::off)
					: Transformer(name, name, logLevel) { }

				PowerComponent<Real>::Ptr clone(String name);

				// #### General ####
				/// Defines component parameters
				void setParameters(Real ratioAbs, Real ratioPhase, Matrix resistance, Matrix inductance);
				///
				void initialize(Matrix frequencies);
				/// Initializes component from power flow data
				void initializeFromPowerflow(Real frequency);

				// #### MNA section ####
				/// Initializes internal variables of the component
				void mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector);
				/// Stamps system matrix
				void mnaApplySystemMatrixStamp(Matrix& systemMatrix);
				/// Stamps right side (source) vector
				void mnaApplyRightSideVectorStamp(Matrix& rightVector);
				/// Updates internal current variable of the component
				void mnaUpdateCurrent(const Matrix& leftVector);
				/// Updates internal voltage variable of the component
				void mnaUpdateVoltage(const Matrix& leftVector);

				class MnaPreStep : public Task {
				public:
					MnaPreStep(Transformer& transformer) :
						Task(transformer.mName + ".MnaPreStep"), mTransformer(transformer) {
						mAttributeDependencies.push_back(transformer.mSubSnubResistor->attribute("right_vector"));
						mAttributeDependencies.push_back(transformer.mSubInductor->attribute("right_vector"));
						if (transformer.mSubResistor)
							mAttributeDependencies.push_back(transformer.mSubResistor->attribute("right_vector"));
						mModifiedAttributes.push_back(transformer.attribute("right_vector"));
					}

					void execute(Real time, Int timeStepCount);

				private:
					Transformer& mTransformer;
				};


				class MnaPostStep : public Task {
				public:
					MnaPostStep(Transformer& transformer, Attribute<Matrix>::Ptr leftVector) :
						Task(transformer.mName + ".MnaPostStep"), mTransformer(transformer), mLeftVector(leftVector) {
						mAttributeDependencies.push_back(transformer.mSubInductor->attribute("i_intf"));
						mAttributeDependencies.push_back(leftVector);
						mModifiedAttributes.push_back(transformer.attribute("i_intf"));
						mModifiedAttributes.push_back(transformer.attribute("v_intf"));
					}

					void execute(Real time, Int timeStepCount);

				private:
					Transformer& mTransformer;
					Attribute<Matrix>::Ptr mLeftVector;
				};
			};
		}
	}
}
