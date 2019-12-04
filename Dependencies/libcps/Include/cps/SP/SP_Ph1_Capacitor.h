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
#include <cps/Base/Base_Ph1_Capacitor.h>

namespace CPS {
	namespace SP {
		namespace Ph1 {
			class Capacitor :
				public Base::Ph1::Capacitor,
				public MNAInterface,
				public PowerComponent<Complex>,
				public SharedFactory<Capacitor> {
			protected:
				/// Equivalent conductance [S]
				Complex mSusceptance;

			public:
				/// Defines UID, name and logging level
				Capacitor(String uid, String name, Logger::Level logLevel = Logger::Level::off);
				/// Defines name, component parameters and logging level
				Capacitor(String name, Logger::Level logLevel = Logger::Level::off)
					: Capacitor(name, name, logLevel) { }

				PowerComponent<Complex>::Ptr clone(String name);

				// #### General ####
				/// Initializes component from power flow data
				void initializeFromPowerflow(Real frequency);
				// #### MNA section ####
				/// Initializes internal variables of the component
				void mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector);
				/// Stamps system matrix
				void mnaApplySystemMatrixStamp(Matrix& systemMatrix);
				/// Update interface voltage from MNA system result
				void mnaUpdateVoltage(const Matrix& leftVector);
				/// Update interface current from MNA system result
				void mnaUpdateCurrent(const Matrix& leftVector);


				class MnaPostStep : public CPS::Task {
				public:
					MnaPostStep(Capacitor& capacitor, Attribute<Matrix>::Ptr leftVector)
						: Task(capacitor.mName + ".MnaPostStep"), mCapacitor(capacitor), mLeftVector(leftVector) {
						mAttributeDependencies.push_back(mLeftVector);
						mModifiedAttributes.push_back(mCapacitor.attribute("v_intf"));
						mModifiedAttributes.push_back(mCapacitor.attribute("i_intf"));
					}

					void execute(Real time, Int timeStepCount);

				private:
					Capacitor& mCapacitor;
					Attribute<Matrix>::Ptr mLeftVector;
				};
			};
		}
	}
}
