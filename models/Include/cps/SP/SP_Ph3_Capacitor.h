/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <cps/SimPowerComp.h>
#include <cps/Solver/MNAInterface.h>
#include <cps/Base/Base_Ph3_Capacitor.h>

namespace CPS {
	namespace SP {
		namespace Ph3 {
			/// \brief Capacitor model
			///
			/// The capacitor is represented by a DC equivalent circuit which
			/// corresponds to one iteration of the trapezoidal integration method.
			/// The equivalent DC circuit is a resistance in parallel with a current source.
			/// The resistance is constant for a defined time step and system
			/// frequency and the current source changes for each iteration.
			class Capacitor :
				public Base::Ph3::Capacitor,
				public MNAInterface,
				public SimPowerComp<Complex>,
				public SharedFactory<Capacitor> {
			protected:
				/// Equivalent conductance [S]
				MatrixComp mSusceptance = Matrix::Zero(3, 1);

			public:
				/// Defines UID, name and logging level
				Capacitor(String uid, String name, Logger::Level logLevel = Logger::Level::off);
				/// Defines name, component parameters and logging level
				Capacitor(String name, Logger::Level logLevel = Logger::Level::off)
					: Capacitor(name, name, logLevel) { }

				SimPowerComp<Complex>::Ptr clone(String name);

				// #### General ####
				/// Initializes component from power flow data
				void initializeFromNodesAndTerminals(Real frequency);
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
