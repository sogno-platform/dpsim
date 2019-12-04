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
#include <cps/Base/Base_Ph1_Resistor.h>

namespace CPS {
namespace EMT {
namespace Ph3 {
	/// @brief EMT three-phase resistor
	///
	/// This resistor has resistance values different from zero
	/// only on the main diagonal. These values are identical.
	class SeriesResistor :
		public Base::Ph1::Resistor,
		public MNAInterface,
		public PowerComponent<Real>,
		public SharedFactory<SeriesResistor> {

	public:
		/// Defines UID, name and log level
		SeriesResistor(String uid, String name,	Logger::Level logLevel = Logger::Level::off);
		/// Defines name and log level
		SeriesResistor(String name,	Logger::Level logLevel = Logger::Level::off)
			: SeriesResistor(name, name, logLevel) { }

		// #### General ####
		/// Return new instance with the same parameters
		PowerComponent<Real>::Ptr clone(String name);
		/// Initializes states from power flow data
		void initializeFromPowerflow(Real frequency);

		// #### MNA section ####
		/// Initializes MNA specific variables
		void mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector);
		/// Stamps system matrix
		void mnaApplySystemMatrixStamp(Matrix& systemMatrix);
		/// Update interface voltage from MNA system results
		void mnaUpdateVoltage(const Matrix& leftVector);
		/// Update interface voltage from MNA system results
		void mnaUpdateCurrent(const Matrix& leftVector);

		class MnaPostStep : public Task {
		public:
			MnaPostStep(SeriesResistor& resistor, Attribute<Matrix>::Ptr leftSideVector) :
				Task(resistor.mName + ".MnaPostStep"),
				mResistor(resistor), mLeftVector(leftSideVector) {

				mAttributeDependencies.push_back(mLeftVector);
				mModifiedAttributes.push_back(mResistor.attribute("v_intf"));
				mModifiedAttributes.push_back(mResistor.attribute("i_intf"));
			}

			void execute(Real time, Int timeStepCount);

		private:
			SeriesResistor& mResistor;
			Attribute<Matrix>::Ptr mLeftVector;
		};
	};
}
}
}
