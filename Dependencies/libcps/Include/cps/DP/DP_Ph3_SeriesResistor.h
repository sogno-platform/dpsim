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
#include <cps/Definitions.h>
#include <cps/Logger.h>
#include <cps/Base/Base_Ph1_Resistor.h>

namespace CPS {
namespace DP {
namespace Ph3 {
	///
	class SeriesResistor :
		public Base::Ph1::Resistor,
		public MNAInterface,
		public PowerComponent<Complex>,
		public SharedFactory<SeriesResistor> {

	public:
			/// Defines UID, name and logging level
		SeriesResistor(String uid, String name,	Logger::Level logLevel = Logger::Level::off);
			/// Defines name and logging level
		SeriesResistor(String name,	Logger::Level logLevel = Logger::Level::off)
			: SeriesResistor(name, name, logLevel) { }

		PowerComponent<Complex>::Ptr clone(String name);

		// #### General ####
		///
		void initialize(Matrix frequencies);
		/// Initializes component from power flow data
		void initializeFromPowerflow(Real frequency);

		// #### MNA section ####
		///
		void mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector);
		/// Stamps system matrix
		void mnaApplySystemMatrixStamp(Matrix& systemMatrix);
		///
		void mnaUpdateVoltage(const Matrix& leftVector);
		///
		void mnaUpdateCurrent(const Matrix& leftVector);

		class MnaPostStep : public Task {
		public:
			MnaPostStep(SeriesResistor& resistor, Attribute<Matrix>::Ptr leftSideVector) :
				Task(resistor.mName + ".MnaPostStep"), mResistor(resistor), mLeftVector(leftSideVector)
			{
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
