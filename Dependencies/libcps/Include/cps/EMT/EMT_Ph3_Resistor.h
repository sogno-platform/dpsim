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
#include <cps/Base/Base_Ph3_Resistor.h>
namespace CPS {
namespace EMT {
namespace Ph3 {
 /// EMT Resistor
class Resistor :
	public Base::Ph3::Resistor,
	public MNAInterface,
	public PowerComponent<Real>,
	public SharedFactory<Resistor> {
protected:
public:
	/// Defines UID, name, component parameters and logging level
	Resistor(String uid, String name, Logger::Level logLevel = Logger::Level::off);
	/// Defines name, component parameters and logging level
	Resistor(String name, Logger::Level logLevel = Logger::Level::off)
		: Resistor(name, name, logLevel) { }

		// #### General ####
		///
		PowerComponent<Real>::Ptr clone(String name);
		/// Initializes component from power flow data
		void initializeFromPowerflow(Real frequency);
		/// enable DP to EMT bach transformation
		void enableBackShift();


		// #### MNA section ####
		/// Initializes internal variables of the component
		void mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftSideVector);
		/// Stamps system matrix
		void mnaApplySystemMatrixStamp(Matrix& systemMatrix);
		/// Stamps right side (source) vector
		void mnaApplyRightSideVectorStamp(Matrix& rightVector) { }
		/// Update interface voltage from MNA system result
		void mnaUpdateVoltage(const Matrix& leftVector);
		/// Update interface current from MNA system result
		void mnaUpdateCurrent(const Matrix& leftVector);

		class MnaPostStep : public Task {
		public:
			MnaPostStep(Resistor& resistor, Attribute<Matrix>::Ptr leftSideVector) :
				Task(resistor.mName + ".MnaPostStep"),
				mResistor(resistor), mLeftVector(leftSideVector) {

				mAttributeDependencies.push_back(mLeftVector);
				mModifiedAttributes.push_back(mResistor.attribute("v_intf"));
				mModifiedAttributes.push_back(mResistor.attribute("i_intf"));
			}

			void execute(Real time, Int timeStepCount);

		private:
			Resistor& mResistor;
			Attribute<Matrix>::Ptr mLeftVector;
		};
	};
}
}
}
