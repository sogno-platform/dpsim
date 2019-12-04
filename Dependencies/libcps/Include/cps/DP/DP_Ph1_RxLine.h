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
#include <cps/Base/Base_Ph1_PiLine.h>
#include <cps/DP/DP_Ph1_Inductor.h>
#include <cps/DP/DP_Ph1_Resistor.h>

namespace CPS {
namespace DP {
namespace Ph1 {

	class RxLine :
		public PowerComponent<Complex>,
		public MNAInterface,
		public Base::Ph1::PiLine,
		public SharedFactory<RxLine> {
	protected:
		/// Voltage across the component [V]
		Complex mVoltage;
		/// Current through the component [A]
		Complex mCurrent;
		/// Inductance submodel
		std::shared_ptr<Inductor> mSubInductor;
		/// Resistor submodel
		std::shared_ptr<Resistor> mSubResistor;
		/// Inductor end to ground resistor to facilitate initialization
		std::shared_ptr<Resistor> mInitialResistor;

	public:
		/// Defines UID, name, logging level
		RxLine(String uid, String name, Logger::Level logLevel = Logger::Level::off);
		/// Defines name, component parameters and logging level
		RxLine(String name, Logger::Level logLevel = Logger::Level::off)
			: RxLine(name, name, logLevel) { }

		PowerComponent<Complex>::Ptr clone(String name);

		// #### General ####
		/// Initializes component from power flow data
		void initializeFromPowerflow(Real frequency);

		// #### MNA section ####
		/// Initializes internal variables of the component
		void mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector);
		/// Stamps system matrix
		void mnaApplySystemMatrixStamp(Matrix& systemMatrix);
		/// Stamps system matrix
		void mnaApplyInitialSystemMatrixStamp(Matrix& systemMatrix);
		/// Stamps right side (source) vector
		void mnaApplyRightSideVectorStamp(Matrix& rightVector);
		void mnaUpdateVoltage(const Matrix& leftVector);
		void mnaUpdateCurrent(const Matrix& leftVector);

		class MnaPreStep : public Task {
		public:
			MnaPreStep(RxLine& line) :
				Task(line.mName + ".MnaPreStep"), mLine(line) {
				mAttributeDependencies.push_back(line.mSubResistor->attribute("right_vector"));
				mAttributeDependencies.push_back(line.mSubInductor->attribute("right_vector"));
				mModifiedAttributes.push_back(line.attribute("right_vector"));
			}

			void execute(Real time, Int timeStepCount);

		private:
			RxLine& mLine;
		};

		class MnaPostStep : public Task {
		public:
			MnaPostStep(RxLine& line, Attribute<Matrix>::Ptr leftVector) :
				Task(line.mName + ".MnaPostStep"), mLine(line), mLeftVector(leftVector) {
				mAttributeDependencies.push_back(leftVector);
				mAttributeDependencies.push_back(line.mSubInductor->attribute("i_intf"));
				mModifiedAttributes.push_back(line.attribute("i_intf"));
				mModifiedAttributes.push_back(line.attribute("v_intf"));
			}

			void execute(Real time, Int timeStepCount);

		private:
			RxLine& mLine;
			Attribute<Matrix>::Ptr mLeftVector;
		};
	};
}
}
}
