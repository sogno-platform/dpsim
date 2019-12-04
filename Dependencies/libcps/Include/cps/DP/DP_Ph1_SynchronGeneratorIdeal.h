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
#include <cps/DP/DP_Ph1_VoltageSource.h>

namespace CPS {
namespace DP {
namespace Ph1 {
	/// Ideal voltage source representing a synchronous generator
	class SynchronGeneratorIdeal :
		public PowerComponent<Complex>,
		public MNAInterface,
		public SharedFactory<SynchronGeneratorIdeal> {
	private:
		/// Voltage set point [V]
		Complex mVoltageRef;
		/// Inner voltage source that represents the generator
		std::shared_ptr<DP::Ph1::VoltageSource> mSubVoltageSource;
	public:
		/// Defines UID, name, component parameters and logging level
		SynchronGeneratorIdeal(String uid, String name,
			Logger::Level logLevel = Logger::Level::off);
		/// Defines name, component parameters and logging level
		SynchronGeneratorIdeal(String name,
			Logger::Level logLevel = Logger::Level::off);

		PowerComponent<Complex>::Ptr clone(String name);

		// #### General ####
		///
		void initialize(Matrix frequencies);
		/// Initializes component from power flow data
		void initializeFromPowerflow(Real frequency);

		// #### MNA section ####
		void mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector);
		/// Stamps system matrix
		void mnaApplySystemMatrixStamp(Matrix& systemMatrix);
		/// Stamps right side (source) vector
		void mnaApplyRightSideVectorStamp(Matrix& rightVector);
	};
}
}
}
