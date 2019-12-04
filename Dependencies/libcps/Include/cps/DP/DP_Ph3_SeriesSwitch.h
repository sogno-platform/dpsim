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
#include <cps/Solver/MNASwitchInterface.h>
#include <cps/Definitions.h>
#include <cps/Logger.h>
#include <cps/Base/Base_Ph1_Switch.h>

namespace CPS {
namespace DP {
namespace Ph3 {
	/// \brief Dynamic phasor three-phase switch
	///
	/// The switch can be opened and closed.
	/// Each state has a specific resistance value.
	/// For this model, the resistance model is the same for all phases and
	/// only in series.
	class SeriesSwitch :
		public Base::Ph1::Switch,
		public PowerComponent<Complex>,
		public SharedFactory<SeriesSwitch>,
		public MNASwitchInterface {
	protected:
	public:
		/// Defines UID, name and logging level
		SeriesSwitch(String uid, String name, Logger::Level loglevel = Logger::Level::off);
		/// Defines name and logging level
		SeriesSwitch(String name, Logger::Level logLevel = Logger::Level::off)
			: SeriesSwitch(name, name, logLevel) { }

		// #### General ####
		/// Initializes component from power flow data
		void initializeFromPowerflow(Real frequency);

		// #### General MNA section ####
		///
		void mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector);
		/// Stamps system matrix
		void mnaApplySystemMatrixStamp(Matrix& systemMatrix);
		/// Update interface voltage from MNA system result
		void mnaUpdateVoltage(const Matrix& leftVector);
		/// Update interface current from MNA system result
		void mnaUpdateCurrent(const Matrix& leftVector);

		// #### MNA section for switches ####
		/// Check if switch is closed
		Bool mnaIsClosed() { return mIsClosed; }
		/// Stamps system matrix considering the defined switch position
		void mnaApplySwitchSystemMatrixStamp(Matrix& systemMatrix, Bool closed);

		class MnaPostStep : public Task {
		public:
			MnaPostStep(SeriesSwitch& sSwitch, Attribute<Matrix>::Ptr leftSideVector) :
				Task(sSwitch.mName + ".MnaPostStep"), mSwitch(sSwitch), mLeftVector(leftSideVector)
			{
				mAttributeDependencies.push_back(mLeftVector);
				mModifiedAttributes.push_back(mSwitch.attribute("v_intf"));
				mModifiedAttributes.push_back(mSwitch.attribute("i_intf"));
			}

			void execute(Real time, Int timeStepCount);

		private:
			SeriesSwitch& mSwitch;
			Attribute<Matrix>::Ptr mLeftVector;
		};
	};
}
}
}
