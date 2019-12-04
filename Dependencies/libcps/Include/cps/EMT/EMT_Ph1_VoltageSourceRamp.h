/** Real voltage source freq (EMT)
 *
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
#include <cps/EMT/EMT_Ph1_VoltageSource.h>

namespace CPS {
namespace EMT {
namespace Ph1 {
	class VoltageSourceRamp :
		public PowerComponent<Real>,
		public MNAInterface,
		public SharedFactory<VoltageSourceRamp> {
	protected:
		///
		Complex mVoltageRef;
		///
		Complex mAddVoltage;
		///
		Real mSrcFreq;
		///
		Real mSwitchTime;
		///
		Real mAddSrcFreq;
		///
		Real mRampTime;
		///
		std::shared_ptr<VoltageSource> mSubVoltageSource;
	public:
		/// Defines UID, name and logging level
		VoltageSourceRamp(String uid, String name, Logger::Level logLevel = Logger::Level::off);
		/// Defines name and logging level
		VoltageSourceRamp(String name, Logger::Level logLevel = Logger::Level::off)
			: VoltageSourceRamp(name, name, logLevel) { }

		PowerComponent<Real>::Ptr clone(String name);

		// #### General ####
		/// Initializes component from power flow data
		void initializeFromPowerflow(Real frequency) { }
		///
		void setParameters(Complex voltage, Complex addVoltage,
			Real srcFreq, Real addSrcFreq, Real switchTime, Real rampTime);
		///
		void initialize(Matrix frequencies);

		// #### MNA section ####
		/// Initializes internal variables of the component
		void mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector);
		/// Stamps system matrix
		void mnaApplySystemMatrixStamp(Matrix& systemMatrix);
		/// Stamps right side (source) vector
		void mnaApplyRightSideVectorStamp(Matrix& rightVector);

		void updateState(Real time);

		class MnaPreStep : public Task {
		public:
			MnaPreStep(VoltageSourceRamp& voltageSource) :
				Task(voltageSource.mName + ".MnaPreStep"), mVoltageSource(voltageSource) {
				// rampTime etc. aren't attributes (yet), so doesn't really depend on anything
				mModifiedAttributes.push_back(voltageSource.mSubVoltageSource->attribute("V_ref"));
			}

			void execute(Real time, Int timeStepCount);

		private:
			VoltageSourceRamp& mVoltageSource;
		};
	};
}
}
}
