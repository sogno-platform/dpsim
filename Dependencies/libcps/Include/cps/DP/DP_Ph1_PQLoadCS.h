/** PQ Load
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
#include <cps/DP/DP_Ph1_CurrentSource.h>
#include <cps/Base/Base_Ph1_PQLoad.h>
#include <cps/PowerProfile.h>

namespace CPS {
namespace DP {
namespace Ph1 {
	/// TODO: read from CSV files
	/// \brief PQ-load represented by a current source
	class PQLoadCS :
		public PowerComponent<Complex>,
		public MNAInterface,
		public SharedFactory<PQLoadCS> {
	protected:
		/// Internal current source
		std::shared_ptr<DP::Ph1::CurrentSource> mSubCurrentSource;
		///
		Attribute<Complex>::Ptr mCurrentSourceRef;
		Attribute<Real>::Ptr mActivePower, mReactivePower, mNomVoltage;

		void updateSetPoint();
		void updateIntfValues();
	public:
		/// Defines UID, name and logging level
		PQLoadCS(String uid, String name,
			Logger::Level logLevel = Logger::Level::off);
		/// Defines UID, name and logging level
		PQLoadCS(String name,
			Logger::Level logLevel = Logger::Level::off);
		/// Defines name, component parameters and logging level
		PQLoadCS(String name,
			Real activePower, Real reactivePower, Real volt,
			Logger::Level logLevel = Logger::Level::off);
		/// Defines UID, name and logging level
		PQLoadCS(String uid, String name,
			Real activePower, Real reactivePower, Real nomVolt,
			Logger::Level logLevel = Logger::Level::off);

		void setParameters(Real activePower, Real reactivePower, Real nomVolt);
		PowerComponent<Complex>::Ptr clone(String name);

		// #### General ####
		/// Initializes component from power flow data
		void initializeFromPowerflow(Real frequency);

		// #### MNA section ####
		/// Initializes internal variables of the component
		void mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector);
		/// Stamps system matrix
		void mnaApplySystemMatrixStamp(Matrix& systemMatrix);
		/// Stamps right side (source) vector
		void mnaApplyRightSideVectorStamp(Matrix& rightVector);

		class MnaPreStep : public Task {
		public:
			MnaPreStep(PQLoadCS& load) :
				Task(load.mName + ".MnaPreStep"), mLoad(load) {
				mAttributeDependencies.push_back(load.attribute("P"));
				mAttributeDependencies.push_back(load.attribute("Q"));
				mAttributeDependencies.push_back(load.attribute("V_nom"));
				mModifiedAttributes.push_back(load.mSubCurrentSource->attribute("I_ref"));
			}

			void execute(Real time, Int timeStepCount);

		private:
			PQLoadCS& mLoad;
		};

		class MnaPostStep : public Task {
		public:
			MnaPostStep(PQLoadCS& load) :
				Task(load.mName + ".MnaPostStep"), mLoad(load) {
				mAttributeDependencies.push_back(load.mSubCurrentSource->attribute("i_intf"));
				mAttributeDependencies.push_back(load.mSubCurrentSource->attribute("v_intf"));
				mModifiedAttributes.push_back(load.attribute("i_intf"));
				mModifiedAttributes.push_back(load.attribute("v_intf"));
			}

			void execute(Real time, Int timeStepCount);

		private:
			PQLoadCS& mLoad;
		};
	};
}
}
}
