/** Current source
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
#include <cps/Base/Base_Ph1_CurrentSource.h>
#include <cps/Task.h>

namespace CPS {
namespace DP {
namespace Ph1 {
	/// \brief Dynamic phasor ideal current source
	///
	/// A positive current is flowing out of node1 and into node2.
	/// In case of a dynamic phasor simulation, a frequency different
	/// from zero is added on top of the system frequency.
	class CurrentSource :
		public MNAInterface,
		public PowerComponent<Complex>,
		public SharedFactory<CurrentSource> {
	protected:
		Attribute<Complex>::Ptr mCurrentRef;
	public:
		/// Defines UID, name and logging level
		CurrentSource(String uid, String name, Logger::Level loglevel = Logger::Level::off);
		/// Defines name and logging level
		CurrentSource(String name, Logger::Level logLevel = Logger::Level::off)
			: CurrentSource(name, name, logLevel) { }
		/// Defines name, component parameters and logging level
		CurrentSource(String name, Complex current,
			Logger::Level logLevel = Logger::Level::off);

		void setParameters(Complex current);

		PowerComponent<Complex>::Ptr clone(String copySuffix);

		// #### General ####
		/// Initializes component from power flow data
		void initializeFromPowerflow(Real frequency);

		// #### MNA section ####
		///
		void mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector);
		/// Stamps system matrix
		void mnaApplySystemMatrixStamp(Matrix& systemMatrix) { }
		/// Stamps right side (source) vector
		void mnaApplyRightSideVectorStamp(Matrix& rightVector);
		///
		void mnaUpdateVoltage(const Matrix& leftVector);

		class MnaPreStep : public Task {
		public:
			MnaPreStep(CurrentSource& currentSource) :
				Task(currentSource.mName + ".MnaPreStep"), mCurrentSource(currentSource)
			{
				mAttributeDependencies.push_back(currentSource.attribute("I_ref"));
				mModifiedAttributes.push_back(currentSource.attribute("right_vector"));
				mModifiedAttributes.push_back(currentSource.attribute("i_intf"));
			}

			void execute(Real time, Int timeStepCount);

		private:
			CurrentSource& mCurrentSource;
		};

		class MnaPostStep : public Task {
		public:
			MnaPostStep(CurrentSource& currentSource, Attribute<Matrix>::Ptr leftSideVector) :
				Task(currentSource.mName + ".MnaPostStep"), mCurrentSource(currentSource), mLeftVector(leftSideVector)
			{
				mAttributeDependencies.push_back(mLeftVector);
				mModifiedAttributes.push_back(mCurrentSource.attribute("v_intf"));
			}

			void execute(Real time, Int timeStepCount);
		private:
			CurrentSource& mCurrentSource;
			Attribute<Matrix>::Ptr mLeftVector;
		};
	};
}
}
}
