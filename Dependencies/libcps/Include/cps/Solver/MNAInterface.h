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

#include <cps/AttributeList.h>
#include <cps/Config.h>
#include <cps/Definitions.h>
#include <cps/Task.h>

namespace CPS {
	/// Interface to be implemented by all models used by the MNA solver.
	class MNAInterface : virtual public AttributeList {
	public:
		typedef std::shared_ptr<MNAInterface> Ptr;
		typedef std::vector<Ptr> List;

		// #### MNA Base Functions ####
		/// Initializes variables of components
		virtual void mnaInitialize(Real omega, Real timeStep) {
			mMnaTasks.clear();
		}
		virtual void mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
			mnaInitialize(omega, timeStep);
		}
		/// Stamps system matrix
		virtual void mnaApplySystemMatrixStamp(Matrix& systemMatrix) { }
		/// Stamps right side (source) vector
		virtual void mnaApplyRightSideVectorStamp(Matrix& rightVector) { }
		/// Update interface voltage from MNA system result
		virtual void mnaUpdateVoltage(const Matrix& leftVector) { }
		/// Update interface current from MNA system result
		virtual void mnaUpdateCurrent(const Matrix& leftVector) { }

		// #### MNA Harmonic Base Functions ####
		/// Initializes variables of components
		virtual void mnaInitializeHarm(Real omega, Real timeStep, std::vector<Attribute<Matrix>::Ptr> leftVector) {
			mnaInitialize(omega, timeStep);
		}
		/// Stamps system matrix considering the frequency index
		virtual void mnaApplySystemMatrixStampHarm(Matrix& systemMatrix, Int freqIdx) { }
		/// Stamps right side (source) vector considering the frequency index
		virtual void mnaApplyRightSideVectorStampHarm(Matrix& sourceVector) { }
		virtual void mnaApplyRightSideVectorStampHarm(Matrix& sourceVector, Int freqIdx) { }
		/// Return list of MNA tasks
		const Task::List& mnaTasks() {
			return mMnaTasks;
		}
	protected:
		/// Every MNA component modifies its source vector attribute.
		MNAInterface() {
			addAttribute<Matrix>("right_vector", &mRightVector, Flags::read);
		}

		/// List of tasks that relate to using MNA for this component (usually pre-step and/or post-step)
		Task::List mMnaTasks;
		/// This component's contribution ("stamp") to the right-side vector.
		/// TODO performance improvements from a sparse representation, at least during copying / summation?
		Matrix mRightVector;
	};
}
