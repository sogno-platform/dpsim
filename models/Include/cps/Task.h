/** Task
 *
 * @file
 * @author Georg Reinke <georg.reinke@rwth-aachen.de>
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

#include <memory>
#include <vector>

#include <cps/Attribute.h>

namespace CPS {
	/// \brief Tasks to be defined by every component.
	///
	/// Each component has to specify its tasks so that the scheduler
	/// can use this information. Tasks declare the attributes that are required
	/// or modified.
	/// The component declares Tasks derived from this general task class.
	/// Then, these tasks need to be pushed into the task list of each component
	/// so that the scheduler can collect them.
 	class Task {
	public:
		typedef std::shared_ptr<Task> Ptr;
		typedef std::vector<Ptr> List;

		Task() {}
		virtual ~Task() {}

		virtual void execute(Real time, Int timeStepCount) = 0;

		virtual String toString() const {
			return mName;
		}

		const std::vector<AttributeBase::Ptr>& getAttributeDependencies() {
			return mAttributeDependencies;
		}

		const std::vector<AttributeBase::Ptr>& getModifiedAttributes() {
			return mModifiedAttributes;
		}

		const std::vector<AttributeBase::Ptr>& getPrevStepDependencies() {
			return mPrevStepDependencies;
		}

	protected:
		Task(std::string name) : mName(name) {}
		std::string mName;
		std::vector<AttributeBase::Ptr> mAttributeDependencies;
		std::vector<AttributeBase::Ptr> mModifiedAttributes;
		std::vector<AttributeBase::Ptr> mPrevStepDependencies;
	};
}
