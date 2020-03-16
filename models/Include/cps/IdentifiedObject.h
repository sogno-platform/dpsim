/** Identified Object
 *
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

#include <cps/Config.h>
#include <cps/Definitions.h>
#include <cps/AttributeList.h>
#include <cps/Utils.h>

namespace CPS {
	/// Common base class of all objects which are
	/// identified by a name and an unique identifier (UID).
	class IdentifiedObject: virtual public AttributeList {
	protected:
		/// Human readable name
		String mName;
		/// Unique identifier
		String mUID;
	public:
		typedef std::shared_ptr<IdentifiedObject> Ptr;
		typedef std::vector<Ptr> List;

		IdentifiedObject() { }

		IdentifiedObject(String uid, String name) {
			mUID = uid;
			mName = name;

			addAttribute<String>("uid", &mUID, Flags::read);
			addAttribute<String>("name", &mName, Flags::read);
		}

		IdentifiedObject(String name) :
			IdentifiedObject(name, name) { }

		virtual ~IdentifiedObject() { }

		/// Returns component name
		String name() { return mName; }
		/// Returns unique id
		String uid() { return mUID; }
		/// Get component type (cross-platform)
		String type() { return Utils::className(this); }
	};
}
