/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
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

		IdentifiedObject(String uid, String name)
		: mName(name), mUID(uid) {

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
