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
		const Attribute<String>::Ptr mName;
		/// Unique identifier
		const Attribute<String>::Ptr mUID;
	public:
		typedef std::shared_ptr<IdentifiedObject> Ptr;
		typedef std::vector<Ptr> List;

		IdentifiedObject() { }

		IdentifiedObject(String uid, String name)
			: 	mName(Attribute<String>::create("name", mAttributes, name)),
				mUID(Attribute<String>::create("uid", mAttributes, uid))
			{ }

		IdentifiedObject(String name) :
			IdentifiedObject(name, name) { }

		virtual ~IdentifiedObject() { }

		/// FIXME: Workaround for pybind. The methods that return attributes with and without the full (template) type should not have the same name!
		AttributeBase::Ptr attributeBase(const String &name) {
			return attribute(name);
		}

		/// Returns component name
		String name() { return **mName; }
		/// Returns unique id
		String uid() { return **mUID; }
		/// Get component type (cross-platform)
		String type() { return Utils::className(this); }
	};
}
