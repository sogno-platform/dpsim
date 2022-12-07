/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <iostream>
#include <vector>
#include <memory>

#include <dpsim-models/Config.h>
#include <dpsim-models/Attribute.h>

namespace CPS {
	/// Base class of objects having attributes to access member variables.
	class AttributeList {
	protected:

		/// Map of all attributes
		AttributeBase::Map mAttributes;

	public:
		typedef std::shared_ptr<AttributeList> Ptr;

		AttributeList() { };

		virtual ~AttributeList() = default;

		const AttributeBase::Map & attributes() { return mAttributes; };

		/// Return pointer to an attribute.
		virtual AttributeBase::Ptr attribute(const String &name) {
			auto it = mAttributes.find(name);
			if (it == mAttributes.end())
				throw InvalidAttributeException();

			return it->second;
		}

		/// Return pointer to an attribute.
		template<typename T>
		typename Attribute<T>::Ptr attributeTyped(const String &name) {
			auto attr = attribute(name);
			auto attrPtr = std::dynamic_pointer_cast<Attribute<T>>(attr.getPtr());

			if (attrPtr == nullptr)
				throw InvalidAttributeException();

			return typename Attribute<T>::Ptr(attrPtr);
		}

	};
}
