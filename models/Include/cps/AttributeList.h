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

#include <cps/Config.h>
#include <cps/Attribute.h>

namespace CPS {
	/// Base class of objects having attributes to access member variables.
	class AttributeList {
	private:
		/// Map of all attributes that should be exported to the Python interface
		AttributeBase::Map mAttributes;

	protected:
		template<typename T, typename... Args>
		void addAttribute(const String &name, Args&&... args) {
			mAttributes[name] = Attribute<T>::make(std::forward<Args>(args)...);
		}

	public:
		typedef std::shared_ptr<AttributeList> Ptr;

		AttributeList() { };

		const AttributeBase::Map & attributes() { return mAttributes; };

		/// Return pointer to an attribute.
		AttributeBase::Ptr attribute(const String &name) {
			auto it = mAttributes.find(name);
			if (it == mAttributes.end())
				throw InvalidAttributeException();

			return it->second;
		}

		/// Return pointer to an attribute.
		template<typename T>
		typename Attribute<T>::Ptr attribute(const String &name) {
			auto attr = attribute(name);
			auto attrPtr = std::dynamic_pointer_cast<Attribute<T>>(attr);

			if (attrPtr == NULL)
				throw InvalidAttributeException();

			return attrPtr;
		}

		template<typename T, typename... Args>
		void addAttributeRef(const String& name, const std::shared_ptr<AttributeBase> &ref, Args&&... args){
			addAttribute<T>(name, std::forward<Args>(args)...);
			setAttributeRef(name, ref);
		}

		void setAttributeRef(const String& name, const std::shared_ptr<AttributeBase> &ref) {
			auto attr = attribute(name);

			// clang complains here for some reason that the expressions in the typeid
			// might be evaluated (which is the whole point)
#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wpotentially-evaluated-expression"
#endif
			if (typeid(*attr) != typeid(*ref))
#ifdef __clang__
#pragma clang diagnostic pop
#endif
				throw InvalidAttributeException();

			mAttributes[name] = ref;
		}

		ComplexAttribute::Ptr attributeComplex(const String &name) {
			auto attr = attribute<Complex>(name);
			auto attrPtr = std::static_pointer_cast<ComplexAttribute>(attr);

			if (attrPtr == NULL)
				throw InvalidAttributeException();

			return attrPtr;
		}

		template<typename VarType>
		typename MatrixAttribute<VarType>::Ptr attributeMatrix(const String &name) {
			auto attr = attribute<MatrixVar<VarType>>(name);
			auto attrPtr = std::static_pointer_cast<MatrixAttribute<VarType>>(attr);

			if (attrPtr == NULL)
				throw InvalidAttributeException();

			return attrPtr;
		}

		MatrixRealAttribute::Ptr attributeMatrixReal(const String &name) {
			auto attr = attribute<Matrix>(name);
			auto attrPtr = std::static_pointer_cast<MatrixRealAttribute>(attr);

			if (attrPtr == NULL)
				throw InvalidAttributeException();

			return attrPtr;
		}

		MatrixCompAttribute::Ptr attributeMatrixComp(const String &name) {
			auto attr = attribute<MatrixComp>(name);
			auto attrPtr = std::static_pointer_cast<MatrixCompAttribute>(attr);

			if (attrPtr == NULL)
				throw InvalidAttributeException();

			return attrPtr;
		}

		void reset() {
			for (auto a : mAttributes) {
				a.second->reset();
			}
		}
	};
}
