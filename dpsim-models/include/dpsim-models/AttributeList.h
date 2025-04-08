/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <iostream>
#include <memory>
#include <vector>

#include <dpsim-models/Attribute.h>
#include <dpsim-models/Config.h>

namespace CPS {
/// Base class of objects having attributes to access member variables.
class AttributeList : public SharedFactory<AttributeList> {
private:
  /// Map of all attributes
  AttributeBase::Map mAttributeMap;

public:
  using Ptr = std::shared_ptr<AttributeList>;

  AttributeList(){};

  const AttributeBase::Map &attributes() const { return mAttributeMap; };

  // Creates a new static Attribute and enters a pointer to it into this Attribute Map using the provided name.
  template <typename T>
  typename Attribute<T>::Ptr create(const String &name, T intitialValue = T()) {
    typename Attribute<T>::Ptr newAttr =
        AttributePointer<Attribute<T>>(AttributeStatic<T>::make(intitialValue));
    mAttributeMap[name] = newAttr;
    return newAttr;
  }

  // Creates a new dynamic Attribute and enters a pointer to it into this Attribute Map using the provided name.
  template <typename T>
  typename Attribute<T>::Ptr createDynamic(const String &name) {
    typename Attribute<T>::Ptr newAttr =
        AttributePointer<Attribute<T>>(AttributeDynamic<T>::make());
    mAttributeMap[name] = newAttr;
    return newAttr;
  }

  /// Return pointer to an attribute.
  AttributeBase::Ptr attribute(const String &name) const {
    auto it = mAttributeMap.find(name);
    if (it == mAttributeMap.end())
      throw InvalidAttributeException();

    return it->second;
  }

  /// Return pointer to an attribute.
  template <typename T>
  typename Attribute<T>::Ptr attributeTyped(const String &name) const {
    auto attr = attribute(name);
    auto attrPtr = std::dynamic_pointer_cast<Attribute<T>>(attr.getPtr());

    if (attrPtr == nullptr)
      throw InvalidAttributeException();

    return typename Attribute<T>::Ptr(attrPtr);
  }
};
} // namespace CPS
