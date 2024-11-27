/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim-models/AttributeList.h>
#include <dpsim-models/Config.h>
#include <dpsim-models/Definitions.h>
#include <dpsim-models/Utils.h>

namespace CPS {
/// Common base class of all objects which are
/// identified by a name and an unique identifier (UID).
class IdentifiedObject {
protected:
  /// Attribute List
  AttributeList::Ptr mAttributes = AttributeList::make();

public:
  /// Human readable name
  const Attribute<String>::Ptr mName;
  /// Unique identifier
  const Attribute<String>::Ptr mUID;

  typedef std::shared_ptr<IdentifiedObject> Ptr;
  typedef std::vector<Ptr> List;

  IdentifiedObject() {}

  IdentifiedObject(const String &uid, const String &name)
      : mName(mAttributes->create("name", name)),
        mUID(mAttributes->create("uid", uid)) {}

  explicit IdentifiedObject(const String &name)
      : IdentifiedObject(name, name) {}

  virtual ~IdentifiedObject() = default;

  /// Return pointer to an attribute.
  AttributeBase::Ptr attribute(const String &name) const {
    return mAttributes->attribute(name);
  }

  /// Return pointer to an attribute.
  template <typename T>
  typename Attribute<T>::Ptr attributeTyped(const String &name) const {
    return mAttributes->attributeTyped<T>(name);
  }

  const AttributeBase::Map &attributes() const {
    return mAttributes->attributes();
  };

  String name() { return **mName; }
  /// Returns unique id
  String uid() { return **mUID; }
  /// Get component type (cross-platform)
  String type() { return Utils::className(this); }
  // Returns a description of the object
  virtual String description() { return ""; }
};
} // namespace CPS
