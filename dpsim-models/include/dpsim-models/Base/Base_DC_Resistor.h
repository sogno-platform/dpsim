// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <dpsim-models/AttributeList.h>

namespace CPS {
namespace Base {
namespace DC {

class Resistor {
public:
  const Attribute<Real>::Ptr mResistance;

  explicit Resistor(AttributeList::Ptr attributeList)
      : mResistance(attributeList->create<Real>("R")) {}
};

} // namespace DC
} // namespace Base
} // namespace CPS
