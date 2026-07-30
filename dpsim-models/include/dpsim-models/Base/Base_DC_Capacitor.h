// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <dpsim-models/AttributeList.h>

namespace CPS {
namespace Base {
namespace DC {

class Capacitor {
public:
  const Attribute<Real>::Ptr mCapacitance;

  explicit Capacitor(AttributeList::Ptr attributeList)
      : mCapacitance(attributeList->create<Real>("C")) {}
};

} // namespace DC
} // namespace Base
} // namespace CPS
