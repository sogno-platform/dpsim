// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <dpsim-models/AttributeList.h>

namespace CPS {
namespace Base {
namespace DC {

class Inductor {
public:
  const Attribute<Real>::Ptr mInductance;
  const Attribute<Real>::Ptr mInitialCurrent;

  explicit Inductor(AttributeList::Ptr attributeList)
      : mInductance(attributeList->create<Real>("L")),
        mInitialCurrent(attributeList->create<Real>("i_init", 0.0)) {}
};

} // namespace DC
} // namespace Base
} // namespace CPS
