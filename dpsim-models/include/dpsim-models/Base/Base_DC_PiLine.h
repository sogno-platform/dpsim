// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <dpsim-models/AttributeList.h>

namespace CPS {
namespace Base {
namespace DC {

class PiLine {
public:
  const Attribute<Real>::Ptr mSeriesResistance;
  const Attribute<Real>::Ptr mSeriesInductance;
  const Attribute<Real>::Ptr mParallelCapacitance;
  const Attribute<Real>::Ptr mParallelConductance;
  const Attribute<Real>::Ptr mInitialCurrent;

  explicit PiLine(AttributeList::Ptr attributeList)
      : mSeriesResistance(attributeList->create<Real>("R_series")),
        mSeriesInductance(attributeList->create<Real>("L_series")),
        mParallelCapacitance(attributeList->create<Real>("C_parallel", 0.0)),
        mParallelConductance(attributeList->create<Real>("G_parallel", 0.0)),
        mInitialCurrent(attributeList->create<Real>("i_init", 0.0)) {}
};

} // namespace DC
} // namespace Base
} // namespace CPS
