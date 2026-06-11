// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <dpsim-models/Definitions.h>

namespace CPS {
namespace Base {

class TurbineParameters {
public:
  TurbineParameters() {}
  virtual ~TurbineParameters() = default;
};

/// @brief Base model for Turbines
class Turbine {
public:
  virtual void
  setParameters(std::shared_ptr<Base::TurbineParameters> parameters) = 0;

  /// Set steady-state initial values (call after setParameters, before first step)
  virtual void initializeStates(Real PmInit) = 0;

  /// Step the turbine with valve/gate opening Pgv and return mechanical power Pm
  virtual Real step(Real Pgv, Real dt) = 0;

  virtual ~Turbine() = default;
};

} // namespace Base
} // namespace CPS
