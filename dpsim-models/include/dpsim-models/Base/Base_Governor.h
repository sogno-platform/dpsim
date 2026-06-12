// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <dpsim-models/Definitions.h>

namespace CPS {
namespace Base {

class GovernorParameters {
public:
  GovernorParameters() {}
  virtual ~GovernorParameters() = default;
};

/// @brief Base model for Governors
class Governor {
public:
  virtual void
  setParameters(std::shared_ptr<Base::GovernorParameters> parameters) = 0;

  /// Set steady-state initial values (call after setParameters, before first step)
  virtual void initializeStates(Real PmRef) = 0;

  /// Step the governor and return the valve/gate opening signal Pgv
  virtual Real step(Real Omega, Real dt) = 0;

  virtual ~Governor() = default;
};

} // namespace Base
} // namespace CPS
