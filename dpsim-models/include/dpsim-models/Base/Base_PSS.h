// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <dpsim-models/Definitions.h>

namespace CPS {
namespace Base {

class PSSParameters {
public:
  PSSParameters(){};
  virtual ~PSSParameters() = default;
};

/// @brief Base model for power system stabilizers
class PSS {

public:
  virtual ~PSS() = default;

  virtual void
  setParameters(std::shared_ptr<Base::PSSParameters> parameters) = 0;

  /// Initializes PSS state variables from power-flow solution
  virtual void initialize(Real omega, Real activePower, Real Vd, Real Vq) = 0;

  /// @returns V_pss: stabilizing signal fed into the exciter
  virtual Real step(Real omega, Real activePower, Real Vd, Real Vq,
                    Real dt) = 0;
};
} // namespace Base
} // namespace CPS
