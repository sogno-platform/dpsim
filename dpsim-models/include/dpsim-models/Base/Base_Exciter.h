// SPDX-FileCopyrightText: 2025 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <dpsim-models/Definitions.h>

namespace CPS {
namespace Base {

class ExciterParameters {
public:
  ExciterParameters(){};
  virtual ~ExciterParameters() = default;
};

/// @brief Base model for exciters
class Exciter {

public:
  virtual ~Exciter() = default;

  /// Sets exciter parameters
  virtual void
  setParameters(std::shared_ptr<Base::ExciterParameters> parameters) = 0;

  /// Initializes exciter variables
  virtual void initialize(Real Vh_init, Real Ef_init) = 0;

  /// @param V_pss: Output of PSS
  virtual Real step(Real Vd, Real Vq, Real dt, Real Vpss = 0) = 0;
};
} // namespace Base
} // namespace CPS
