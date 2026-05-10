// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <vector>

#include <dpsim-models/EMT/EMT_Ph3_TwoTerminalVTypeVariableSSNComp.h>

namespace CPS {
namespace EMT {
namespace Ph3 {

/// \brief Three-phase, two-terminal, piecewise-linear flux-state inductor.
///
/// The component uses the state choice
///
///   x = phi_abc
///   u = v_abc
///   y = i_abc
///
/// with
///
///   d(phi)/dt = v
///
/// and a piecewise-linear constitutive relation
///
///   i_abc = f(phi_abc)
///
/// The piecewise-linear characteristic is assumed to be odd-symmetric and is
/// provided by non-negative breakpoint vectors for flux and current.
class PiecewiseLinearInductor final
    : public TwoTerminalVTypeVariableSSNComp,
      public SharedFactory<PiecewiseLinearInductor> {
private:
  std::vector<Real> mFluxBreakpoints;
  std::vector<Real> mCurrentBreakpoints;

  std::pair<Real, Real> slopeAndOffsetFromFlux(Real flux) const;

protected:
  Bool updateComponentParameters() override final;

public:
  using SharedFactory<PiecewiseLinearInductor>::make;

  PiecewiseLinearInductor(String uid, String name,
                          Logger::Level logLevel = Logger::Level::off);
  PiecewiseLinearInductor(String name,
                          Logger::Level logLevel = Logger::Level::off)
      : PiecewiseLinearInductor(name, name, logLevel) {}

  SimPowerComp<Real>::Ptr clone(String name) override final;

  void setParameters(const std::vector<Real> &fluxBreakpoints,
                     const std::vector<Real> &currentBreakpoints);
};

} // namespace Ph3
} // namespace EMT
} // namespace CPS
