// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <dpsim-models/Base/Base_Governor.h>
#include <dpsim-models/Logger.h>
#include <dpsim-models/SimSignalComp.h>

namespace CPS {
namespace Signal {

class HydroGovernorParameters : public Base::GovernorParameters,
                                public SharedFactory<HydroGovernorParameters> {
public:
  /// Droop coefficient
  Real R = 0;
  /// Time constant of the first PT1 block (lag, denominator)
  Real T1 = 0;
  /// Time constant of the lead numerator
  Real T2 = 0;
  /// Time constant of the second PT1 block (lag, denominator)
  Real T3 = 0;
  /// Maximum mechanical power (pu)
  Real Pmax = 0;
  /// Minimum mechanical power (pu)
  Real Pmin = 0;
  /// Speed setpoint (pu); typically 1
  Real OmRef = 0;
};

/// Hydro turbine governor — transfer function K(1+sT2)/((1+sT1)(1+sT3))
/// decomposed into two parallel PT1 blocks.
/// Ref.: A. Roehder, B. Fuchs, J. Massman, M. Quester, A. Schnettler,
///       "Transmission system stability assessment within an integrated grid
///       development process"
class HydroTurbineGovernor : public SimSignalComp,
                             public Base::Governor,
                             public SharedFactory<HydroTurbineGovernor> {
private:
  std::shared_ptr<HydroGovernorParameters> mParameters;

  // ### Setpoints ###
  Real mPref = 0;

  // ### State variables (previous step) ###
  Real mDelOm_prev = 0;
  Real mX1_prev = 0;
  Real mX2_prev = 0;

  // ### State variables (current step) — loggable via .attr() ###
  /// Delta omega = OmRef - Omega at time k
  const Attribute<Real>::Ptr mDelOm;
  /// State variable of the T1 PT1 block at time k
  const Attribute<Real>::Ptr mX1;
  /// State variable of the T3 PT1 block at time k
  const Attribute<Real>::Ptr mX2;
  /// Gate opening (governor output) at time k
  const Attribute<Real>::Ptr mPgv;

  // ### Auxiliary variables computed in initializeStates ###
  Real mCa = 0;
  Real mCb = 0;

public:
  HydroTurbineGovernor(const String &name,
                       CPS::Logger::Level logLevel = CPS::Logger::Level::off);

  void
  setParameters(std::shared_ptr<Base::GovernorParameters> parameters) final;
  void initializeStates(Real Pref) final;
  Real step(Real Omega, Real dt) final;
};

} // namespace Signal
} // namespace CPS
