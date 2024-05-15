/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim-models/Base/Base_Governor.h>
#include <dpsim-models/Logger.h>
#include <dpsim-models/SimSignalComp.h>

namespace CPS {
namespace Signal {

class SteamGorvernorParameters
    : public Base::GovernorParameters,
      public SharedFactory<SteamGorvernorParameters> {

public:
  /// Droop, the value 1/K in the controller K(1+sT_2)
  Real R = 0;
  /// T_1 related to the differentiator in the controlle K(1+sT_2)/(1+sT_1)
  Real T1 = 0;
  /// T_2 related to the differentiator in the controlle K(1+sT_2)
  Real T2 = 0;
  /// Time constant T_3 of the actuator in the Governor
  Real T3 = 0;

  // ### Physical constraints ###
  /// Maximum growth rate
  Real dPmax = 0;
  /// Minimum decay rate
  Real dPmin = 0;
  /// Maximum mechanical power(pu)
  Real Pmax = 0;
  /// Minimum mechanical power (pu)
  Real Pmin = 0;

  /// Setpoint for omega (pu). It is adviced to choose Om_ref=1
  Real OmRef = 0;

  /// Proportional gain of anti-Windup
  Real Kbc = 0;
};

/// Steam turbine governor, where Omega_ref, P_ref are constant and T_1=0
/// Ref.: MATPAT Paper
class SteamTurbineGovernor : public SimSignalComp,
                             public Base::Governor,
                             public SharedFactory<SteamTurbineGovernor> {

private:
  /// Governor Parameters
  std::shared_ptr<SteamGorvernorParameters> mParameters;

  // ### Setpoints of the machine ###
  /// Setpoint for mechanical Power (pu)
  Real mPref;

  // ### State Variables ###
  /// Delta Omega = Omega_ref-Omega_meas at t=k, t=k-1 and t=k-2
  Real mDelOm;
  Real mDelOm_prev;
  Real mDelOm_2prev;
  /// Windup variable at t=k and t=k-1
  Real mP1;
  Real mP1_prev;
  /// p at time t=k-1
  Real mP;
  /// Derivative of Pgv at time k-1
  Real mDerPgv;
  /// The output of the Governor at k before the limiter
  Real mPgvLim;
  /// The output of the Governor at k after the limiter
  Real mPgv;

  /// Auxiliar variables
  Real mCa;
  Real mCb;

public:
  ///
  explicit SteamTurbineGovernor(const String &name)
      : SimSignalComp(name, name) {}

  /// Constructor with log level
  SteamTurbineGovernor(const String &name, CPS::Logger::Level logLevel);

  /// Sets Parameters of the turbine
  void
  setParameters(std::shared_ptr<Base::GovernorParameters> parameters) final;

  /// Initialises the initial state of the turbine
  void initializeFromPowerFlow(Real Pref) final;

  /// Performs a step to update all state variables and the output
  Real step(Real Omega, Real dt) final;
};

} // namespace Signal
} // namespace CPS