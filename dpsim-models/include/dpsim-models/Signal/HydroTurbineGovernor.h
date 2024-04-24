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

class HydroGorvernorParameters
    : public Base::GovernorParameters,
      public SharedFactory<HydroGorvernorParameters> {

public:
  /// droop coefficient
  Real R;
  ///
  Real T1;
  ///
  Real T2;
  ///
  Real T3;
  /// Maximum mechanical power(pu)
  Real Pmax;
  /// Minimum mechanical power (pu)
  Real Pmin;
  // Setpoint for omega (pu). It is adviced to choose Om_ref=1
  Real OmRef;
};

// ### Hydro turbine governor parameters, where Omega_ref,  P_ref are constant and T_1=0 ###
/// Ref.: MATPAT Paper
class HydroTurbineGovernor : public SimSignalComp,
                             public Base::Governor,
                             public SharedFactory<HydroTurbineGovernor> {

private:
  /// Governor Parameters
  std::shared_ptr<HydroGorvernorParameters> mParameters;

  //Setpoint for mechanical Power (pu)
  Real mPref;

  // Delta Omega = Omega_ref-Omega_meas at k and at k-1
  Real mDelOm;
  Real mDelOm_prev;

  // State Variable of T1 PT1 at k and at k-1
  Real mX1;
  Real mX1_prev;

  // State Variable of T3 PT1 at k and at k-1
  Real mX2;
  Real mX2_prev;

  // The outpur of the Governor at k
  Real mPgv;

  /// Auxiliar variables
  Real mCa;
  Real mCb;

public:
  ///
  explicit HydroTurbineGovernor(const String &name)
      : SimSignalComp(name, name) {}

  /// Constructor with log level
  HydroTurbineGovernor(const String &name, CPS::Logger::Level logLevel);

  /// Sets Parameters of the turbine
  void
  setParameters(std::shared_ptr<Base::GovernorParameters> parameters) final;

  /// Initialises the initial state of the turbine
  void initialize(Real Pref) final;

  /// Performs a step to update all state variables and the output
  Real step(Real Omega, Real dt) final;
};

} // namespace Signal
} // namespace CPS