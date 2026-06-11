// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

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
  /// Droop — the value 1/K in the controller K(1+sT2)/(1+sT1)
  Real R = 0;
  /// T1 related to the lag in the controller K(1+sT2)/(1+sT1)
  Real T1 = 0;
  /// T2 related to the lead in the controller K(1+sT2)
  Real T2 = 0;
  /// Time constant T3 of the actuator
  Real T3 = 0;

  /// Maximum rate of change of Pgv (pu/s)
  Real dPmax = 0;
  /// Minimum rate of change of Pgv (pu/s)
  Real dPmin = 0;
  /// Maximum mechanical power (pu)
  Real Pmax = 0;
  /// Minimum mechanical power (pu)
  Real Pmin = 0;

  /// Speed setpoint (pu); typically 1
  Real OmRef = 0;

  /// Anti-windup proportional gain
  Real Kbc = 0;
};

/// Steam turbine governor
/// Ref.: A. Roehder, B. Fuchs, J. Massman, M. Quester, A. Schnettler,
///       "Transmission system stability assessment within an integrated grid
///       development process"
class SteamTurbineGovernor : public SimSignalComp,
                             public Base::Governor,
                             public SharedFactory<SteamTurbineGovernor> {
private:
  std::shared_ptr<SteamGorvernorParameters> mParameters;

  // ### Setpoints ###
  Real mPref = 0;

  // ### State variables (previous step) ###
  Real mDelOm_prev = 0;
  Real mDelOm_2prev = 0;
  Real mP1_prev = 0;

  // ### State variables (current step) — loggable via .attr() ###
  /// Delta omega = OmRef - Omega at time k
  const Attribute<Real>::Ptr mDelOm;
  /// Intermediate lead-lag state at time k
  const Attribute<Real>::Ptr mP1;
  /// Lead-lag output at time k
  const Attribute<Real>::Ptr mP;
  /// Gate/valve opening before limiter
  const Attribute<Real>::Ptr mPgvLim;
  /// Gate/valve opening (governor output) at time k
  const Attribute<Real>::Ptr mPgv;

  // ### Auxiliar variables computed in initializeStates ###
  Real mCa = 0;
  Real mCb = 0;

  // ### Internal transient ###
  Real mDerPgv = 0;

public:
  explicit SteamTurbineGovernor(const String &name)
      : SimSignalComp(name, name), mDelOm(mAttributes->create<Real>("DelOm")),
        mP1(mAttributes->create<Real>("P1")),
        mP(mAttributes->create<Real>("P")),
        mPgvLim(mAttributes->create<Real>("PgvLim")),
        mPgv(mAttributes->create<Real>("Pgv")) {}

  SteamTurbineGovernor(const String &name, CPS::Logger::Level logLevel);

  void
  setParameters(std::shared_ptr<Base::GovernorParameters> parameters) final;
  void initializeStates(Real Pref) final;
  Real step(Real Omega, Real dt) final;
};

} // namespace Signal
} // namespace CPS
