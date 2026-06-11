// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <dpsim-models/Base/Base_Turbine.h>
#include <dpsim-models/Logger.h>
#include <dpsim-models/SimSignalComp.h>

namespace CPS {
namespace Signal {

class SteamTurbineParameters : public Base::TurbineParameters,
                               public SharedFactory<SteamTurbineParameters> {
public:
  /// Power fraction of the high-pressure stage
  Real Fhp = 0;
  /// Power fraction of the intermediate-pressure stage
  Real Fip = 0;
  /// Power fraction of the low-pressure stage
  Real Flp = 0;
  /// Time constant of main inlet volume and steam chest (s)
  Real Tch = 0;
  /// Time constant of reheater (s)
  Real Trh = 0;
  /// Time constant of crossover piping and LP inlet volumes (s)
  Real Tco = 0;
};

/// Three-stage steam turbine (tandem compound: HP/IP/LP)
/// Ref.: A. Roehder, B. Fuchs, J. Massman, M. Quester, A. Schnettler,
///       "Transmission system stability assessment within an integrated grid
///       development process"
class SteamTurbine : public SimSignalComp,
                     public Base::Turbine,
                     public SharedFactory<SteamTurbine> {
private:
  std::shared_ptr<SteamTurbineParameters> mParameters;

  // ### State variables at time k — loggable via .attr() ###
  /// High-pressure stage power
  const Attribute<Real>::Ptr mPhp;
  /// Intermediate-pressure stage power
  const Attribute<Real>::Ptr mPip;
  /// Low-pressure stage power
  const Attribute<Real>::Ptr mPlp;
  /// Mechanical output power (pu)
  const Attribute<Real>::Ptr mPm;

  // ### Staging variables for next step ###
  Real mPhp_next = 0;
  Real mPip_next = 0;
  Real mPlp_next = 0;

public:
  explicit SteamTurbine(const String &name)
      : SimSignalComp(name, name), mPhp(mAttributes->create<Real>("Php")),
        mPip(mAttributes->create<Real>("Pip")),
        mPlp(mAttributes->create<Real>("Plp")),
        mPm(mAttributes->create<Real>("Pm")) {}

  SteamTurbine(const String &name, CPS::Logger::Level logLevel);

  void setParameters(std::shared_ptr<Base::TurbineParameters> parameters) final;
  void initializeStates(Real Pminit) final;
  Real step(Real Pgv, Real dt) final;
};

} // namespace Signal
} // namespace CPS
