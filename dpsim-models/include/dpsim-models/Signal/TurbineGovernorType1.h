/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim-models/Logger.h>
#include <dpsim-models/SimSignalComp.h>

namespace CPS {
namespace Signal {

/// Turbine governor type 1 model
/// Ref.: Milano - Power system modelling and scripting, page 358
class TurbineGovernorType1 : public SimSignalComp,
                             public SharedFactory<TurbineGovernorType1> {
private:
  // ### Steam Turbine Parameters ####
  /// Maximum turbine output
  Real mTmax;
  /// Minimum turbine output
  Real mTmin;
  /// Droop
  Real mR;
  /// Transient gain time constant (s)
  Real mT3;
  /// Power fraction time constant (s)
  Real mT4;
  /// Reheat time constant (s)
  Real mT5;
  /// Servo time constant (s)
  Real mTc;
  /// Governor time constant (s)
  Real mTs;
  /// Speed reference (pu)
  Real mOmRef;
  /// Reference torque (pu)
  Real mTmRef;

  // ### Variables ###
  /// Mechanical speed at time k-1
  Real mOmega_prev;
  /// Governor output at time k-1
  Real mXg1_prev;
  /// Servo output at time k-1
  Real mXg2_prev;
  /// Reheat output at time k-1
  Real mXg3_prev;

protected:
  // ### Variables ###
  /// Governor output at time k
  const Attribute<Real>::Ptr mXg1;
  /// Servo output at time k
  const Attribute<Real>::Ptr mXg2;
  /// Reheat output at time k
  const Attribute<Real>::Ptr mXg3;
  /// Mechanical Torque in pu (output of steam turbine)
  const Attribute<Real>::Ptr mTm;

public:
  ///
  explicit TurbineGovernorType1(const String &name)
      : SimSignalComp(name, name) {}

  /// Constructor with log level
  TurbineGovernorType1(const String &name, CPS::Logger::Level logLevel);

  /// Initializes exciter parameters
  void setParameters(Real T3, Real T4, Real T5, Real Tc, Real Ts, Real R,
                     Real Tmin, Real Tmax, Real OmRef);
  ///
  void initialize(Real TmRef) override;
  /// Performs an step to update field voltage value
  Real step(Real Omega, Real dt);
};
} // namespace Signal
} // namespace CPS
