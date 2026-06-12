// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

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
  Real mTmax = 0;
  /// Minimum turbine output
  Real mTmin = 0;
  /// Droop
  Real mR = 0;
  /// Transient gain time constant (s)
  Real mT3 = 0;
  /// Power fraction time constant (s)
  Real mT4 = 0;
  /// Reheat time constant (s)
  Real mT5 = 0;
  /// Servo time constant (s)
  Real mTc = 0;
  /// Governor time constant (s)
  Real mTs = 0;
  /// Speed reference (pu)
  Real mOmRef = 0;
  /// Reference torque (pu)
  Real mTmRef = 0;

  // ### State variables (previous step) ###
  /// Governor output at time k-1
  Real mXg1_prev = 0;
  /// Servo output at time k-1
  Real mXg2_prev = 0;
  /// Reheat output at time k-1
  Real mXg3_prev = 0;

  // ### State variables (current step) ###
  /// Governor output at time k
  const Attribute<Real>::Ptr mXg1;
  /// Servo output at time k
  const Attribute<Real>::Ptr mXg2;
  /// Reheat output at time k
  const Attribute<Real>::Ptr mXg3;
  /// Mechanical Torque in pu (output of steam turbine)
  const Attribute<Real>::Ptr mTm;

public:
  TurbineGovernorType1(const String &name,
                       Logger::Level logLevel = Logger::Level::off);

  /// Initializes exciter parameters
  void setParameters(Real T3, Real T4, Real T5, Real Tc, Real Ts, Real R,
                     Real Tmin, Real Tmax, Real OmRef);
  /// Set steady-state initial values for all governor/turbine states (call after
  /// setParameters, before the first step).
  void initializeStates(Real TmRef);
  /// Performs an step to update field voltage value
  Real step(Real Omega, Real dt);
};
} // namespace Signal
} // namespace CPS
