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
/// AVR model type 1
/// Ref.: Milano - Power system modelling and scripting, page 363
class Exciter : public SimSignalComp, public SharedFactory<Exciter> {

private:
  // ### Exciter Parameters ####
  /// Ampliﬁer time constant (s)
  Real mTa;
  /// Ampliﬁer gain (pu/pu)
  Real mKa;
  /// Field circuit integral deviation
  Real mKe;
  /// Field circuit time constant (s)
  Real mTe;
  /// Stabilizer gain (s pu/pu)
  Real mKf;
  /// Stabilizer time constant (s)
  Real mTf;
  /// Measurement time constant (s)
  Real mTr;

  /// Reference voltage
  Real mVref = 0;
  /// Output of voltage transducer at time k-1
  Real mVm_prev = 0;
  /// Output of stablizing feedback at time k-1
  Real mVis_prev = 0;
  /// Output of ceiling function at time k-1
  Real mVse_prev = 0;
  /// Output of regulator output at time k-1
  Real mVr_prev = 0;
  /// Exciter output at time k-1
  Real mEf_prev = 0;
  /// Maximum regulator voltage (p.u.)
  Real mMaxVr;
  /// Minumum regulator voltage (p.u.)
  Real mMinVr;

protected:
  /// Output of voltage transducer at time k-1
  const Attribute<Real>::Ptr mVm;
  /// Input of voltage transducer
  const Attribute<Real>::Ptr mVh;
  /// Output of stablizing feedback at time k
  const Attribute<Real>::Ptr mVis;
  /// Regulator output at time k
  const Attribute<Real>::Ptr mVr;
  /// Output of ceiling function at time k-1
  const Attribute<Real>::Ptr mVse;
  /// Exciter output at time k (induced emf by the field current under no-load conditions)
  const Attribute<Real>::Ptr mEf;

public:
  ///
  explicit Exciter(const String &name) : SimSignalComp(name, name) {}
  /// Constructor with log level
  Exciter(const String &name, CPS::Logger::Level logLevel);

  /// Initializes exciter parameters
  void setParameters(Real Ta, Real Ka, Real Te, Real Ke, Real Tf, Real Kf,
                     Real Tr, Real maxVr = 5.0, Real minVr = -5.0);
  /// Initializes exciter variables
  void initialize(Real Vh_init, Real Vf_init);
  /// Performs an step to update field voltage value
  Real step(Real mVd, Real mVq, Real dt);
};
} // namespace Signal
} // namespace CPS
