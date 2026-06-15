// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <dpsim-models/Base/Base_PSS.h>
#include <dpsim-models/Logger.h>
#include <dpsim-models/SimSignalComp.h>

namespace CPS {
namespace Signal {

class PSS1AParameters : public Base::PSSParameters,
                        public SharedFactory<PSS1AParameters> {

public:
  /// Gain for active power (pu/pu)
  Real Kp = 0;
  /// Gain for bus voltage magnitude (pu/pu)
  Real Kv = 0;
  /// Stabilizer gain (pu/pu)
  Real Kw = 0;
  /// First stabilizer time constant (s)
  Real T1 = 0;
  /// Second stabilizer time constant (s)
  Real T2 = 0;
  /// Third stabilizer time constant (s)
  Real T3 = 0;
  /// Fourth stabilizer time constant (s)
  Real T4 = 0;
  /// Max stabilizer output signal (pu)
  Real Vs_max = 0;
  /// Min stabilizer output signal (pu)
  Real Vs_min = 0;
  /// Wash-out time constant (s)
  Real Tw = 0;
};

/// Power system stabilizer type 1A
/// Ref.: Milano - Power system modelling and scripting, page 371
class PSS1A : public SimSignalComp,
              public Base::PSS,
              public SharedFactory<PSS1A> {

private:
  std::shared_ptr<PSS1AParameters> mParameters;
  Real mA;
  Real mB;

  Real mV1_prev;
  Real mV2_prev;
  Real mV3_prev;
  Real mVs_prev;
  Real mOmega_prev;
  Real mActivePower_prev;
  Real mVh_prev;

private:
  /// Wash-out output
  Real mV1;
  /// Output of the first phase compensation block
  Real mV2;
  /// Output of the second phase compensation block
  Real mV3;
  /// PSS output at t=k
  Real mVs;

public:
  explicit PSS1A(const String &name) : SimSignalComp(name, name) {}
  PSS1A(const String &name, CPS::Logger::Level logLevel);

  void setParameters(std::shared_ptr<Base::PSSParameters> parameters) final;
  void initializeStates(Real omega, Real activePower, Real Vd, Real Vq) final;
  Real step(Real omega, Real activePower, Real Vd, Real Vq, Real dt) final;
};
} // namespace Signal
} // namespace CPS
