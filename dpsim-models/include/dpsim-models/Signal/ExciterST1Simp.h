// SPDX-FileCopyrightText: 2025 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <dpsim-models/Base/Base_Exciter.h>
#include <dpsim-models/Logger.h>
#include <dpsim-models/SimSignalComp.h>

namespace CPS {
namespace Signal {

class ExciterST1Parameters : public Base::ExciterParameters,
                             public SharedFactory<ExciterST1Parameters> {

public:
  /// Transducer time constant (s)
  Real Tr = 0;
  /// Amplifier gain
  Real Ka = 0;
  ///
  Real MaxVa = 0;
  ///
  Real MinVa = 0;
};

/// Simplified Type ST1 exciter (Regulator time constant Ta=0, without transient gain reduction: Tb=Tc=0)
/// Used in Kundur two areas system
/// Ref.: Kundur, 9.815
class ExciterST1Simp : public Base::Exciter,
                       public SimSignalComp,
                       public SharedFactory<ExciterST1Simp> {

private:
  /// Exciter Parameters
  std::shared_ptr<ExciterST1Parameters> mParameters;

  /// Transducer input at time k-1
  Real mVh = 0;
  /// Transducer output at time k
  Real mVr = 0;
  /// Transducer output at time k-1
  Real mVr_prev = 0;
  /// Exciter output at time k (induced emf by the field current under no-load conditions)
  Real mEf = 0;
  ///
  Real mVref = 0;

public:
  /// Constructor
  ExciterST1Simp(const String &name,
                 Logger::Level logLevel = Logger::Level::info);
  /// Initializes exciter parameters
  void setParameters(std::shared_ptr<Base::ExciterParameters> parameters) final;
  /// Initializes exciter variables
  void initialize(Real Vh_init, Real Vf_init) final;
  /// Performs an step to update field voltage value
  Real step(Real Vd, Real Vq, Real dt, Real Vpss = 0) final;
};
} // namespace Signal
} // namespace CPS
