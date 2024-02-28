/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim-models/Base/Base_Exciter.h>
#include <dpsim-models/Logger.h>
#include <dpsim-models/SimSignalComp.h>

namespace CPS {
namespace Signal {

class ExciterStaticParameters : public Base::ExciterParameters,
                                public SharedFactory<ExciterStaticParameters> {

public:
  /// Transducer time constant (s) (optional parameter)
  Real Tr = 0;
  /// Time constant of the numerator in the lead lag block (s)
  Real Ta = 0;
  /// Time constant of the denumerator in the lead lag block (s)
  Real Tb = 0;
  /// Time constant of the exciter machine (s)
  Real Te = 0;
  /// Proportional controller with the gain Ka
  Real Ka = 0;

  /// Maximum EMF
  Real MaxEfd = 0;
  /// Minimum EMF
  Real MinEfd = 0;

  /// Proportional gain of anti-Windup
  Real Kbc = 0;
};

/// Static Excitation System
/// Ref.: MatPAT Paper "Transmission System stability assesment within an integrated grid develpmpent process" Andreas Roehred
class ExciterStatic : public Base::Exciter,
                      public SimSignalComp,
                      public SharedFactory<ExciterStatic> {

private:
  /// Exciter Parameters
  std::shared_ptr<ExciterStaticParameters> mParameters;
  /// Set point of the exciter
  Real mVref = 0;
  /// Measured voltage at the terminal of generator (Input of the exciter) at step k
  Real mVh = 0;
  /// Transducer output at time k and k+1
  Real mVr = 0;
  Real mVr_prev = 0;
  /// Auxilary state variable from partial fraction decompostion (1+sT_A)/(1+sT_B) at step k and k+1
  Real mXb = 0;
  Real mXb_prev = 0;
  /// Input of the first lead lag block at time k (=Vref-Vh+Vpss)
  Real mVin = 0;
  /// Input of the second lag block 1/(1+sTe)
  Real mVe;
  /// Exciter output (EMF of the generator) at step k+1
  Real mEfd = 0;
  /// Exciter output (EMF of the generator) at step k+1 after the windup
  Real mEfdLim = 0;

  /// Auxiliar parameters
  Real mCa;
  Real mCb;

public:
  /// Constructor
  ExciterStatic(const String &name,
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