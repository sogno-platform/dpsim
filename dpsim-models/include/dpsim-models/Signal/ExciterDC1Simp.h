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

class ExciterDC1SimpParameters
    : public Base::ExciterParameters,
      public SharedFactory<ExciterDC1SimpParameters> {

public:
  /// Transducer time constant (s)
  Real Tr = 0;
  /// Amplifier time constant
  Real Ta = 0;
  /// Field circuit time constant
  Real Tef = 0;
  /// Stabilizer time constant
  Real Tf = 0;
  /// Amplifier gain
  Real Ka = 0;
  /// Field circuit integral deviation
  Real Kef = 0;
  /// Stabilizer gain
  Real Kf = 0;
  /// First ceiling coefficient
  Real Aef = 0;
  /// Second ceiling coefficient
  Real Bef = 0;
  ///
  Real MaxVa = 0;
  ///
  Real MinVa = 0;
};

	private: 
		/// Exciter Parameters
		std::shared_ptr<ExciterDC1SimpParameters> mParameters;

class ExciterDC1Simp : public Base::Exciter,
                       public SimSignalComp,
                       public SharedFactory<ExciterDC1Simp> {

	public:
		/// Constructor
		ExciterDC1Simp(const String & name, CPS::Logger::Level logLevel = Logger::Level::info);
		/// Initializes exciter parameters
		void setParameters(std::shared_ptr<Base::ExciterParameters> parameters) final;
		/// Initializes exciter variables
		void initialize(Real Vh_init, Real Vf_init) final;
		/// Performs an step to update field voltage value
		Real step(Real mVd, Real mVq, Real dt, Real Vpss = 0) final;
	};
}
}