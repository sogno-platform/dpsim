/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim-models/SimSignalComp.h>
#include <dpsim-models/Base/Base_Exciter.h>
#include <dpsim-models/Logger.h>

namespace CPS {
namespace Signal {
	/// Type DC1 exciter
	/// Ref.: IEEE Recommended Practice for Excitation System Models for Power System Stability Studies
	class ExciterDC1 :
		public Base::Exciter,
		public SimSignalComp,
		public SharedFactory<ExciterDC1> {

	private: 
		// ### Exciter Parameters ####
		/// Transducer time constant (s)
		Real mTr;
		/// Amplifier time constant
		Real mTa;
		/// Pole of the regulator inherent dynamic
		Real mTb;
		/// Zero of the regulator inherent dynamic
		Real mTc;
		/// Field circuit time constant
		Real mTef;
		/// Stabilizer time constant
		Real mTf;
		/// Amplifier gain
		Real mKa;
		/// Field circuit integral deviation
		Real mKef;
		/// Stabilizer gain
		Real mKf;
		/// First ceiling coefficient
		Real mAef;
		/// Second ceiling coefficient
		Real mBef;
		///
		Real mMaxVa;
		///
		Real mMinVa;

		/// Transducer input at time k-1
		Real mVh = 0;
		/// Transducer output at time k
		Real mVr = 0;
		/// Transducer output at time k-1
		Real mVr_prev = 0;
		/// Output of stablizing feedback at time k
		Real mVf = 0;
		/// Output of stablizing feedback at time k-1
		Real mVf_prev = 0;
		///
		Real mVb = 0;
		/// 
		Real mVb_prev = 0;
		/// Output of Regulator at time k
		Real mVin = 0;
		/// Output of regulator at time k-1
		Real mVin_prev = 0;
		/// Output of amplifier at time k
		Real mVa = 0;
		/// Output of amplifiert at time k-1
		Real mVa_prev = 0;
		///
		/// Exciter output at time k (induced emf by the field current under no-load conditions)
		Real mEf = 0;
		/// Exciter output at time k-1
		Real mEf_prev = 0;
		/// Saturation function
		Real mVsat = 0;
		/// 
		Real mVref = 0;

	public:
		/// Constructor
		ExciterDC1(const String & name, Logger::Level logLevel = Logger::Level::debug);
		/// Initializes exciter parameters
		void setParameters(Base::ExciterParameters parameters) override;
		/// Initializes exciter variables
		void initialize(Real Vh_init, Real Vf_init) override;
		/// Performs an step to update field voltage value
		Real step(Real Vd, Real Vq, Real dt, Real Vpss = 0) override;
	};
}
}
