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

	class ExciterDC1SimpParameters :
		public Base::ExciterParameters,
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

	/// AVR model type 1
	/// Simplified model of IEEE DC1 type exciter. It does not model the time constants
	/// Tb and Tc which are normally small and thereby ignored. 
	/// Ref.: Milano - Power system modelling and scripting, page 363
	
	class ExciterDC1Simp :
		public Base::Exciter,
		public SimSignalComp,
		public SharedFactory<ExciterDC1Simp> {

	private: 
		// ### Exciter Parameters ####
		/// Ampliﬁer time constant (s)
		Real mTa;
		/// Ampliﬁer gain (pu/pu)
		Real mKa;
		/// Field circuit integral deviation
		Real mKef;
		/// Field circuit time constant (s)
		Real mTef;
		/// Stabilizer gain (s pu/pu)
		Real mKf;
		/// Stabilizer time constant (s)
		Real mTf;
		/// Measurement time constant (s)
		Real mTr;
		/// First ceiling coefficient
		Real mAef;
		/// Second ceiling coefficient
		Real mBef;
		/// Maximum amplifier output (p.u.)
		Real mMaxVa;
		/// Minumum amplifier output (p.u.)
		Real mMinVa;

		/// Reference voltage (with effect of PSS)
		Real mVref = 0;
		/// Output of voltage transducer at time k-1
		Real mVr_prev = 0;
		/// Output of stablizing feedback at time k-1
		Real mVf_prev = 0;
		/// Output of amplifier output at time k-1
		Real mVa_prev = 0;
		/// Exciter output at time k-1
		Real mEf_prev = 0;
		
		/// Input of voltage transducer
		Real mVh;
		/// Output of voltage transducer at time k-1
		Real mVr;
		/// Output of stablizing feedback at time k
		Real mVf;
		/// Input of amplifier at time k
		Real mVin;
		/// Output of amplifier at time k
		Real mVa;
		/// Amplifier output at time k
		Real mVsat;
		/// Exciter output at time k (induced emf by the field current under no-load conditions)
		Real mEf;

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