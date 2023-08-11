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
	/// Simplified Type ST1 exciter (Regulator time constant Ta=0, without transient gain reduction: Tb=Tc=0)
	/// Used in Kundur two areas system
	/// Ref.: Kundur, 9.815
	class ExciterST1Simp :
		public Base::Exciter,
		public SimSignalComp,
		public SharedFactory<ExciterST1Simp> {

	private: 
		// ### Exciter Parameters ####
		/// Transducer time constant (s)
		Real mTr;
		///  Regulator gain
		Real mKa;


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
		ExciterST1Simp(const String & name, Logger::Level logLevel = Logger::Level::debug);
		/// Initializes exciter parameters
		void setParameters(Base::ExciterParameters parameters) override;
		/// Initializes exciter variables
		void initialize(Real Vh_init, Real Vf_init) override;
		/// Performs an step to update field voltage value
		Real step(Real Vd, Real Vq, Real dt, Real Vpss = 0) override;
	};
}
}
