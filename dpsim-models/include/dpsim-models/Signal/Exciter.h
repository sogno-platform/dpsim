/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim-models/SimSignalComp.h>
#include <dpsim-models/Logger.h>

namespace CPS {
namespace Signal {
	/// Exciter model
	class Exciter :
		public SimSignalComp,
		public SharedFactory<Exciter> {
	protected:
		Real mTa;
		Real mKa;
		Real mKe;
		Real mTe;
		Real mKf;
		Real mTf;
		Real mTr;

		/// Reference voltage
		Real mVref = 0;
		/// Output of voltage transducer at time k-1
		const Attribute<Real>::Ptr mVm;
		/// Output of voltage transducer at time k-1
		Real mVm_prev = 0;
		/// Input of voltage transducer
		const Attribute<Real>::Ptr mVh;
		/// Output of stablizing feedback at time k
		const Attribute<Real>::Ptr mVis;
		/// Output of stablizing feedback at time k-1
		Real mVis_prev = 0;
		/// Output of ceiling function at time k-1
		const Attribute<Real>::Ptr mVse;
		/// Output of ceiling function at time k-1
		Real mVse_prev = 0;
		/// Regulator output at time k
		const Attribute<Real>::Ptr mVr;
		/// Output of regulator output at time k-1
		Real mVr_prev = 0;
		/// Exciter output at time k
		const Attribute<Real>::Ptr mVf;
		/// Exciter output at time k-1
		Real mVf_prev = 0;
		/// Maximum regulator voltage (p.u.)
		Real mMaxVr;
		/// Minumum regulator voltage (p.u.)
		Real mMinVr;

	public:
		///
		Exciter(String name) : SimSignalComp(name, name) { }

		/// Constructor with log level
		Exciter(String name, CPS::Logger::Level logLevel);

		/// Initializes exciter parameters
		void setParameters(Real Ta, Real Ka, Real Te, Real Ke, Real Tf, Real Kf, Real Tr, Real maxVr=1.0, Real minVr = -0.9);
		/// Initializes exciter variables
		void initialize(Real Vh_init, Real Vf_init);
		/// Performs an step to update field voltage value
		Real step(Real mVd, Real mVq, Real dt);
	};
}
}
