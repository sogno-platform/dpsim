/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <cps/IdentifiedObject.h>
#include <cps/Logger.h>

namespace CPS {
namespace Signal {
	/// Exciter model
	class Exciter : public IdentifiedObject {
	protected:
		Real mTa;
		Real mKa;
		Real mKe;
		Real mTe;
		Real mKf;
		Real mTf;
		Real mTr;

		Real mLad;
		Real mRfd;

		/// Reference voltage
		Real mVref = 0;
		/// Output of voltage transducer
		Real mVm = 0;
		/// Input of voltage transducer
		Real mVh = 0;
		/// Output of stablizing feedback
		Real mVis = 0;
		/// Output of se function
		Real mVse = 0;
		/// Regulator output
		Real mVr = 0;
		/// Exciter output
		Real mVf = 0;
		/// Auxiliar variable
		Real mVfl = 0;
		/// Initial field votlage
		Real mVf_init;

	public:
		///
		Exciter(String name) : IdentifiedObject(name) { }

		/// Initializes exciter parameters
		void setParameters(Real Ta, Real Ka, Real Te, Real Ke, Real Tf, Real Kf, Real Tr, Real Lad, Real Rfd);
		/// Initializes exciter variables
		void initialize(Real Vh_init, Real Vf_init);
		/// Performs an step to update field voltage value
		Real step(Real mVd, Real mVq, Real Vref, Real dt);
	};
}
}
