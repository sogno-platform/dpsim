/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim-models/Definitions.h>

namespace CPS {
namespace Base {

	struct ExciterParameters {
		/// Transducer time constant (s)
		Real Tr;
		/// Pole of the regulator inherent dynamic
		Real Tb;
		/// Zero of the regulator inherent dynamic
		Real Tc;
		/// Amplifier time constant
		Real Ta;
		/// Amplifier gain
		Real Ka;
		/// Field circuit time constant
		Real Tef;
		/// Field circuit integral deviation
		Real Kef;
		/// Stabilizer time constant
		Real Tf;
		/// Stabilizer gain
		Real Kf;
		/// First ceiling coefficient
		Real Aef;
		/// Second ceiling coefficient
		Real Bef;
		///
		Real MaxVr;
		///
		Real MinVr;
	};

	/// @brief Base model for exciters
	class Exciter {

		private:
        	ExciterType mExciterType = ExciterType::DC1Simp;

		public:
			/// 
        	void setExciterType(ExciterType exciterType) {mExciterType = exciterType;};

			///
			virtual void setParameters(ExciterParameters parameters) = 0;

			/// Initializes exciter variables
			virtual void initialize(Real Vh_init, Real Ef_init) = 0;

			/// @param V_pss: Output of PSS
			virtual Real step(Real Vd, Real Vq, Real dt, Real Vpss = 0) = 0;
	};
}
}


