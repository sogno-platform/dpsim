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
		Real Tr = 0;
		/// Amplifier time constant
		Real Ta = 0;
		/// Pole of the regulator inherent dynamic
		Real Tb = 0;
		/// Zero of the regulator inherent dynamic
		Real Tc = 0;
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


