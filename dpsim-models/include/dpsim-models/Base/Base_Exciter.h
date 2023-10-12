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

	class ExciterParameters {
		public:
			ExciterParameters() { };
			virtual ~ExciterParameters() = default;
	 };

	/// @brief Base model for exciters
	class Exciter {

		private:
        	ExciterType mExciterType = ExciterType::DC1Simp;

		public:
			/// 
        	void setExciterType(ExciterType exciterType) {mExciterType = exciterType;};

			///
			virtual void setParameters(std::shared_ptr<Base::ExciterParameters> parameters) = 0;

			/// Initializes exciter variables
			virtual void initialize(Real Vh_init, Real Ef_init) = 0;

			/// @param V_pss: Output of PSS
			virtual Real step(Real Vd, Real Vq, Real dt, Real Vpss = 0) = 0;
	};
}
}


