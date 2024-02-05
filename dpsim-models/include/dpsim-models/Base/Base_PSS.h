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

	class PSSParameters {
		public:
			PSSParameters() { };
			virtual ~PSSParameters() = default;
	 };

	/// @brief Base model for exciters
	class PSS {

		public:
			///
			virtual void setParameters(std::shared_ptr<Base::PSSParameters> parameters) = 0;

			/// Initializes exciter variables
			virtual void initialize(Real omega, Real activePower, Real Vd, Real Vq) = 0;

			/// @param V_pss: Output of PSS
			virtual Real step(Real omega, Real activePower, Real Vd, Real Vq, Real dt) = 0;
	};
}
}


