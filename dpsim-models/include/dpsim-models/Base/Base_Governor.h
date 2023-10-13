/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

namespace CPS {
namespace Base {

	class GovernorParameters {
		public:
			GovernorParameters() { };
			virtual ~GovernorParameters() = default;
	 };

	/// @brief Base model for Governors
	class Governor {

		public:

			///
			virtual void setParameters(std::shared_ptr<Base::GovernorParameters> parameters) = 0;

			/// Initializes Governor variables
			virtual void initialize(Real PmRef) = 0;

			/// @param V_pss: Output of PSS
			virtual Real step(Real Omega, Real dt) = 0;
	};
}
}


