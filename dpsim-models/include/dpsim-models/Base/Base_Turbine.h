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

	class TurbineParameters {
		public:
			TurbineParameters() { };
			virtual ~TurbineParameters() = default;
	 };

	/// @brief Base model for Turbine
	class Turbine {

		public:

			///
			virtual void setParameters(std::shared_ptr<Base::TurbineParameters> parameters) = 0;

			/// Initializes Turbine variables
			virtual void initialize(Real PmInit) = 0;

			/// 
			virtual Real step(Real Pgv, Real dt) = 0;
	};
}
}


