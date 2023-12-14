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

	class VSIControlParameters {
		public:
			VSIControlParameters() { };
			virtual ~VSIControlParameters() = default;
	 };

	/// @brief Base model for VSI controllers in dq reference frame
	class VSIControlDQ {

		protected:
			///
			Real mTimeStep;

		public:

			///
			virtual void setParameters(std::shared_ptr<Base::VSIControlParameters> parameters) = 0;

			/// Initializes
			virtual void initialize(const Complex& Vsref_dq, const Complex& Vcap_dq, const Complex& Ifilter_dq, Real time_step) = 0;

			/// 
			virtual Complex step(const Complex& Vcap_dq, const Complex& Ifilter_dq) = 0;

	};
}
}


