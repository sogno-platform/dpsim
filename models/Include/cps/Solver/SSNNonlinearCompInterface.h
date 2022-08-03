/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <cps/Config.h>
#include <cps/Definitions.h>
#include <cps/AttributeList.h>

namespace CPS {
	/// SSN interface to be used by SSN elements that require recomputing of the Jacobian system matrix
	class SSNNonlinearCompInterface: virtual public AttributeList {
	public:

		typedef std::shared_ptr<SSNNonlinearCompInterface> Ptr;
		typedef std::vector<Ptr> List;

		const Attribute<Matrix>::Ptr SSN_Function_Result;

		virtual void ssnUpdate(const Matrix& leftVector) = 0;

	protected:
		SSNNonlinearCompInterface(): SSN_Function_Result(Attribute<Matrix>::createDynamic("SSN_Function_Result", mAttributes)) { }
	};
}
