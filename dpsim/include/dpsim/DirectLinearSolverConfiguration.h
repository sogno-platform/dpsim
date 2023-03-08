/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
*                     EONERC, RWTH Aachen University
*
* This Source Code Form is subject to the terms of the Mozilla Public
* License, v. 2.0. If a copy of the MPL was not distributed with this
* file, You can obtain one at https://mozilla.org/MPL/2.0/.
*********************************************************************************/

#pragma once

#include <dpsim/Config.h>
#include <dpsim/Definitions.h>

namespace DPsim
{
	// Define method to scale the input matrix
	enum class SCALING_METHOD {
		NO_SCALING,
		SUM_SCALING,
		MAX_SCALING
	};

	// Define preordering to reduce fill-in
	enum class FILL_IN_REDUCTION_METHOD {
		AMD,
		AMD_NV,
		AMD_RA,
		COLAMD
	};

	// Define partial refactorization method, if applicable
	enum class PARTIAL_REFACTORIZATION_METHOD {
		NO_PARTIAL_REFACTORIZATION,
		FACTORIZATION_PATH,
		REFACTORIZATION_RESTART
	};

	// Define BTF usage
	enum class USE_BTF {
		NO_BTF,
		DO_BTF
	};

	class DirectLinearSolverConfiguration
	{
		SCALING_METHOD m_scalingMethod;
		FILL_IN_REDUCTION_METHOD m_fillInReductionMethod;
		PARTIAL_REFACTORIZATION_METHOD m_partialRefactorizationMethod;
		USE_BTF m_useBTF;

		public:
		DirectLinearSolverConfiguration();

		void setFillInReductionMethod(FILL_IN_REDUCTION_METHOD);

		void setScalingMethod(SCALING_METHOD);

		void setPartialRefactorizationMethod(PARTIAL_REFACTORIZATION_METHOD);

		void setBTF(USE_BTF);

		SCALING_METHOD getScalingMethod() const;

		FILL_IN_REDUCTION_METHOD getFillInReductionMethod() const;

		PARTIAL_REFACTORIZATION_METHOD getPartialRefactorizationMethod() const;

		USE_BTF getBTF() const;
	};
}
