/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
*                     EONERC, RWTH Aachen University
*
* This Source Code Form is subject to the terms of the Mozilla Public
* License, v. 2.0. If a copy of the MPL was not distributed with this
* file, You can obtain one at https://mozilla.org/MPL/2.0/.
*********************************************************************************/

#include <dpsim/DirectLinearSolverConfiguration.h>

using namespace DPsim;

namespace DPsim
{
	DirectLinearSolverConfiguration::DirectLinearSolverConfiguration()
	{
		m_scalingMethod = SCALING_METHOD::NO_SCALING;
		m_partialRefactorizationMethod = PARTIAL_REFACTORIZATION_METHOD::NO_PARTIAL_REFACTORIZATION;
	}

	void DirectLinearSolverConfiguration::setFillInReductionMethod(FILL_IN_REDUCTION_METHOD fillInReductionMethod)
	{
		m_fillInReductionMethod = fillInReductionMethod;
	}

	void DirectLinearSolverConfiguration::setScalingMethod(SCALING_METHOD scalingMethod)
	{
		m_scalingMethod = scalingMethod;
	}

	void DirectLinearSolverConfiguration::setPartialRefactorizationMethod(PARTIAL_REFACTORIZATION_METHOD partialRefactorizationMethod)
	{
		m_partialRefactorizationMethod = partialRefactorizationMethod;
	}

	void DirectLinearSolverConfiguration::setBTF(USE_BTF useBTF)
	{
		m_useBTF = useBTF;
	}

	SCALING_METHOD DirectLinearSolverConfiguration::getScalingMethod() const
	{
		return m_scalingMethod;
	}

	FILL_IN_REDUCTION_METHOD DirectLinearSolverConfiguration::getFillInReductionMethod() const
	{
		return m_fillInReductionMethod;
	}

	PARTIAL_REFACTORIZATION_METHOD DirectLinearSolverConfiguration::getPartialRefactorizationMethod() const
	{
		return m_partialRefactorizationMethod;
	}

	USE_BTF DirectLinearSolverConfiguration::DirectLinearSolverConfiguration::getBTF() const
	{
		return m_useBTF;
	}
}
