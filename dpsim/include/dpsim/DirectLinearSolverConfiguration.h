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

namespace DPsim {
// Define method to scale the input matrix
enum class SCALING_METHOD { NO_SCALING, SUM_SCALING, MAX_SCALING };

// Define preordering to reduce fill-in
enum class FILL_IN_REDUCTION_METHOD {
  AMD,
  AMD_NV, // AMD ordering with non-varying preference
  AMD_RA, // AMD ordering with right arranging of varying entries
  COLAMD
};

// Define partial refactorization method, if applicable
enum class PARTIAL_REFACTORIZATION_METHOD {
  NO_PARTIAL_REFACTORIZATION,
  FACTORIZATION_PATH,
  REFACTORIZATION_RESTART
};

// Define BTF usage, if applicable
enum class USE_BTF { NO_BTF, DO_BTF };

class DirectLinearSolverConfiguration {
  SCALING_METHOD mScalingMethod;
  FILL_IN_REDUCTION_METHOD mFillInReductionMethod;
  PARTIAL_REFACTORIZATION_METHOD mPartialRefactorizationMethod;
  USE_BTF mUseBTF;

public:
  DirectLinearSolverConfiguration();

  void setFillInReductionMethod(FILL_IN_REDUCTION_METHOD fillInReductionMethod);

  void setScalingMethod(SCALING_METHOD scalingMethod);

  void setPartialRefactorizationMethod(
      PARTIAL_REFACTORIZATION_METHOD partialRefactorizationMethod);

  void setBTF(USE_BTF useBTF);

  SCALING_METHOD getScalingMethod() const;

  FILL_IN_REDUCTION_METHOD getFillInReductionMethod() const;

  PARTIAL_REFACTORIZATION_METHOD getPartialRefactorizationMethod() const;

  USE_BTF getBTF() const;

  String getScalingMethodString() const;

  String getFillInReductionMethodString() const;

  String getPartialRefactorizationMethodString() const;

  String getBTFString() const;
};
} // namespace DPsim
