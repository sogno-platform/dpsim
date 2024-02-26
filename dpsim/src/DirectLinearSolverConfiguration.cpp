/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
*                     EONERC, RWTH Aachen University
*
* This Source Code Form is subject to the terms of the Mozilla Public
* License, v. 2.0. If a copy of the MPL was not distributed with this
* file, You can obtain one at https://mozilla.org/MPL/2.0/.
*********************************************************************************/

#include <dpsim/DirectLinearSolverConfiguration.h>

using namespace DPsim;

namespace DPsim {
DirectLinearSolverConfiguration::DirectLinearSolverConfiguration() {
  mScalingMethod = SCALING_METHOD::MAX_SCALING;
  mPartialRefactorizationMethod =
      PARTIAL_REFACTORIZATION_METHOD::NO_PARTIAL_REFACTORIZATION;
  mUseBTF = USE_BTF::DO_BTF;
  mFillInReductionMethod = FILL_IN_REDUCTION_METHOD::AMD;
}

void DirectLinearSolverConfiguration::setFillInReductionMethod(
    FILL_IN_REDUCTION_METHOD fillInReductionMethod) {
  mFillInReductionMethod = fillInReductionMethod;
}

void DirectLinearSolverConfiguration::setScalingMethod(
    SCALING_METHOD scalingMethod) {
  mScalingMethod = scalingMethod;
}

void DirectLinearSolverConfiguration::setPartialRefactorizationMethod(
    PARTIAL_REFACTORIZATION_METHOD partialRefactorizationMethod) {
  mPartialRefactorizationMethod = partialRefactorizationMethod;
}

void DirectLinearSolverConfiguration::setBTF(USE_BTF useBTF) {
  mUseBTF = useBTF;
}

SCALING_METHOD DirectLinearSolverConfiguration::getScalingMethod() const {
  return mScalingMethod;
}

FILL_IN_REDUCTION_METHOD
DirectLinearSolverConfiguration::getFillInReductionMethod() const {
  return mFillInReductionMethod;
}

PARTIAL_REFACTORIZATION_METHOD
DirectLinearSolverConfiguration::getPartialRefactorizationMethod() const {
  return mPartialRefactorizationMethod;
}

USE_BTF
DirectLinearSolverConfiguration::DirectLinearSolverConfiguration::getBTF()
    const {
  return mUseBTF;
}

String DirectLinearSolverConfiguration::getScalingMethodString() const {
  switch (mScalingMethod) {
  case SCALING_METHOD::MAX_SCALING:
    return "max scaling";
  case SCALING_METHOD::SUM_SCALING:
    return "sum scaling";
  case SCALING_METHOD::NO_SCALING:
    return "without scaling";
  default:
    return "using the solver's default scaling method";
  }
}

String DirectLinearSolverConfiguration::getFillInReductionMethodString() const {
  switch (mFillInReductionMethod) {
  case FILL_IN_REDUCTION_METHOD::AMD_NV:
    return "AMD using a preference for non-varying entries";
  case FILL_IN_REDUCTION_METHOD::AMD_RA:
    return "AMD using a right-arranging of varying entries";
  case FILL_IN_REDUCTION_METHOD::COLAMD:
    return "COLAMD";
  case FILL_IN_REDUCTION_METHOD::AMD:
    return "AMD";
  default:
    return "using the solver's default fill-in reduction";
  }
}

String
DirectLinearSolverConfiguration::getPartialRefactorizationMethodString() const {
  switch (mPartialRefactorizationMethod) {
  case PARTIAL_REFACTORIZATION_METHOD::REFACTORIZATION_RESTART:
    return "with refactorization restart";
  case PARTIAL_REFACTORIZATION_METHOD::NO_PARTIAL_REFACTORIZATION:
    return "without partial refactorization";
  case PARTIAL_REFACTORIZATION_METHOD::FACTORIZATION_PATH:
    return "with factorization path";
  default:
    return "with unknown method";
  }
}

String DirectLinearSolverConfiguration::getBTFString() const {
  switch (mUseBTF) {
  case USE_BTF::DO_BTF:
    return "with BTF";
  case USE_BTF::NO_BTF:
  default:
    return "without BTF";
  }
}
} // namespace DPsim
