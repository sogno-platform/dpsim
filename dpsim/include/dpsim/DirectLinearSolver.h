/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
*                     EONERC, RWTH Aachen University
*
* This Source Code Form is subject to the terms of the Mozilla Public
* License, v. 2.0. If a copy of the MPL was not distributed with this
* file, You can obtain one at https://mozilla.org/MPL/2.0/.
*********************************************************************************/

#pragma once

#include <bitset>
#include <iostream>
#include <list>
#include <unordered_map>
#include <vector>

#include <dpsim-models/Logger.h>
#include <dpsim/Config.h>
#include <dpsim/Definitions.h>
#include <dpsim/DirectLinearSolverConfiguration.h>

namespace DPsim {
class DirectLinearSolver {
public:
  /// Constructor
  DirectLinearSolver() = default;

  /// Destructor
  virtual ~DirectLinearSolver() = default;

  /// Copy Constructor
  DirectLinearSolver(const DirectLinearSolver &) = default;

  /// Copy Assignment Operator
  DirectLinearSolver &operator=(const DirectLinearSolver &) = default;

  /// Move Constructor
  DirectLinearSolver(DirectLinearSolver &&) = default;

  /// Move Assignment Operator
  DirectLinearSolver &operator=(DirectLinearSolver &&) = default;

  /// Constructor with Logger
  DirectLinearSolver(CPS::Logger::Log log) : mSLog(log) {
    // no further default configuration of DirectLinearSolver or logger
  }

  /// preprocessing function pre-ordering and scaling the matrix
  virtual void preprocessing(
      SparseMatrix &systemMatrix,
      std::vector<std::pair<UInt, UInt>> &listVariableSystemMatrixEntries) = 0;

  /// factorization function with partial pivoting
  virtual void factorize(SparseMatrix &systemMatrix) = 0;

  /// refactorization without partial pivoting
  virtual void refactorize(SparseMatrix &systemMatrix) = 0;

  /// partial refactorization withouth partial pivoting
  virtual void partialRefactorize(
      SparseMatrix &systemMatrix,
      std::vector<std::pair<UInt, UInt>> &listVariableSystemMatrixEntries) = 0;

  /// solution function for a right hand side
  virtual Matrix solve(Matrix &rightSideVector) = 0;

  virtual void
  setConfiguration(DirectLinearSolverConfiguration &configuration) {
    mConfiguration = configuration;
    this->applyConfiguration();
  }

protected:
  /// Stores logger of solver class
  CPS::Logger::Log mSLog;

  /// Object that carries configuration options
  DirectLinearSolverConfiguration mConfiguration;

  virtual void applyConfiguration() {
    // no default application, configuration options vary for each solver
    // warn user that no configuration setting is used
    SPDLOG_LOGGER_WARN(mSLog, "Linear solver configuration is not used!");
  }
};
} // namespace DPsim
