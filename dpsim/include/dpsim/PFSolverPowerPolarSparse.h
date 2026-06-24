// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <dpsim/DirectLinearSolver.h>
#include <dpsim/PFSolverPowerPolar.h>

namespace DPsim {
/// Polar-coordinate powerflow solver that assembles a sparse Jacobian and reuses
/// the symbolic factorization across Newton-Raphson iterations (scales to large grids).
class PFSolverPowerPolarSparse : public PFSolverPowerPolar {
protected:
  /// Sparse Jacobian (row-major, fixed pattern, values updated in place each iteration)
  CPS::SparseMatrixRow mJsparse;
  /// Direct linear solver reused across iterations (symbolic factorization computed once)
  std::shared_ptr<DirectLinearSolver> mLinearSolver;
  /// Empty list: PF uses full refactorization, not partial refactorization
  std::vector<std::pair<CPS::UInt, CPS::UInt>> mVariableSystemMatrixEntries;

  /// Build the fixed sparsity pattern, create the linear solver and analyze the pattern once
  void setUpJacobianStorage() override;
  /// Fill the sparse Jacobian values in place (pattern unchanged)
  void calculateJacobian() override;
  /// Factorize (first iteration of a run) or refactorize, then solve
  void solveJacobianSystem() override;

  /// Construct the structural sparsity pattern of the Jacobian from the bus admittance matrix
  void buildJacobianPattern();
  /// Whether buses k and j are connected (off-diagonal Jacobian entry exists)
  bool isConnected(CPS::UInt k, CPS::UInt j);

public:
  /// Constructor to be used in simulation examples.
  PFSolverPowerPolarSparse(CPS::String name, const CPS::SystemTopology &system,
                           CPS::Real timeStep, CPS::Logger::Level logLevel);
  ///
  virtual ~PFSolverPowerPolarSparse(){};
};
} // namespace DPsim
