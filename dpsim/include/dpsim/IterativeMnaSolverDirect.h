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
#include <memory>
#include <unordered_map>
#include <vector>

#include <dpsim/Config.h>
#include <dpsim/DataLogger.h>
#include <dpsim/DenseLUAdapter.h>
#include <dpsim/DirectLinearSolver.h>
#include <dpsim/DirectLinearSolverConfiguration.h>
#include <dpsim/Solver.h>
#ifdef WITH_KLU
#include <dpsim/KLUAdapter.h>
#endif
#include <dpsim/SparseLUAdapter.h>
#ifdef WITH_CUDA
#include <dpsim/GpuDenseAdapter.h>
#ifdef WITH_CUDA_SPARSE
#include <dpsim/GpuSparseAdapter.h>
#endif
#ifdef WITH_MAGMA
#include <dpsim/GpuMagmaAdapter.h>
#endif
#endif
#include <dpsim-models/AttributeList.h>
#include <dpsim-models/SimPowerComp.h>
#include <dpsim-models/SimSignalComp.h>
#include <dpsim-models/Solver/MNANonlinearVariableCompInterface.h>
#include <dpsim-models/Solver/MNASwitchInterface.h>
#include <dpsim-models/Solver/MNAVariableCompInterface.h>
#include <dpsim/MNASolverDirect.h>

namespace DPsim {

template <typename VarType>
class IterativeMnaSolverDirect : public MnaSolverDirect<VarType> {

public:
  IterativeMnaSolverDirect(
      String name, CPS::Domain domain = CPS::Domain::EMT,
      CPS::Logger::Level logLevel = CPS::Logger::Level::info)
      : MnaSolverDirect<VarType>(name, domain, logLevel) {
    MnaSolverDirect<VarType>::mImplementationInUse =
        DirectLinearSolverImpl::KLU;
  }

  virtual ~IterativeMnaSolverDirect() = default;

  virtual void initialize() override;

protected:
  // Delta of iterative Solutions
  Matrix mLeftStep;
  // Number of Iterations per step
  unsigned iterations = 0;
  // If mRightSideVector deviates less than Epsilon per element from the
  // result of the system defining node equations, mesh equations
  // and auxhiliary equations (calculationError),
  // the solution is good enough
  bool isConverged = true;
  Real Epsilon = 0.0001;
  Matrix calculationError;
  Real calculationErrorElement;

  // List of all nonlinear component function contributions
  std::vector<const Matrix *> mNonlinearFunctionStamps;
  // Sum of all nonlinear component function contributions
  Matrix mNonlinearFunctionResult = Matrix::Zero(1, 1);

  CPS::MNANonlinearVariableCompInterface::NonlinearList
      mMNANonlinearVariableComponents;

  using MnaSolverDirect<VarType>::mVariableSystemMatrix;
  using MnaSolverDirect<VarType>::mBaseSystemMatrix;
  using MnaSolverDirect<VarType>::mMNAIntfSwitches;
  using MnaSolverDirect<VarType>::mDirectLinearSolverVariableSystemMatrix;
  using MnaSolverDirect<VarType>::mRecomputationTimes;
  using MnaSolverDirect<VarType>::mNumRecomputations;
  using MnaSolverDirect<VarType>::mMNAIntfVariableComps;
  using MnaSolverDirect<VarType>::mRightSideVector;

  using MnaSolver<VarType>::mNumMatrixNodeIndices;

  void solveWithSystemMatrixRecomputation(Real time,
                                          Int timeStepCount) override;
  virtual void recomputeSystemMatrix(Real time) override;
  std::shared_ptr<CPS::Task> createSolveTaskRecomp() override;
};
} // namespace DPsim