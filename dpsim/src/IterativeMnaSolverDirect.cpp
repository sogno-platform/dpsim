/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 ******************************************************************************/

#include <dpsim/IterativeMnaSolverDirect.h>
#include <dpsim/SequentialScheduler.h>

using namespace DPsim;
using namespace CPS;

template <typename VarType>
std::shared_ptr<CPS::Task>
IterativeMnaSolverDirect<VarType>::createSolveTaskRecomp() {
  return std::make_shared<
      typename IterativeMnaSolverDirect<VarType>::SolveTaskRecomp>(*this);
}

template <typename VarType>
void IterativeMnaSolverDirect<VarType>::recomputeSystemMatrix(Real time) {
  // Start from base matrix
  mVariableSystemMatrix = mBaseSystemMatrix;

  // Now stamp switches into matrix
  for (auto sw : mMNAIntfSwitches)
    sw->mnaApplySystemMatrixStamp(mVariableSystemMatrix);

  // Now stamp variable elements into matrix
  for (auto comp : mMNAIntfVariableComps)
    comp->mnaApplySystemMatrixStamp(mVariableSystemMatrix);

  auto start = std::chrono::steady_clock::now();
  mDirectLinearSolverVariableSystemMatrix->factorize(mVariableSystemMatrix);
  auto end = std::chrono::steady_clock::now();
  std::chrono::duration<Real> diff = end - start;
  mRecomputationTimes.push_back(diff.count());

  ++mNumRecomputations;
}

template <typename VarType>
void IterativeMnaSolverDirect<VarType>::solveWithSystemMatrixRecomputation(
    Real time, Int timeStepCount) {
  // Reset source vector
  this->mRightSideVector.setZero();
  // Add together the right side vector (computed by the components'
  // pre-step tasks)
  for (auto stamp : this->mRightVectorStamps)
    this->mRightSideVector += *stamp;

  //Number of Iterations per step
  unsigned iterations = 0;
  do {
    // Get switch and variable comp status and update system matrix and lu
    //factorization accordingly
    if (this->hasVariableComponentChanged()) {
      this->recomputeSystemMatrix(time);
    }

    // 	Calculate new delta solution vector:
    //	systemMatrix*leftStep = mRightSideVector-mNonlinearSSNfunctionResult
    //	Corresponds to Newton-Raphson:
    //
    //	f(x2) =~ f(x1) + Df(x1)*(x2-x1)
    //
    //	f(x) is the function vector containing the system describing
    //	equations WITHOUT injections and SSN history terms
    //	These are the right side source vector and are set equal to the
    //	function:
    //	f(x2) = mRightSideVector =~ f(x1) + Df(x1)*(x2-x1)
    // 	Subtracting the past function value leaves:
    //	systemMatrix*leftStep = mRightSideVector-(mBaseSystemMatrix*(**mLeftSideVector)+ mNonlinearSSNfunctionResult)
    // (mBaseSystemMatrix*(**mLeftSideVector)+ mNonlinearSSNfunctionResult) = f(x1),
    //	that is the old mRightSideVector of the previous step
    //	mRightSideVector is stamped by mna-prestep tasks and only updated each
    //	step, not iteration.

    //	mDirectLinearSolvers.solve takes a Matrix reference,
    //	thus we need a temporary variable as an argument
    Matrix temp = (this->mRightSideVector) -
                  ((this->mBaseSystemMatrix) * (**(this->mLeftSideVector)) +
                   mNonlinearFunctionResult);
    mLeftStep = mDirectLinearSolverVariableSystemMatrix->solve(temp);

    //	x2 = x1 + (x2-x1)
    **(this->mLeftSideVector) += mLeftStep;

    // 	*Update all CURRENT nonexplicit states that have a defining equation
    //	in the system matrix since they could not be expressed as a function
    //	of system inputs due to nonlinearity

    //	*Update all CURRENT states

    //	*Update system Jacobian with new system solution
    //	(including new nonexplicit states)

    //	*Calculate the actual system function result with
    //	the new solution vector

    for (auto comp : mMNANonlinearVariableComponents)
      comp->iterationUpdate(**(this->mLeftSideVector));

    //	Collect all System equation contributions from
    //	nonlinear SSN components

    Matrix ResetFunctionResult =
        mNonlinearFunctionResult; //use automatic adjustment of dimensions
    mNonlinearFunctionResult -=
        ResetFunctionResult; //to reset function result vector

    for (auto stamp : mNonlinearFunctionStamps)
      mNonlinearFunctionResult += *stamp;

    //	Check Convergence:
    //	Is the deviation of the system function result
    //	with the new solution vector
    //
    //	(mBaseSystemMatrix * **mLeftSideVector + mNonlinearSSNfunctionResult)
    //
    //	with mBaseSystemMatrix * **mLeftSideVector the contribution of
    //	static, linear components such as capacitors, inductors, resistors
    //	(Jacobian times vector is not only approximation but
    //	actual function of those components)
    //
    // 	and mNonlinearSSNfunctionResult the non-approximated,
    //	actual contribution of nonlinear SSN components
    //	using the new Solution vector
    //
    //	from the system excitation in the source vector
    //	mRightSideVector small enough? If yes: Newton-Raphson has converged.

    calculationError = this->mRightSideVector -
                       (this->mBaseSystemMatrix * (**(this->mLeftSideVector)) +
                        mNonlinearFunctionResult);

    isConverged = true;
    for (int i = 0; i < calculationError.rows(); i++) {
      calculationErrorElement = calculationError(i, 0);
      if (abs(calculationErrorElement) >= Epsilon) {
        isConverged = false;
        break;
      }
    }
    iterations++;
  } while (!isConverged && iterations < 100);
  /// TODO: split into separate task?
  //(dependent on x, updating all v attributes)
  for (UInt nodeIdx = 0; nodeIdx < this->mNumNetNodes; ++nodeIdx)
    this->mNodes[nodeIdx]->mnaUpdateVoltage(**(this->mLeftSideVector));
}

template <typename VarType>
void IterativeMnaSolverDirect<VarType>::initialize() {
  this->mFrequencyParallel = false;
  this->mSystemMatrixRecomputation = true;
  MnaSolver<VarType>::initialize();

  mMNANonlinearVariableComponents.clear();
  mMNANonlinearVariableComponents.shrink_to_fit();

  mNonlinearFunctionStamps.clear();
  mNonlinearFunctionStamps.shrink_to_fit();

  for (auto comp : MnaSolver<VarType>::mSystem.mComponents) {
    auto MNANonlinearComp =
        std::dynamic_pointer_cast<CPS::MNANonlinearVariableCompInterface>(comp);
    if (MNANonlinearComp)
      mMNANonlinearVariableComponents.push_back(MNANonlinearComp);
  }

  for (auto comp : this->mMNANonlinearVariableComponents) {
    const Matrix &stamp = comp->mNonlinearFunctionStamp.get()->get();
    if (stamp.size() != 0) {
      this->mNonlinearFunctionStamps.push_back(&stamp);
    }
  }

  mNonlinearFunctionResult = Matrix::Zero(this->mRightSideVector.rows(), 1);
  for (auto stamp : mNonlinearFunctionStamps)
    mNonlinearFunctionResult += *stamp;

  // Delta of iterative Solutions
  mLeftStep = Matrix::Zero(this->mLeftSideVector->get().rows(), 1);

  //If mRightSideVector deviates less than Epsilon per element
  //from the result of the system defining node equations, mesh equations
  //and auxhiliary equations (calculationError), the solution is good enough
  calculationError = Matrix::Zero(this->mRightSideVector.rows(), 1);
  calculationErrorElement = 0.;
}

template class DPsim::IterativeMnaSolverDirect<Real>;
template class DPsim::IterativeMnaSolverDirect<Complex>;