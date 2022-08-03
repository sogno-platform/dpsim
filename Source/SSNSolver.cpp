/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim/SSNSolver.h>
#include <dpsim/SequentialScheduler.h>
#include <math.h>
#include <Eigen/Dense>

using namespace DPsim;
using namespace CPS;

namespace DPsim {



SSNSolver::SSNSolver(String name, CPS::SystemTopology& system, Real timeStep, CPS::Logger::Level logLevel): MnaSolverEigenSparse(name, CPS::Domain::EMT, logLevel)
{
	mSystem = system;
	mTimeStep = timeStep;

	mSSNNonlinearComponents.clear();
	mSSNNonlinearComponents.shrink_to_fit();

	mNonlinearSSNfunctionStamps.clear();
	mNonlinearSSNfunctionStamps.shrink_to_fit();
}



void SSNSolver::initialize()
{
	mSystemMatrixRecomputation = true;
	MnaSolver::initialize();

	for (auto comp : mSystem.mComponents) 
	{
		auto SSNNonlinearComp = std::dynamic_pointer_cast<CPS::SSNNonlinearCompInterface>(comp);
		if (SSNNonlinearComp) mSSNNonlinearComponents.push_back(SSNNonlinearComp);
	}

	for(auto comp : mSSNNonlinearComponents)
	{
		const Matrix& stamp = comp->template attribute<Matrix>("SSN_Function_Result")->get();
		if (stamp.size() != 0) {
			mNonlinearSSNfunctionStamps.push_back(&stamp);
		}
	}

	mNonlinearSSNfunctionResult = Matrix::Zero(mRightSideVector.rows(), 1);
	for (auto stamp : mNonlinearSSNfunctionStamps) mNonlinearSSNfunctionResult += *stamp;

	// Delta of iterative Solutions
	leftStep = Matrix::Zero(mLeftSideVector->get().rows(), 1);

	//If mRightSideVector deviates less than Epsilon per element from the result of the system defining node equations, mesh equations
	//and auxhiliary equations (calculationError), the solution is good enough 
	calculationError = Matrix::Zero(mRightSideVector.rows(), 1);
	calculationErrorElement = 0.;

	stampVariableSystemMatrix();
}


void SSNSolver::recomputeSystemMatrix(Real time) {
	// Start from base matrix
	mVariableSystemMatrix = mBaseSystemMatrix;

	// Now stamp switches into matrix
	for (auto sw : mSwitches)
		sw->mnaApplySystemMatrixStamp(mVariableSystemMatrix);

	// Now stamp variable elements into matrix
	for (auto comp : mMNAIntfVariableComps)
		comp->mnaApplySystemMatrixStamp(mVariableSystemMatrix);

	Matrix temp = mVariableSystemMatrix;
	CPS::SparseMatrixRow sparse;
	sparse = temp.sparseView();
	mLuFactorizationVariableSystemMatrix.analyzePattern(sparse);
	mLuFactorizationVariableSystemMatrix.factorize(sparse);

	++mNumRecomputations;
}

void SSNSolver::solveWithSystemMatrixRecomputation(Real time, Int timeStepCount)
{		
		// Reset source vector
		mRightSideVector.setZero();
		// Add together the right side vector (computed by the components'
		// pre-step tasks)
		for (auto stamp : mRightVectorStamps)
			mRightSideVector += *stamp;

		//Number of Iterations per step
		unsigned iterations = 0;
	do
	{
		// Get switch and variable comp status and update system matrix and lu factorization accordingly
		if (hasVariableComponentChanged())
		{
			recomputeSystemMatrix(time);
		}

		// 	Calculate new delta solution vector: systemMatrix*leftStep = mRightSideVector-mNonlinearSSNfunctionResult
		//	Corresponds to Newton-Raphson:
		//
		//	f(x2) =~ f(x1) + Df(x1)*(x2-x1) 
		//	
		//	f(x) is the function vector containing the system describing equations WITHOUT injections and SSN history terms
		//	These are the right side source vector and are set equal to the function:
		//	f(x2) = mRightSideVector =~ f(x1) + Df(x1)*(x2-x1)
		// 	Subtracting the past function value leaves: systemMatrix*leftStep = mRightSideVector-(mBaseSystemMatrix*(**mLeftSideVector)+ mNonlinearSSNfunctionResult)
		//	mRightSideVector is stamped by mna-prestep tasks and only updated each step, not iteration.

		leftStep = mLuFactorizationVariableSystemMatrix.solve(mRightSideVector-(mBaseSystemMatrix*(**mLeftSideVector)+ mNonlinearSSNfunctionResult));

		//	x2 = x1 + (x2-x1)
		**mLeftSideVector += leftStep;

		//	*Update all CURRENT dq-voltages
		// 	*Update all CURRENT nonexplicit states that have a defining equation in the system matrix since they could not be expressed as a function of system inputs due to nonlinearity
		//	*Update all CURRENT states
		//	*Update system Jacobian with new system solution (including new nonexplicit states)
		//	*Calculate the actual system function result with the new solution vector

		for(auto comp : mSSNNonlinearComponents) comp->ssnUpdate(**mLeftSideVector);

		//	Collect all System equation contributions from nonlinear SSN components
		mNonlinearSSNfunctionResult -= mNonlinearSSNfunctionResult;
		for (auto stamp : mNonlinearSSNfunctionStamps)
			mNonlinearSSNfunctionResult += *stamp;


		//	Check Convergence:
		//	Is the deviation of the system function result with the new solution vector
		//		
		//	(mBaseSystemMatrix * **mLeftSideVector + mNonlinearSSNfunctionResult)
		//
		//	with mBaseSystemMatrix * **mLeftSideVector the contribution of static, linear components such as capacitors, inductors, resistors 
		//	(Jacobian times vector is not only approximation but actual function of those components)
		//
		// 	and mNonlinearSSNfunctionResult the non-approximated, actual contribution of nonlinear SSN components using the new Solution vector
		//
		//	from the system excitation in the source vector mRightSideVector small enough? If yes: Newton-Raphson has converged.

		calculationError = mRightSideVector-(mBaseSystemMatrix*(**mLeftSideVector)+ mNonlinearSSNfunctionResult);									
		
		isConverged = true;
		for(int i = 0; i < calculationError.rows(); i++)
		{
			calculationErrorElement = calculationError(i, 0);
				//std::cout << calculationErrorElement << std::endl;
			if(abs(calculationErrorElement) >= Epsilon)
			{
				isConverged = false;
				break;
			}
		}
		iterations++;
	} 	while (!isConverged && iterations < 1000);
	if(iterations > 1) std::cout << iterations << std::endl;
	/// TODO: split into separate task? (dependent on x, updating all v attributes)
	for (UInt nodeIdx = 0; nodeIdx < mNumNetNodes; ++nodeIdx)
			mNodes[nodeIdx]->mnaUpdateVoltage(**mLeftSideVector);
}

std::shared_ptr<CPS::Task> SSNSolver::createSolveTaskRecomp()
{
	return std::make_shared<MnaSolverEigenSparse<Real>::SolveTaskRecomp>(*this);
}
}