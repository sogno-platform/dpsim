/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim/MNASolverEigenSparse.h>
#include <cps/Solver/SSNNonlinearCompInterface.h>


namespace DPsim{

class SSNSolver: public MnaSolverEigenSparse<Real>{
	public:
		SSNSolver(String name, CPS::SystemTopology& system, Real timeStep, CPS::Logger::Level logLevel);
		
		virtual void solveWithSystemMatrixRecomputation(Real time, Int timeStepCount) override;

		virtual void initialize() override;

		virtual std::shared_ptr<CPS::Task> createSolveTaskRecomp() override;

		virtual void recomputeSystemMatrix(Real time);
	protected:
		Matrix mNonlinearSSNfunctionResult = Matrix::Zero(1,1);

		CPS::SSNNonlinearCompInterface::List mSSNNonlinearComponents;

		/// List of all nonlinear SSN component function contributions
		std::vector<const Matrix*> mNonlinearSSNfunctionStamps;

		// Delta of iterative Solutions
		Matrix leftStep;
		//Number of Iterations per step
		unsigned iterations = 0;
		//If mRightSideVector deviates less than Epsilon per element from the result of the system defining node equations, mesh equations
		//and auxhiliary equations (calculationError), the solution is good enough 
		bool isConverged = true;
		Real Epsilon = 0.00001;
		Matrix calculationError;
		Real calculationErrorElement;
};
}






