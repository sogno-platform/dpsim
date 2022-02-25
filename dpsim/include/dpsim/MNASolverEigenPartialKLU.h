
/* Copyright 2017-2020 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <iostream>
#include <vector>
#include <list>
#include <unordered_map>
#include <bitset>
#include <memory>

#include <dpsim/Config.h>
#include <dpsim/Solver.h>
#include <dpsim/DataLogger.h>
#include <dpsim-models/AttributeList.h>
#include <dpsim-models/Solver/MNASwitchInterface.h>
#include <dpsim-models/Solver/MNAVariableCompInterface.h>
#include <dpsim-models/SimSignalComp.h>
#include <dpsim-models/SimPowerComp.h>
#include <dpsim/MNASolver.h>
#include <dpsim/MNASolverEigenKLU.h>


namespace DPsim {

	/// Solver class using Modified Nodal Analysis (MNA).
	template <typename VarType>
	class MnaSolverEigenPartialKLU : public MnaSolverEigenKLU<VarType> {
		protected:
		/// Recomputes systems matrix
		void recomputeSystemMatrix(Real time);
		void stampVariableSystemMatrix();
		void solveWithSystemMatrixRecomputation(Real time, Int timeStepCount);

		public:
		MnaSolverEigenPartialKLU(String name,
			CPS::Domain domain = CPS::Domain::DP,
			CPS::Logger::Level logLevel = CPS::Logger::Level::info);

		/// Destructor
		virtual ~MnaSolverEigenPartialKLU() { };

		///
		class SolveTaskRecomp : public CPS::Task {
		public:
			~SolveTaskRecomp(){
				mSolver.logSolveTime();
				mSolver.logLUTime();
				mSolver.logRecomputationTime();
			}
			SolveTaskRecomp(MnaSolverEigenPartialKLU<VarType>& solver) :
				Task(solver.mName + ".Solve"), mSolver(solver) {

				for (auto it : solver.mMNAComponents) {
					if (it->template attribute<Matrix>("right_vector")->get().size() != 0)
						mAttributeDependencies.push_back(it->attribute("right_vector"));
				}
				for (auto it : solver.mMNAIntfVariableComps) {
					if (it->template attribute<Matrix>("right_vector")->get().size() != 0)
						mAttributeDependencies.push_back(it->attribute("right_vector"));
				}
				for (auto node : solver.mNodes) {
					mModifiedAttributes.push_back(node->attribute("v"));
				}
				mModifiedAttributes.push_back(solver.attribute("left_vector"));
			}

			void execute(Real time, Int timeStepCount) { 
				mSolver.solveWithSystemMatrixRecomputation(time, timeStepCount); 
				mSolver.log(time, timeStepCount);
				}

		private:
			MnaSolverEigenPartialKLU<VarType>& mSolver;
		};
	};
}