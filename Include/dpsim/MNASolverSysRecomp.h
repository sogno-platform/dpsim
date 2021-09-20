/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim/MNASolverEigenSparse.h>

namespace DPsim {
	/// Solver class using Modified Nodal Analysis (MNA).
	template <typename VarType>
	class MnaSolverSysRecomp : public MnaSolverEigenSparse<VarType> {
	protected:
		/// Initialization of system matrices and source vector
		virtual void initializeSystem() override;
		///
		virtual void solve(Real time, Int timeStepCount) override;

		// #### Dynamic matrix recomputation ####
		/// Flag that initiates recomputation of system matrix
		Bool mUpdateSysMatrix;
		/// Recomputes systems matrix
		void updateSystemMatrix(Real time);
		/// Collects the status of variable MNA elements to decide if system matrix has to be recomputed
		void updateVariableCompStatus();
		/// Initialization of system matrices and source vector
		void initializeSystemWithDynamicMatrix();

	public:
		///
		MnaSolverSysRecomp(String name,
			CPS::Domain domain = CPS::Domain::DP,
			CPS::Logger::Level logLevel = CPS::Logger::Level::info);
		///
		virtual ~MnaSolverSysRecomp() { };
		///
		virtual CPS::Task::List getTasks() override;

		// #### MNA Solver Tasks ####
		///
		class SolveTask : public CPS::Task {
		public:
			SolveTask(MnaSolverSysRecomp<VarType>& solver) :
				Task(solver.mName + ".Solve"), mSolver(solver) {

				for (auto it : solver.mMNAComponents) {
					if (it->template attribute<Matrix>("right_vector")->get().size() != 0)
						mAttributeDependencies.push_back(it->attribute("right_vector"));
				}
				for (auto node : solver.mNodes) {
					mModifiedAttributes.push_back(node->attribute("v"));
				}
				mModifiedAttributes.push_back(solver.attribute("left_vector"));
			}

			void execute(Real time, Int timeStepCount) { mSolver.solve(time, timeStepCount); }

		private:
			MnaSolverSysRecomp<VarType>& mSolver;
		};

		class LogTask : public CPS::Task {
		public:
			LogTask(MnaSolverSysRecomp<VarType>& solver) :
				Task(solver.mName + ".Log"), mSolver(solver) {
				mAttributeDependencies.push_back(solver.attribute("left_vector"));
				mModifiedAttributes.push_back(Scheduler::external);
			}

			void execute(Real time, Int timeStepCount) { mSolver.log(time, timeStepCount); }

		private:
			MnaSolverSysRecomp<VarType>& mSolver;
		};
	};
}
