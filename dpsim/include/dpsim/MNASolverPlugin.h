/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim/MNASolverDirect.h>
#include <dpsim/MNASolverDynInterface.h>
#include <dpsim/SolverParametersMNA.h>


namespace DPsim {

	template <typename VarType>
    class MnaSolverPlugin : public MnaSolverDirect<VarType>{
	protected:
		using Solver::mSLog;
		String mPluginName;
		struct dpsim_mna_plugin *mPlugin;
		void *mDlHandle;

		/// Initialize cuSparse-library
        void initialize() override;
		void recomputeSystemMatrix(Real time) override;
		void solve(Real time, Int timeStepCount) override;

	public:
		MnaSolverPlugin(String pluginName,
			String name,
			CPS::Domain domain = CPS::Domain::DP,
			std::shared_ptr<SolverParametersMNA> solverParams = SolverParametersMNA(),
			CPS::Logger::Level logLevel = CPS::Logger::Level::info);

		virtual ~MnaSolverPlugin();

		CPS::Task::List getTasks() override;

		class SolveTask : public CPS::Task {
	public:
			SolveTask(MnaSolverPlugin<VarType>& solver) :
				Task(solver.mName + ".Solve"), mSolver(solver) {

				for (auto it : solver.mMNAComponents) {
					if (it->getRightVector()->get().size() != 0)
						mAttributeDependencies.push_back(it->getRightVector());
				}
				for (auto node : solver.mNodes) {
					mModifiedAttributes.push_back(node->mVoltage);
				}
				mModifiedAttributes.push_back(solver.mLeftSideVector);
			}

			void execute(Real time, Int timeStepCount) { mSolver.solve(time, timeStepCount); }

		private:
			MnaSolverPlugin<VarType>& mSolver;
		};

		class LogTask : public CPS::Task {
		public:
			LogTask(MnaSolverPlugin<VarType>& solver) :
				Task(solver.mName + ".Log"), mSolver(solver) {
				mAttributeDependencies.push_back(solver.mLeftSideVector);
				mModifiedAttributes.push_back(Scheduler::external);
			}

			void execute(Real time, Int timeStepCount) { mSolver.log(time, timeStepCount); }

		private:
			MnaSolverPlugin<VarType>& mSolver;
		};
    };

}
