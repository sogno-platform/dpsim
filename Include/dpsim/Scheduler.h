/** Task scheduler base class and utilities / static methods
 *
 * @author Georg Reinke <georg.reinke@rwth-aachen.de>
 * @copyright 2017-2018, Institute for Automation of Complex Power Systems, EONERC
 *
 * DPsim
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *********************************************************************************/

#pragma once

#include <cps/Task.h>

#include <dpsim/Definitions.h>

#include <condition_variable>
#include <deque>
#include <mutex>

namespace DPsim {
	class Scheduler {
	public:
		typedef std::unordered_map<CPS::Task::Ptr, std::deque<CPS::Task::Ptr>> Edges;

		virtual void createSchedule(const CPS::Task::List& tasks, const Edges& inEdges, const Edges& outEdges) = 0;
		// TODO think of better interface for this
		virtual void getMeasurements() = 0;
		virtual void step(Real time, Int timeStepCount) = 0;

		// Helper function that resolves the task-attribute dependencies to task-task dependencies.
		static void resolveDeps(const CPS::Task::List& tasks, Edges& inEdges, Edges& outEdges);
		// Simple topological sorting using Kahn's algorithm.
		static void topologicalSort(const CPS::Task::List& tasks, const Edges& inEdges, const Edges& outEdges, CPS::Task::List& sortedTasks);
		// Separate topologically sorted list of tasks into levels which can be
		// executed in parallel
		static void levelSchedule(const CPS::Task::List& tasks, const Edges& inEdges, const Edges& outEdges, std::vector<CPS::Task::List>& levels);
	};

	// TODO extend / subclass
	class SchedulingException {};

	class Barrier {
	public:
		Barrier() = delete;
		Barrier(Int limit);

		/// Blocks until |limit| calls have been made, at which point all threads
		/// return. Can in general be reused, but some other synchronization method
		/// (like a second barrier) has to be used to ensure that all calls of the
		/// first use have returned before the first call to the second use.
		void wait();

	private:
		Int mLimit, mCount;
		std::mutex mMutex;
		std::condition_variable mCondition;
	};

	class BarrierTask : public CPS::Task {
	public:
		BarrierTask() = delete;
		BarrierTask(Int limit) : mBarrier(limit) {}

		void execute(Real time, Int timeStepCount);

	private:
		Barrier mBarrier;
	};
}
