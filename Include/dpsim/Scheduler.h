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

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <unordered_map>

namespace DPsim {
	// TODO extend / subclass
	class SchedulingException {};

	class Scheduler {
	public:
		typedef std::unordered_map<CPS::Task::Ptr, std::deque<CPS::Task::Ptr>> Edges;
		typedef std::chrono::steady_clock::duration TaskTime;

		Scheduler() : mRoot(std::make_shared<Root>()) { }
		virtual ~Scheduler() { }

		// Interface functions

		// Creates the schedule for the given dependency graph
		virtual void createSchedule(const CPS::Task::List& tasks, const Edges& inEdges, const Edges& outEdges) = 0;
		// Performs a single simulation step
		virtual void step(Real time, Int timeStepCount) = 0;
		// Called on simulation stop to reliably clean up e.g. running helper threads
		virtual void stop() {}

		// Helper function that resolves the task-attribute dependencies to task-task dependencies
		// and inserts a root task
		void resolveDeps(CPS::Task::List& tasks, Edges& inEdges, Edges& outEdges);

		// Special attribute that can be returned in the modified attributes of a task
		// to mark that this task has external side-effects (like logging / interfacing)
		// and thus has to be executed even though it doesn't modify any attribute.
		// TODO is it really fine to use nullptr or should we create a special sentinel attribute?
		static CPS::AttributeBase::Ptr external;

	protected:
		// Simple topological sort, filtering out tasks that do not need to be executed.
		void topologicalSort(const CPS::Task::List& tasks, const Edges& inEdges, const Edges& outEdges, CPS::Task::List& sortedTasks);
		// Separate topologically sorted list of tasks into levels which can be
		// executed in parallel
		static void levelSchedule(const CPS::Task::List& tasks, const Edges& inEdges, const Edges& outEdges, std::vector<CPS::Task::List>& levels);

		void initMeasurements(const CPS::Task::List& tasks);
		// Not thread-safe for multiple calls with same task, but should only
		// be called once for each task in each step anyway
		void updateMeasurement(CPS::Task::Ptr task, TaskTime time);
		void writeMeasurements(CPS::String filename);
		void readMeasurements(CPS::String filename, std::unordered_map<CPS::String, TaskTime::rep>& measurements);

		CPS::Task::Ptr mRoot;

	private:
		// TODO more sophisticated measurement method might be necessary for
		// longer simulations (risk of high memory requirements and integer
		// overflow)
		std::unordered_map<CPS::Task::Ptr, std::vector<TaskTime>> mMeasurements;

		class Root : public CPS::Task {
		public:
			Root() : Task("Root") {
				mAttributeDependencies.push_back(external);
			}

			void execute(Real time, Int timeStepCount) {
				throw SchedulingException();
			}
		};
	};

	class Barrier {
	public:
		Barrier() = delete;
		Barrier(Int limit, Bool useCondition = false) :
			mLimit(limit), mCount(0), mUseCondition(useCondition) {}

		/// Blocks until |limit| calls have been made, at which point all threads
		/// return. Can in general be reused, but some other synchronization method
		/// (like a second barrier) has to be used to ensure that all calls of the
		/// first use have returned before the first call to the second use.
		void wait();

	private:
		Int mLimit;
		std::atomic<Int> mCount;
		Bool mUseCondition;

		std::mutex mMutex;
		std::condition_variable mCondition;
	};

	class BarrierTask : public CPS::Task {
	public:
		BarrierTask() = delete;
		BarrierTask(Int limit, Bool useConditionVariable = false) : mBarrier(limit, useConditionVariable) {}

		void execute(Real time, Int timeStepCount);

	private:
		Barrier mBarrier;
	};
}
