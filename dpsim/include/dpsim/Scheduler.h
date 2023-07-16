/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim-models/Task.h>

#include <dpsim/Definitions.h>
#include <dpsim-models/Logger.h>

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
		/// Edges describe the dependency from the first task to a list of other tasks
		/// or the other way around.
		typedef std::unordered_map<CPS::Task::Ptr, std::deque<CPS::Task::Ptr>> Edges;
		/// Time measurement for the task execution
		typedef std::chrono::steady_clock::duration TaskTime;

		///
		Scheduler(CPS::Logger::Level logLevel = CPS::Logger::Level::info, CPS::Logger::Level cliLevel = CPS::Logger::Level::off) :
			mRoot(std::make_shared<Root>()),
			mSLog(CPS::Logger::get(CPS::Logger::LoggerType::SIMULATION, "Scheduler", logLevel, cliLevel)) {
		}
		///
		virtual ~Scheduler() { }

		// #### Interface functions ####

		/// Creates the schedule for the given dependency graph
		virtual void createSchedule(const CPS::Task::List& tasks, const Edges& inEdges, const Edges& outEdges) = 0;
		/// Performs a single simulation step
		virtual void step(Real time, Int timeStepCount) = 0;
		/// Called on simulation stop to reliably clean up e.g. running helper threads
		virtual void stop() {}

		/// Helper function that resolves the task-attribute dependencies to task-task dependencies
		/// and inserts a root task
		void resolveDeps(CPS::Task::List& tasks, Edges& inEdges, Edges& outEdges);

		// Special attribute that can be returned in the modified attributes of a task
		// to mark that this task has external side-effects (like logging / interfacing)
		// and thus has to be executed even though it doesn't modify any attribute.
		// TODO is it really fine to use nullptr or should we create a special sentinel attribute?
		static CPS::AttributeBase::Ptr external;

		TaskTime getAveragedMeasurement(CPS::Task::Ptr task) {
			return getAveragedMeasurement(task.get());
		}

		/// Root task that has a dependency on the external attribute
		/// which means that it should not be removed from the task graph
		class Root : public CPS::Task {
		public:
			Root() : Task("Root") {
				mAttributeDependencies.push_back(external);
			}

			void execute(Real time, Int timeStepCount) {
				throw SchedulingException();
			}
		};

	protected:
		/// Simple topological sort, filtering out tasks that do not need to be executed.
		void topologicalSort(const CPS::Task::List& tasks, const Edges& inEdges, const Edges& outEdges, CPS::Task::List& sortedTasks);
		/// Separate topologically sorted list of tasks into levels which can be
		/// executed in parallel
		static void levelSchedule(const CPS::Task::List& tasks, const Edges& inEdges, const Edges& outEdges, std::vector<CPS::Task::List>& levels);

		void initMeasurements(const CPS::Task::List& tasks);
		/// Not thread-safe for multiple calls with same task, but should only
		/// be called once for each task in each step anyway
		void updateMeasurement(CPS::Task* task, TaskTime time);
		/// Write measurement data to file
		void writeMeasurements(CPS::String filename);
		/// Read measurement data from file to use it for the scheduling
		void readMeasurements(CPS::String filename, std::unordered_map<CPS::String, TaskTime::rep>& measurements);
		///
		TaskTime getAveragedMeasurement(CPS::Task* task);

		///
		CPS::Task::Ptr mRoot;
		/// Logger
		CPS::Logger::Log mSLog;
	private:
		// TODO more sophisticated measurement method might be necessary for
		// longer simulations (risk of high memory requirements and integer
		// overflow)
		std::unordered_map<CPS::Task*, std::vector<TaskTime>> mMeasurements;
	};

	/// A barrier is used to synchronize threads. Threads running into the barrier
	/// have to wait until the barrier state is released when a defined number
	/// of threads reaches the barrier.
	class Barrier {
	public:
		/// Constructor without parameters is forbidden.
		Barrier() = delete;
		/// Limit sets the number of threads that need to reach the barrier
		/// to release it.
		Barrier(Int limit, Bool useCondition = false) :
			mLimit(limit), mCount(0), mGeneration(0), mUseCondition(useCondition) {}

		/// Blocks until |limit| calls have been made, at which point all threads
		/// return. Provides synchronization, i.e. all writes from before this call
		/// are visible in all threads after this call.
		void wait() {
			if (mUseCondition) {
				std::unique_lock<std::mutex> lk(mMutex);
				Int gen = mGeneration;
				mCount++;
				if (mCount == mLimit) {
					mCount = 0;
					mGeneration++;
					lk.unlock();
					mCondition.notify_all();
				} else {
					// necessary because of spurious wakeups
					while (gen == mGeneration)
						mCondition.wait(lk);
				}
			} else {
				Int gen = mGeneration.load(std::memory_order_acquire);
				// We need at least one release from each thread to ensure that
				// every write from before the wait() is visible in every other thread,
				// and the fetch needs to be an acquire anyway, so use acq_rel instead of acquire.
				// (This generates the same code on x86.)
				if (mCount.fetch_add(1, std::memory_order_acq_rel) == mLimit-1) {
					mCount.store(0, std::memory_order_relaxed);
					mGeneration.fetch_add(1, std::memory_order_release);
				} else {
					while (mGeneration.load(std::memory_order_acquire) == gen);
				}
			}
		}

		/// Increases the barrier counter like wait does, but does not wait for it to
		/// reach the limit (so this does not provide any synchronization with
		/// other threads). Can be used to eliminate unnecessary waits if
		/// multiple barriers are used in sequence.
		void signal() {
			if (mUseCondition) {
				std::unique_lock<std::mutex> lk(mMutex);
				mCount++;
				if (mCount == mLimit) {
					mCount = 0;
					mGeneration++;
					lk.unlock();
					mCondition.notify_all();
				}
			} else {
				// No release here, as this call does not provide any synchronization anyway.
				if (mCount.fetch_add(1, std::memory_order_acquire) == mLimit-1) {
					mCount.store(0, std::memory_order_relaxed);
					mGeneration.fetch_add(1, std::memory_order_release);
				}
			}
		}

	private:
		/// Barrier limit which has to be reached before the barrier is released.
		Int mLimit;
		/// Barrier counter which is tested against limit
		std::atomic<Int> mCount;
		/// Allows multiple use of the barrier
		std::atomic<Int> mGeneration;
		Bool mUseCondition;

		std::mutex mMutex;
		std::condition_variable mCondition;
	};

	class BarrierTask : public CPS::Task {
	public:
		typedef std::shared_ptr<BarrierTask> Ptr;
		void addBarrier(Barrier* b);
		void execute(Real time, Int timeStepCount);

	private:
		std::vector<Barrier*> mBarriers;
	};

	class Counter {
	public:
		Counter() : mValue(0) {}

		void inc() {
			mValue.fetch_add(1, std::memory_order_release);
		}

		void wait(Int value) {
			while (mValue.load(std::memory_order_acquire) != value);
		}

	private:
		std::atomic<Int> mValue;
	};
}
