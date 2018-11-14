/** Level scheduler using std::thread
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

#include <dpsim/ThreadLevelScheduler.h>

#include <algorithm>
#include <iostream>

using namespace CPS;
using namespace DPsim;

ThreadLevelScheduler::ThreadLevelScheduler(Int threads, String outMeasurementFile, String inMeasurementFile) :
	mNumThreads(threads), mOutMeasurementFile(outMeasurementFile), mInMeasurementFile(inMeasurementFile), mStartBarrier(threads) {
	if (threads < 1)
		throw SchedulingException();
	mSchedules.resize(threads);
}

void ThreadLevelScheduler::createSchedule(const Task::List& tasks, const Edges& inEdges, const Edges& outEdges) {
	Task::List ordered;
	std::vector<Task::List> levels;

	Scheduler::topologicalSort(tasks, inEdges, outEdges, ordered);
	Scheduler::levelSchedule(ordered, inEdges, outEdges, levels);

	if (!mOutMeasurementFile.empty())
		Scheduler::initMeasurements(tasks);

	if (!mInMeasurementFile.empty()) {
		std::unordered_map<String, TaskTime::rep> measurements;
		readMeasurements(mInMeasurementFile, measurements);
		for (size_t level = 0; level < levels.size(); level++) {
			// Distribute tasks such that the execution time is (approximately) minimized
			scheduleLevel(levels[level], measurements);
			// Insert BarrierTask for synchronization
			auto barrier = std::make_shared<BarrierTask>(mSchedules.size());
			for (int thread = 0; thread < mNumThreads; thread++)
				mSchedules[thread].push_back(barrier);
		}
	} else {
		for (size_t level = 0; level < levels.size(); level++) {
			// Distribute tasks of one level evenly between threads
			size_t nextThread = 0;
			for (auto task : levels[level]) {
				mSchedules[nextThread++].push_back(task);
				if (nextThread == mSchedules.size())
					nextThread = 0;
			}

			// Insert BarrierTask for synchronization
			auto barrier = std::make_shared<BarrierTask>(mSchedules.size());
			for (int thread = 0; thread < mNumThreads; thread++)
				mSchedules[thread].push_back(barrier);
		}
	}

	for (int i = 1; i < mNumThreads; i++) {
		std::thread thread(threadFunction, this, i);
		mThreads.push_back(std::move(thread));
	}
}

void ThreadLevelScheduler::scheduleLevel(const Task::List& tasks, const std::unordered_map<String, TaskTime::rep>& measurements) {
	Task::List tasksSorted = tasks;

	// Check that measurements map is complete
	for (auto task : tasks) {
		if (measurements.find(task->toString()) == measurements.end())
			throw SchedulingException();
	}

	// Sort tasks in descending execution time
	auto cmp = [&measurements](const Task::Ptr& p1, const Task::Ptr& p2) -> bool {
		return measurements.at(p1->toString()) > measurements.at(p2->toString());
	};
	std::sort(tasksSorted.begin(), tasksSorted.end(), cmp);

	// Greedy heuristic: schedule the tasks to the thread with the smallest current execution time
	std::vector<TaskTime::rep> totalTimes(mSchedules.size(), 0);
	for (auto task : tasksSorted) {
		auto minIt = std::min_element(totalTimes.begin(), totalTimes.end());
		size_t minIdx = minIt - totalTimes.begin();
		mSchedules[minIdx].push_back(task);
		totalTimes[minIdx] += measurements.at(task->toString());
	}
}

void ThreadLevelScheduler::step(Real time, Int timeStepCount) {
	mTime = time;
	mTimeStepCount = timeStepCount;
	mStartBarrier.wait();
	doStep(0);
}

void ThreadLevelScheduler::stop() {
	if (!mThreads.empty()) {
		mJoining = true;
		mStartBarrier.wait();
		for (size_t thread = 0; thread < mThreads.size(); thread++) {
			mThreads[thread].join();
		}
	}
	if (!mOutMeasurementFile.empty()) {
		writeMeasurements(mOutMeasurementFile);
	}
}

void ThreadLevelScheduler::threadFunction(ThreadLevelScheduler* sched, Int idx) {
	while (true) {
		sched->mStartBarrier.wait();
		if (sched->mJoining)
			return;

		sched->doStep(idx);
	}
}

void ThreadLevelScheduler::doStep(Int idx) {
	if (mOutMeasurementFile.empty()) {
		for (auto task : mSchedules[idx])
			task->execute(mTime, mTimeStepCount);
	} else {
		for (auto task : mSchedules[idx]) {
			auto start = std::chrono::steady_clock::now();
			task->execute(mTime, mTimeStepCount);
			auto end = std::chrono::steady_clock::now();
			// kind of ugly workaround since the barrier tasks are shared
			// between threads and we don't want to measure them anyway
			if (!std::dynamic_pointer_cast<BarrierTask>(task))
				updateMeasurement(task, end-start);
		}
	}
}
