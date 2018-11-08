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

#include <iostream>

using namespace CPS;
using namespace DPsim;

ThreadLevelScheduler::ThreadLevelScheduler(Int threads, String outMeasurementFile) :
	mNumThreads(threads), mOutMeasurementFile(outMeasurementFile), mStartBarrier(threads+1), mEndBarrier(threads+1) {
	mSchedules.resize(threads);
}

void ThreadLevelScheduler::createSchedule(const Task::List& tasks, const Edges& inEdges, const Edges& outEdges) {
	Task::List ordered;
	std::vector<Task::List> levels;

	Scheduler::topologicalSort(tasks, inEdges, outEdges, ordered);
	Scheduler::levelSchedule(ordered, inEdges, outEdges, levels);

	if (!mOutMeasurementFile.empty())
		Scheduler::initMeasurements(tasks);

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

	for (int i = 0; i < mNumThreads; i++) {
		std::thread thread(threadFunction, this, i);
		mThreads.push_back(std::move(thread));
	}
}

void ThreadLevelScheduler::step(Real time, Int timeStepCount) {
	mTime = time;
	mTimeStepCount = timeStepCount;
	mStartBarrier.wait();
	mEndBarrier.wait();
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

		if (sched->mOutMeasurementFile.empty()) {
			for (auto task : sched->mSchedules[idx])
				task->execute(sched->mTime, sched->mTimeStepCount);
		} else {
			for (auto task : sched->mSchedules[idx]) {
				auto start = std::chrono::steady_clock::now();
				task->execute(sched->mTime, sched->mTimeStepCount);
				auto end = std::chrono::steady_clock::now();
				// kind of ugly workaround since the barrier tasks are shared
				// between threads and we don't want to measure them anyway
				if (!std::dynamic_pointer_cast<BarrierTask>(task))
					sched->updateMeasurement(task, end-start);
			}
		}

		sched->mEndBarrier.wait();
	}
}
