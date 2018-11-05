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

ThreadLevelScheduler::ThreadLevelScheduler(Int threads) :
	mNumThreads(threads), mStartBarrier(threads+1), mEndBarrier(threads+1) {
	mSchedules.resize(threads);
}

ThreadLevelScheduler::~ThreadLevelScheduler() {
	if (!mThreads.empty()) {
		mJoining = true;
		mStartBarrier.wait();
		for (size_t thread = 0; thread < mThreads.size(); thread++) {
			mThreads[thread].join();
		}
	}
}

void ThreadLevelScheduler::createSchedule(const Task::List& tasks, const Edges& inEdges, const Edges& outEdges) {
	Task::List ordered;
	std::vector<Task::List> levels;

	Scheduler::topologicalSort(tasks, inEdges, outEdges, ordered);
	Scheduler::levelSchedule(ordered, inEdges, outEdges, levels);

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

void ThreadLevelScheduler::threadFunction(ThreadLevelScheduler* sched, Int idx) {
	while (true) {
		sched->mStartBarrier.wait();
		if (sched->mJoining)
			return;

		for (auto task : sched->mSchedules[idx])
			task->execute(sched->mTime, sched->mTimeStepCount);

		sched->mEndBarrier.wait();
	}
}

void ThreadLevelScheduler::getMeasurements() {
}
