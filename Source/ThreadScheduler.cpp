/** Abstract base class for schedulers using std::thread and the self-written
 * Barrier and Counter synchronization primitives.
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

#include <dpsim/ThreadScheduler.h>

using namespace CPS;
using namespace DPsim;

ThreadScheduler::ThreadScheduler(Int threads, String outMeasurementFile, Bool useConditionVariable) :
	mNumThreads(threads), mOutMeasurementFile(outMeasurementFile), mStartBarrier(threads, useConditionVariable) {
	if (threads < 1)
		throw SchedulingException();
	mSchedules.resize(threads);
}

ThreadScheduler::~ThreadScheduler() {
	for (auto pair : mCounters)
		delete pair.second;
}

void ThreadScheduler::scheduleTask(int thread, CPS::Task::Ptr task, const Edges& inEdges) {
	std::vector<Counter*> reqCounters;
	if (inEdges.find(task) != inEdges.end()) {
		for (auto req : inEdges.at(task)) {
			if (mCounters.find(req) == mCounters.end())
				mCounters[req] = new Counter();
			reqCounters.push_back(mCounters[req]);
		}
	}
	if (mCounters.find(task) == mCounters.end())
		mCounters[task] = new Counter();
	mSchedules[thread].push_back({task, mCounters[task], reqCounters});
}

void ThreadScheduler::startThreads() {
	for (int i = 1; i < mNumThreads; i++) {
		mThreads.emplace_back(threadFunction, this, i);
	}
}

void ThreadScheduler::step(Real time, Int timeStepCount) {
	mTime = time;
	mTimeStepCount = timeStepCount;
	mStartBarrier.wait();
	doStep(0);
	// since we don't have a final BarrierTask, wait for all threads to finish
	// their last task explicitly
	for (int thread = 1; thread < mNumThreads; thread++) {
		if (mSchedules[thread].size() != 0)
			mSchedules[thread].back().endCounter->wait(mTimeStepCount+1);
	}
}

void ThreadScheduler::stop() {
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

void ThreadScheduler::threadFunction(ThreadScheduler* sched, Int idx) {
	while (true) {
		sched->mStartBarrier.wait();
		if (sched->mJoining)
			return;

		sched->doStep(idx);
	}
}

void ThreadScheduler::doStep(Int idx) {
	if (mOutMeasurementFile.empty()) {
		for (auto& entry : mSchedules[idx]) {
			for (Counter* counter : entry.reqCounters)
				counter->wait(mTimeStepCount+1);
			entry.task->execute(mTime, mTimeStepCount);
			entry.endCounter->inc();
		}
	} else {
		for (auto& entry : mSchedules[idx]) {
			for (Counter* counter : entry.reqCounters)
				counter->wait(mTimeStepCount+1);
			auto start = std::chrono::steady_clock::now();
			entry.task->execute(mTime, mTimeStepCount);
			auto end = std::chrono::steady_clock::now();
			updateMeasurement(entry.task, end-start);
			entry.endCounter->inc();
		}
	}
}
