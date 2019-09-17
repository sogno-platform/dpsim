/** Scheduler that distributes tasks to a pool of POSIX threads.
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

#include <cps/Logger.h>
#include <dpsim/PthreadPoolScheduler.h>

#include <iostream>

#include <villas/memory.h>
#include <villas/queue.h>

using namespace CPS;
using namespace DPsim;

PthreadPoolScheduler::PthreadPoolScheduler(size_t poolSize) {
	mThreads.resize(poolSize);
}

PthreadPoolScheduler::~PthreadPoolScheduler() {
	for (size_t i = 0; i < mThreads.size(); i++) {
		queue_signalled_push(&mOutQueue, nullptr);
	}
	for (size_t i = 0; i < mThreads.size(); i++) {
		pthread_join(mThreads[i], nullptr);
	}
}

void PthreadPoolScheduler::createSchedule(const Task::List& tasks, const Edges& inEdges, const Edges& outEdges) {
	// TODO: we're not actually creating the schedule here, but just copying
	// the dependency graph so we can do the schedule dynamically in every step
	mTasks = tasks;
	mInEdges = inEdges;
	mOutEdges = outEdges;

	// TODO Wastes memory, but guarantees that the writes always succeed.
	// Figure out a smarter way to do this.
	queue_signalled_init(&mOutQueue, tasks.size(), &memory_heap, QueueSignalledMode::POLLING);
	queue_signalled_init(&mDoneQueue, tasks.size(), &memory_heap, QueueSignalledMode::POLLING);

	for (size_t i = 0; i < mThreads.size(); i++) {
		if (pthread_create(&mThreads[i], NULL, poolThreadFunction, this))
			throw SchedulingException();
	}
}

void PthreadPoolScheduler::step(Real time, Int timeStepCount) {
	// these are read-only for each time step, so there's no harm in just having
	// them shared via the scheduler instance
	mTime = time;
	mTimeStepCount = timeStepCount;

	// copy incoming edges since we remove them during execution to mark a dependency as "done"
	Edges inEdgesCpy = mInEdges;

	// Basically topological sort, but instead of marking tasks as ready, send them to the worker pool,
	// and check if another task can be run everytime a task finishes.
	for (size_t i = 0; i < mTasks.size(); i++) {
		if (inEdgesCpy[mTasks[i]].empty()) {
			// TODO since mTasks already contains smart pointers, the additional
			// indirection is kind of unnecessary and defeats the smart pointers'
			// purpose (although it should at least be safe)
			if (queue_signalled_push(&mOutQueue, &mTasks[i]) != 1)
				throw SchedulingException();
			//std::cout << "scheduler: pushed " << mTasks[i]->toString() << std::endl;
		}
	}

	size_t done = 0;
	Task::Ptr t;
	void *p;
	while (done != mTasks.size()) {
		if (queue_signalled_pull(&mDoneQueue, &p) != 1)
			throw SchedulingException();
		t = *static_cast<Task::Ptr*>(p);
		//std::cout << "scheduler: " << t->toString() << " done" << std::endl;
		done++;
		for (size_t i = 0; i < mOutEdges[t].size(); i++) {
			Task::Ptr after = mOutEdges[t][i];
			for (auto edgeIt = inEdgesCpy[after].begin(); edgeIt != inEdgesCpy[after].end(); ++edgeIt) {
				if (*edgeIt == t) {
					inEdgesCpy[after].erase(edgeIt);
					break;
				}
			}
			if (inEdgesCpy[after].empty()) {
				// TODO: somewhat of a hack (see above), but should be safe
				// since mOutEdges is not modified during a step
				if (queue_signalled_push(&mOutQueue, &mOutEdges[t][i]) != 1)
					throw SchedulingException();
				//std::cout << "scheduler: pushed " << after->toString() << std::endl;
			}
		}
	}
	//std::cout << "scheduler: timestep done" << std::endl;
}

void* PthreadPoolScheduler::poolThreadFunction(void* data) {
	PthreadPoolScheduler* sched = static_cast<PthreadPoolScheduler*>(data);
	Task::Ptr t;
	void* p;

	while (1) {
		if (queue_signalled_pull(&sched->mOutQueue, &p) != 1)
			throw SchedulingException();
		if (!p)
			break;

		t = *static_cast<Task::Ptr*>(p);
		//std::cout << "worker: pulled " << t->toString() << std::endl;
		t->execute(sched->mTime, sched->mTimeStepCount);
		if (queue_signalled_push(&sched->mDoneQueue, p) != 1)
			throw SchedulingException();
		//std::cout << "worker: done with " << t->toString() << std::endl;
	}
	//std::cout << "worker exiting" << std::endl;
	return nullptr;
}
