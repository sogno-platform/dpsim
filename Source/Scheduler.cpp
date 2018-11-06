/** Common scheduler methods
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

#include <dpsim/Scheduler.h>

using namespace CPS;
using namespace DPsim;

void Scheduler::resolveDeps(const Task::List& tasks, Edges& inEdges, Edges& outEdges) {
	// Create graph (list of out/in edges for each node) from attribute dependencies
	std::unordered_map<CPS::AttributeBase::Ptr, std::deque<Task::Ptr>> dependencies;
	for (auto task : tasks) {
		for (auto attr : task->getAttributeDependencies()) {
			dependencies[AttributeBase::getRefAttribute(attr)].push_back(task);
		}
	}

	for (auto from : tasks) {
		for (auto attr : from->getModifiedAttributes()) {
			for (auto to : dependencies[attr]) {
				outEdges[from].push_back(to);
				inEdges[to].push_back(from);
			}
		}
	}
}

void Scheduler::topologicalSort(const Task::List& tasks, const Edges& inEdges, const Edges& outEdges, Task::List& sortedTasks) {
	sortedTasks.clear();

	// make copies of the edge lists because we modify them
	Edges inEdgesCpy = inEdges, outEdgesCpy = outEdges;

	std::deque<Task::Ptr> ready;
	for (auto task : tasks) {
		if (inEdgesCpy[task].empty()) {
			ready.push_back(task);
		}
	}

	// keep list of tasks without incoming edges;
	// iteratively remove such tasks from the graph and put them into the schedule
	while (!ready.empty()) {
		Task::Ptr t = ready.front();
		ready.pop_front();
		sortedTasks.push_back(t);

		for (auto after : outEdgesCpy[t]) {
			for (auto edgeIt = inEdgesCpy[after].begin(); edgeIt != inEdgesCpy[after].end(); ++edgeIt) {
				if (*edgeIt == t) {
					inEdgesCpy[after].erase(edgeIt);
					break;
				}
			}
			if (inEdgesCpy[after].empty()) {
				ready.push_back(after);
			}
		}
		outEdgesCpy.erase(t);
	}

	// sanity check: all edges should have been removed, otherwise
	// the graph had a cycle
	for (auto t : tasks) {
		if (!outEdgesCpy[t].empty() || !inEdgesCpy[t].empty())
			throw SchedulingException();
	}
}

void Scheduler::levelSchedule(const Task::List& tasks, const Edges& inEdges, const Edges& outEdges, std::vector<Task::List>& levels) {
	std::unordered_map<Task::Ptr, int> time;

	for (auto task : tasks) {
		if (inEdges.find(task) == inEdges.end() || inEdges.at(task).empty()) {
			time[task] = 0;
		} else {
			int maxdist = 0;
			for (auto before : inEdges.at(task)) {
				if (time[before] > maxdist)
					maxdist = time[before];
			}
			time[task] = maxdist + 1;
		}
	}

	levels.clear();
	levels.resize(time[tasks.back()] + 1);
	for (auto task : tasks) {
		levels[time[task]].push_back(task);
	}
}

void Barrier::wait() {
	if (mUseCondition) {
		std::unique_lock<std::mutex> lk(mMutex);
		mCount++;
		if (mCount == mLimit) {
			mCount = 0;
			lk.unlock();
			mCondition.notify_all();
		} else {
			// necessary because of spurious wakeups
			while (mCount != mLimit && mCount != 0)
				mCondition.wait(lk);
		}
	} else {
		if (mCount.fetch_add(1) == mLimit-1) {
			mCount = 0;
		} else {
			while (mCount != 0);
		}
	}
}

void BarrierTask::execute(Real time, Int timeStepCount) {
	mBarrier.wait();
}
