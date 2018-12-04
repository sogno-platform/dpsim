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

#include <fstream>
#include <iostream>
#include <unordered_map>
#include <unordered_set>

using namespace CPS;
using namespace DPsim;

CPS::AttributeBase::Ptr Scheduler::external;

void Scheduler::initMeasurements(const Task::List& tasks) {
	// Fill map here already since it's not protected by a mutex
	for (auto task : tasks) {
		mMeasurements[task] = std::vector<TaskTime>();
	}
}

void Scheduler::updateMeasurement(Task::Ptr ptr, TaskTime time) {
	mMeasurements[ptr].push_back(time);
}

void Scheduler::writeMeasurements(String filename) {
	std::ofstream os(filename);
	std::unordered_map<String, TaskTime> averages;
	for (auto& pair : mMeasurements) {
		TaskTime avg(0), tot(0);

		for (TaskTime time : pair.second) {
			tot += time;
		}
		if (!pair.second.empty())
			avg = tot / pair.second.size();
		averages[pair.first->toString()] = avg;
	}
	// TODO think of nicer output format
	for (auto pair : averages) {
		os << pair.first << " " << pair.second.count() << std::endl;
	}
	os.close();
}

void Scheduler::readMeasurements(String filename, std::unordered_map<String, TaskTime::rep>& measurements) {
	std::ifstream fs(filename);
	if (!fs.good())
		throw SchedulingException();

	String name;
	TaskTime::rep duration;
	while (fs.good()) {
		fs >> name >> duration;
		measurements[name] = duration;
	}
}

void Scheduler::resolveDeps(Task::List& tasks, Edges& inEdges, Edges& outEdges) {
	// Create graph (list of out/in edges for each node) from attribute dependencies
	tasks.push_back(mRoot);
	std::unordered_map<AttributeBase::Ptr, std::deque<Task::Ptr>> dependencies;
	std::unordered_set<AttributeBase::Ptr> prevStepDependencies;
	for (auto task : tasks) {
		for (auto attr : task->getAttributeDependencies()) {
			dependencies[AttributeBase::getRefAttribute(attr)].push_back(task);
		}
		for (auto attr : task->getPrevStepDependencies()) {
			prevStepDependencies.insert(attr);
		}
	}

	for (auto from : tasks) {
		for (auto attr : from->getModifiedAttributes()) {
			for (auto to : dependencies[attr]) {
				outEdges[from].push_back(to);
				inEdges[to].push_back(from);
			}
			if (prevStepDependencies.count(attr)) {
				outEdges[from].push_back(mRoot);
				inEdges[mRoot].push_back(from);
			}
		}
	}
}

void Scheduler::topologicalSort(const Task::List& tasks, const Edges& inEdges, const Edges& outEdges, Task::List& sortedTasks) {
	sortedTasks.clear();

	// make copies of the edge lists because we modify them (and it makes
	// things easier since empty lists are automatically created)
	Edges inEdgesCpy = inEdges, outEdgesCpy = outEdges;

	// do a breadth-first search backwards from the root node first to filter
	// out unnecessary nodes
	std::deque<Task::Ptr> q;
	std::unordered_set<Task::Ptr> visited;

	q.push_back(mRoot);
	while (!q.empty()) {
		auto t = q.front();
		q.pop_front();
		if (visited.count(t))
			continue;

		visited.insert(t);
		for (auto dep : inEdgesCpy[t]) {
			if (!visited.count(dep)) {
				q.push_back(dep);
			}
		}
	}

	for (auto t : tasks) {
		if (inEdgesCpy[t].empty()) {
			q.push_back(t);
		}
	}
	// keep list of tasks without incoming edges;
	// iteratively remove such tasks from the graph and put them into the schedule
	while (!q.empty()) {
		Task::Ptr t = q.front();
		q.pop_front();
		if (!visited.count(t)) {
			// don't put unneeded tasks in the schedule, but process them as usual
			// so the cycle check still works
			//std::cout << "Dropping " << t->toString() << std::endl;
		} else if (t != mRoot) {
			sortedTasks.push_back(t);
		}

		for (auto after : outEdgesCpy[t]) {
			for (auto edgeIt = inEdgesCpy[after].begin(); edgeIt != inEdgesCpy[after].end(); ++edgeIt) {
				if (*edgeIt == t) {
					inEdgesCpy[after].erase(edgeIt);
					break;
				}
			}
			if (inEdgesCpy[after].empty()) {
				q.push_back(after);
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
		if (mCount.fetch_add(1, std::memory_order_acquire) == mLimit-1) {
			mCount.store(0, std::memory_order_release);
		} else {
			while (mCount.load(std::memory_order_acquire) != 0);
		}
	}
}

void Barrier::signal() {
	if (mUseCondition) {
		std::unique_lock<std::mutex> lk(mMutex);
		mCount++;
		if (mCount == mLimit) {
			mCount = 0;
			lk.unlock();
			mCondition.notify_all();
		}
	} else {
		if (mCount.fetch_add(1, std::memory_order_acquire) == mLimit-1) {
			mCount.store(0, std::memory_order_release);
		}
	}
}

void BarrierTask::addBarrier(Barrier* b) {
	mBarriers.push_back(b);
}

void BarrierTask::execute(Real time, Int timeStepCount) {
	if (mBarriers.size() == 1) {
		mBarriers[0]->wait();
	} else {
		for (size_t i = 0; i < mBarriers.size()-1; i++) {
			mBarriers[i]->signal();
		}
		mBarriers[mBarriers.size()-1]->wait();
	}
}
