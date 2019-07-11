/** List scheduler using std::thread
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

#include <dpsim/ThreadListScheduler.h>

#include <queue>

using namespace CPS;
using namespace DPsim;

ThreadListScheduler::ThreadListScheduler(Int threads, String outMeasurementFile, String inMeasurementFile, Bool useConditionVariables) :
	ThreadScheduler(threads, outMeasurementFile, useConditionVariables), mInMeasurementFile(inMeasurementFile) {
}

void ThreadListScheduler::createSchedule(const Task::List& tasks, const Edges& inEdges, const Edges& outEdges) {
	Task::List ordered;

	Scheduler::topologicalSort(tasks, inEdges, outEdges, ordered);
	Scheduler::initMeasurements(ordered);

	std::unordered_map<Task::Ptr, int64_t> priorities;
	std::unordered_map<String, TaskTime::rep> measurements;
	if (!mInMeasurementFile.empty()) {
		readMeasurements(mInMeasurementFile, measurements);

		// Check that measurements map is complete
		for (auto task : ordered) {
			if (measurements.find(task->toString()) == measurements.end())
				throw SchedulingException();
		}
	} else {
		// Insert constant cost for each task (HLFNET)
		for (auto task : ordered) {
			measurements[task->toString()] = 1;
		}
	}

	// HLFET
	for (auto it = ordered.rbegin(); it != ordered.rend(); ++it) {
		auto task = *it;
		int64_t maxLevel = 0;
		if (outEdges.find(task) != outEdges.end()) {
			for (auto dep : outEdges.at(task)) {
				if (priorities[dep] > maxLevel) {
					maxLevel = priorities[dep];
				}
			}
		}
		priorities[task] = measurements.at(task->toString()) + maxLevel;
	}

	auto cmp = [&priorities](const Task::Ptr& p1, const Task::Ptr& p2) -> bool {
		return priorities[p1] < priorities[p2];
	};
	std::priority_queue<Task::Ptr, std::deque<Task::Ptr>, decltype(cmp)> queue(cmp);
	for (auto task : ordered) {
		if (inEdges.find(task) == inEdges.end() || inEdges.at(task).empty()) {
			queue.push(task);
		} else {
			break;
		}
	}

	std::vector<TaskTime::rep> totalTimes(mNumThreads, 0);
	Edges inEdgesCpy = inEdges;
	while (!queue.empty()) {
		auto task = queue.top();
		queue.pop();

		auto minIt = std::min_element(totalTimes.begin(), totalTimes.end());
		size_t minIdx = minIt - totalTimes.begin();
		scheduleTask(minIdx, task);
		totalTimes[minIdx] += measurements.at(task->toString());

		if (outEdges.find(task) != outEdges.end()) {
			for (auto after : outEdges.at(task)) {
				for (auto edgeIt = inEdgesCpy[after].begin(); edgeIt != inEdgesCpy[after].end(); ++edgeIt) {
					if (*edgeIt == task) {
						inEdgesCpy[after].erase(edgeIt);
						break;
					}
				}
				if (inEdgesCpy[after].empty() && std::find(ordered.begin(), ordered.end(), after) != ordered.end()) {
					queue.push(after);
				}
			}
		}
	}

	ThreadScheduler::finishSchedule(inEdges);
}
