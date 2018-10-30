/** Simple non-parallel task scheduler
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


#include <dpsim/SequentialScheduler.h>

using namespace CPS;
using namespace DPsim;

#include <chrono>
#include <deque>
#include <iostream>
#include <set>
#include <typeinfo>
#include <unordered_map>

// Simple topological sorting using Kahn's algorithm.
void SequentialScheduler::createSchedule(const Task::List& tasks, const Edges& inEdges, const Edges& outEdges) {
	mSchedule.clear();

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
		mSchedule.push_back(t);

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

void SequentialScheduler::getMeasurements() {
	std::set<size_t> done;
	for (auto it : mSchedule) {
		if (done.find(typeid(*it).hash_code()) != done.end())
			continue;
		done.insert(typeid(*it).hash_code());
		std::vector<std::chrono::nanoseconds> meas = mMeasurements[typeid(*it).hash_code()];
		std::chrono::nanoseconds tot(0);
		for (size_t i = 0; i < meas.size(); i++)
			tot += meas[i];
		auto avg = tot / meas.size();
		std::cout << typeid(*it).name() << " " << avg.count() << std::endl;
	}
}

void SequentialScheduler::step(Real time, Int timeStepCount) {
	for (auto it : mSchedule) {
		auto start = std::chrono::system_clock::now();
		it->execute(time, timeStepCount);
		auto end = std::chrono::system_clock::now();
		mMeasurements[typeid(*it).hash_code()].push_back(end-start);
	}
}
