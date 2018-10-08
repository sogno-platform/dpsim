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

#include <deque>
#include <iostream>
#include <typeinfo>
#include <unordered_map>

// Simple topological sorting using Kahn's algorithm.
void SequentialScheduler::createSchedule(Task::List& tasks) {
	mSchedule.clear();

	// Create graph (list of out/in edges for each node) from attribute dependencies
	std::unordered_map<CPS::AttributeBase::Ptr, std::deque<Task::Ptr>> dependencies;
	for (auto task : tasks) {
		for (auto attr : task->getAttributeDependencies()) {
			dependencies[attr].push_back(task);
		}
	}

	std::unordered_map<Task::Ptr, std::deque<Task::Ptr>> out_edges;
	std::unordered_map<Task::Ptr, std::deque<Task::Ptr>> in_edges;
	for (auto from : tasks) {
		for (auto attr : from->getModifiedAttributes()) {
			for (auto to : dependencies[attr]) {
				out_edges[from].push_back(to);
				in_edges[to].push_back(from);
			}
		}
	}

	std::deque<Task::Ptr> ready;
	for (auto task : tasks) {
		if (in_edges[task].empty()) {
			ready.push_back(task);
		}
	}

	// keep list of tasks without incoming edges;
	// iteratively remove such tasks from the graph and put them into the schedule
	while (!ready.empty()) {
		Task::Ptr t = ready.front();
		ready.pop_front();
		mSchedule.push_back(t);

		for (auto after : out_edges[t]) {
			for (auto edgeIt = in_edges[after].begin(); edgeIt != in_edges[after].end(); ++edgeIt) {
				if (*edgeIt == t) {
					in_edges[after].erase(edgeIt);
					break;
				}
			}
			if (in_edges[after].empty()) {
				ready.push_back(after);
			}
		}
		out_edges.erase(t);
	}

	// sanity check: all edges should have been removed, otherwise
	// the graph had a cycle
	for (auto t : tasks) {
		if (!out_edges[t].empty() || !in_edges[t].empty())
			throw SchedulingException();
	}

	std::cout << "Schedule:" << std::endl;
	for (auto it : mSchedule) {
		std::cout << typeid(*it).name() << std::endl;
	}
}

void SequentialScheduler::step() {
	for (auto it : mSchedule) {
		it->execute();
	}
}
