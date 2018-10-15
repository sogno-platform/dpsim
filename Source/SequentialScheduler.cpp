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

#include <deque>
#include <fstream>
#include <iostream>
#include <typeinfo>
#include <unordered_map>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/graph/topological_sort.hpp>

using namespace CPS;
using namespace DPsim;
using namespace boost;

// Simple topological sorting using Kahn's algorithm.
void SequentialScheduler::createSchedule(Task::List& tasks) {
	mSchedule.clear();

	// Create graph (list of out/in edges for each node) from attribute dependencies
	std::unordered_map<CPS::AttributeBase::Ptr, std::deque<int>> dependencies;
	for (size_t i = 0; i < tasks.size(); i++) {
		for (auto attr : tasks[i]->getAttributeDependencies()) {
			dependencies[attr].push_back(i);
		}
	}

	typedef adjacency_list<vecS, vecS, directedS> Graph;
	typedef graph_traits<Graph>::vertex_descriptor Vertex;
	typedef std::list<Vertex> Order;

	Graph g(tasks.size());

	for (size_t i = 0; i < tasks.size(); i++) {
		for (auto attr : tasks[i]->getModifiedAttributes()) {
			for (auto j : dependencies[attr]) {
				add_edge(i, j, g);
			}
		}
	}

	Order order;
	topological_sort(g, std::front_inserter(order));
	for (auto i : order)
		mSchedule.push_back(tasks[i]);

	std::vector<String> names(tasks.size());
	for (size_t i = 0; i < tasks.size(); i++)
		names[i] = tasks[i]->toString();

	std::ofstream outfile("dependencies.dot");
	write_graphviz(outfile, g, make_label_writer(names.data()));
}

void SequentialScheduler::step(Real time, Int timeStepCount) {
	for (auto it : mSchedule) {
		it->execute(time, timeStepCount);
	}
}
