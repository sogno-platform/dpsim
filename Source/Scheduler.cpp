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
