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

void SequentialScheduler::createSchedule(const Task::List& tasks, const Edges& inEdges, const Edges& outEdges) {
	if (mOutMeasurementFile.size() != 0)
		Scheduler::initMeasurements(tasks);
	Scheduler::topologicalSort(tasks, inEdges, outEdges, mSchedule);
}

void SequentialScheduler::step(Real time, Int timeStepCount) {
	if (mOutMeasurementFile.size() != 0) {
		for (auto it : mSchedule) {
			auto start = std::chrono::steady_clock::now();
			it->execute(time, timeStepCount);
			auto end = std::chrono::steady_clock::now();
			updateMeasurement(it, end-start);
		}
	} else {
		for (auto it : mSchedule) {
			it->execute(time, timeStepCount);
		}
	}
}

void SequentialScheduler::stop() {
	if (mOutMeasurementFile.size() != 0)
		writeMeasurements(mOutMeasurementFile);
}
