/** Level scheduler using OpenMP
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

#include <dpsim/OpenMPLevelScheduler.h>
#include <omp.h>

#include <iostream>

using namespace CPS;
using namespace DPsim;

void OpenMPLevelScheduler::createSchedule(const Task::List& tasks, const Edges& inEdges, const Edges& outEdges) {
	Task::List ordered;

	Scheduler::topologicalSort(tasks, inEdges, outEdges, ordered);
	Scheduler::levelSchedule(ordered, inEdges, outEdges, mLevels);
}

void OpenMPLevelScheduler::step(Real time, Int timeStepCount) {
	size_t i = 0;

	for (size_t level = 0; level < mLevels.size(); level++) {
		#pragma omp parallel shared(time,timeStepCount,level) private(i)
		{
			#pragma omp for schedule(static)
			for (i = 0; i < mLevels[level].size(); i++) {
				mLevels[level][i]->execute(time, timeStepCount);
			}
		}
	}
}

void OpenMPLevelScheduler::getMeasurements() {
}
