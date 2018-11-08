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

OpenMPLevelScheduler::OpenMPLevelScheduler(Int threads, String outMeasurementFile) : mOutMeasurementFile(outMeasurementFile) {
	if (threads >= 0)
		mNumThreads = threads;
	else
		mNumThreads = omp_get_num_threads();
}

void OpenMPLevelScheduler::createSchedule(const Task::List& tasks, const Edges& inEdges, const Edges& outEdges) {
	Task::List ordered;

	Scheduler::topologicalSort(tasks, inEdges, outEdges, ordered);
	Scheduler::levelSchedule(ordered, inEdges, outEdges, mLevels);

	if (!mOutMeasurementFile.empty())
		Scheduler::initMeasurements(tasks);
}

void OpenMPLevelScheduler::step(Real time, Int timeStepCount) {
	size_t i = 0;
	std::chrono::steady_clock::time_point start, end;

	if (!mOutMeasurementFile.empty()) {
		for (size_t level = 0; level < mLevels.size(); level++) {
			#pragma omp parallel shared(time,timeStepCount,level) private(i, start, end) num_threads(mNumThreads)
			{
				#pragma omp for schedule(static)
				for (i = 0; i < mLevels[level].size(); i++) {
					start = std::chrono::steady_clock::now();
					mLevels[level][i]->execute(time, timeStepCount);
					end = std::chrono::steady_clock::now();
					updateMeasurement(mLevels[level][i], end-start);
				}
			}
		}
	} else {
		for (size_t level = 0; level < mLevels.size(); level++) {
			#pragma omp parallel shared(time,timeStepCount,level) private(i) num_threads(mNumThreads)
			{
				#pragma omp for schedule(static)
				for (i = 0; i < mLevels[level].size(); i++) {
					mLevels[level][i]->execute(time, timeStepCount);
				}
			}
		}
	}
}

void OpenMPLevelScheduler::stop() {
	if (!mOutMeasurementFile.empty()) {
		writeMeasurements(mOutMeasurementFile);
	}
}
