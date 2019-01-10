/** Level scheduler using std::thread
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

#include <dpsim/ThreadLevelScheduler.h>

#include <algorithm>
#include <iostream>
#include <typeinfo>

using namespace CPS;
using namespace DPsim;

ThreadLevelScheduler::ThreadLevelScheduler(Int threads, String outMeasurementFile, String inMeasurementFile, Bool useConditionVariable, Bool sortTaskTypes) :
	ThreadScheduler(threads, outMeasurementFile, useConditionVariable), mInMeasurementFile(inMeasurementFile), mSortTaskTypes(sortTaskTypes) {
}

void ThreadLevelScheduler::createSchedule(const Task::List& tasks, const Edges& inEdges, const Edges& outEdges) {
	Task::List ordered;
	std::vector<Task::List> levels;

	Scheduler::topologicalSort(tasks, inEdges, outEdges, ordered);
	Scheduler::initMeasurements(ordered);

	Scheduler::levelSchedule(ordered, inEdges, outEdges, levels);

	if (!mInMeasurementFile.empty()) {
		std::unordered_map<String, TaskTime::rep> measurements;
		readMeasurements(mInMeasurementFile, measurements);
		for (size_t level = 0; level < levels.size(); level++) {
			// Distribute tasks such that the execution time is (approximately) minimized
			scheduleLevel(levels[level], measurements, inEdges);
		}
	} else {
		for (size_t level = 0; level < levels.size(); level++) {
			if (mSortTaskTypes)
				sortTasksByType(levels[level].begin(), levels[level].end());
			// Distribute tasks of one level evenly between threads
			for (int thread = 0; thread < mNumThreads; thread++) {
				int start = levels[level].size() * thread / mNumThreads;
				int end = levels[level].size() * (thread + 1) / mNumThreads;
				for (int idx = start; idx != end; idx++)
					scheduleTask(thread, levels[level][idx]);
			}
		}
	}

	ThreadScheduler::finishSchedule(inEdges);
}

void ThreadLevelScheduler::sortTasksByType(Task::List::iterator begin, CPS::Task::List::iterator end) {
	auto cmp = [](const Task::Ptr& p1, const Task::Ptr& p2) -> bool {
		// TODO: according to the standard, the ordering may change between invocations
		return typeid(*p1).before(typeid(*p2));
	};
	std::sort(begin, end, cmp);
}

void ThreadLevelScheduler::scheduleLevel(const Task::List& tasks, const std::unordered_map<String, TaskTime::rep>& measurements, const Edges& inEdges) {
	Task::List tasksSorted = tasks;

	// Check that measurements map is complete
	for (auto task : tasks) {
		if (measurements.find(task->toString()) == measurements.end())
			throw SchedulingException();
	}

	if (mSortTaskTypes) {
		TaskTime::rep totalTime = 0;
		for (auto task : tasks) {
			totalTime += measurements.at(task->toString());
		}

		TaskTime::rep avgTime = totalTime / mNumThreads;

		// Sort the tasks by type and then push them to the threads in order
		// while aiming for approximately equal execution time.
		// Should work well enough for a large enough number of tasks
		sortTasksByType(tasksSorted.begin(), tasksSorted.end());
		size_t task = 0;
		for (int thread = 0; thread < mNumThreads; thread++) {
			TaskTime::rep curTime = 0;
			while (curTime < avgTime && task < tasksSorted.size()) {
				scheduleTask(thread, tasksSorted[task]);
				curTime += measurements.at(tasksSorted[task]->toString());
				task++;
			}
		}
		// All tasks should be distributed, but just to be sure, put the remaining
		// ones to the last thread
		for (; task < tasksSorted.size(); task++)
			scheduleTask(mNumThreads-1, tasksSorted[task]);
	}
	else {
		// Sort tasks in descending execution time
		auto cmp = [&measurements](const Task::Ptr& p1, const Task::Ptr& p2) -> bool {
			return measurements.at(p1->toString()) > measurements.at(p2->toString());
		};
		std::sort(tasksSorted.begin(), tasksSorted.end(), cmp);

		// Greedy heuristic: schedule the tasks to the thread with the smallest current execution time
		std::vector<TaskTime::rep> totalTimes(mNumThreads, 0);
		for (auto task : tasksSorted) {
			auto minIt = std::min_element(totalTimes.begin(), totalTimes.end());
			size_t minIdx = minIt - totalTimes.begin();
			scheduleTask(minIdx, task);
			totalTimes[minIdx] += measurements.at(task->toString());
		}
	}
}
