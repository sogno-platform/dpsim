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

#pragma once

#include <dpsim/Scheduler.h>

#include <thread>
#include <vector>

namespace DPsim {
	class ThreadLevelScheduler : public Scheduler {
	public:
		ThreadLevelScheduler(Int threads = 1, String outMeasurementFile = String(), String inMeasurementFile = String(), Bool useConditionVariables = false, Bool sortTaskTypes = false);
		~ThreadLevelScheduler();

		void createSchedule(const CPS::Task::List& tasks, const Edges& inEdges, const Edges& outEdges);
		void step(Real time, Int timeStepCount);
		void stop();
		void getMeasurements();

	private:
		void scheduleTask(int thread, CPS::Task::Ptr task, const Edges& inEdges);
		void scheduleLevel(const CPS::Task::List& tasks, const std::unordered_map<String, TaskTime::rep>& measurements, const Edges& inEdges);
		void sortTasksByType(CPS::Task::List::iterator begin, CPS::Task::List::iterator end);

		void doStep(Int scheduleIdx);
		static void threadFunction(ThreadLevelScheduler* sched, Int idx);

		Int mNumThreads;
		String mOutMeasurementFile;
		String mInMeasurementFile;
		Bool mUseConditionVariable;
		Barrier mStartBarrier;
		Bool mSortTaskTypes;

		std::vector<std::thread> mThreads;

		std::map<CPS::Task::Ptr, Counter*> mCounters;
		struct ScheduleEntry {
			CPS::Task::Ptr task;
			Counter* endCounter;
			std::vector<Counter*> reqCounters;
		};
		std::vector<std::vector<ScheduleEntry>> mSchedules;

		Bool mJoining = false;
		Real mTime = 0;
		Int mTimeStepCount = 0;
	};
};
