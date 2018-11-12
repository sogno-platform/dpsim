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
		// TODO: maybe extra parameter for whether the start and/or inter-level barriers use spinlock or CV
		ThreadLevelScheduler(Int threads = 1, String outMeasurementFile = String(), String inMeasurementFile = String());

		void createSchedule(const CPS::Task::List& tasks, const Edges& inEdges, const Edges& outEdges);
		void step(Real time, Int timeStepCount);
		void stop();
		void getMeasurements();

	private:
		void scheduleLevel(const CPS::Task::List& tasks, const std::unordered_map<String, TaskTime::rep>& measurements);
		static void threadFunction(ThreadLevelScheduler* sched, Int idx);
		void doStep(Int scheduleIdx);

		Int mNumThreads;
		String mOutMeasurementFile;
		String mInMeasurementFile;
		std::vector<std::thread> mThreads;
		std::vector<CPS::Task::List> mSchedules;
		Barrier mStartBarrier;

		Bool mJoining = false;
		Real mTime = 0;
		Int mTimeStepCount = 0;
	};
};
