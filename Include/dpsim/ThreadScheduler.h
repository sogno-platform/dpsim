/** Abstract base class for schedulers using std::thread and the self-written
 * Barrier and Counter synchronization primitives.
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
	class ThreadScheduler : public Scheduler {
	public:
		ThreadScheduler(Int threads, String outMeasurementFile, Bool useConditionVariable);
		virtual ~ThreadScheduler();

		void step(Real time, Int timeStepCount);
		virtual void stop();

	protected:
		void startThreads();
		void scheduleTask(int thread, CPS::Task::Ptr task, const Edges& inEdges);

		Int mNumThreads;

	private:
		void doStep(Int scheduleIdx);
		static void threadFunction(ThreadScheduler* sched, Int idx);

		String mOutMeasurementFile;
		Barrier mStartBarrier;

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
}
