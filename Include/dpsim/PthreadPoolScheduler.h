/** Scheduler that dynamically distributes tasks to a pool of POSIX threads.
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

#include <cps/Logger.h>
#include <dpsim/Scheduler.h>

#include <villas/queue_signalled.h>

#include <vector>

namespace DPsim {
	class PthreadPoolScheduler : public Scheduler {
	public:
		PthreadPoolScheduler(size_t poolSize);
		~PthreadPoolScheduler();

		void createSchedule(const CPS::Task::List& tasks, const Edges& inEdges, const Edges& outEdges);
		void step(Real time, Int timeStepCount);

	private:
		static void* poolThreadFunction(void* data);

		std::vector<pthread_t> mThreads;

		CPS::Task::List mTasks;
		Edges mInEdges;
		Edges mOutEdges;

		struct queue_signalled mOutQueue;
		struct queue_signalled mDoneQueue;

		Real mTime;
		Int mTimeStepCount;
	};
};
