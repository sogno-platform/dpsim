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

#pragma once

#include <dpsim/Scheduler.h>

#include <chrono>
#include <typeinfo>
#include <unordered_map>
#include <vector>

namespace DPsim {
	class SequentialScheduler : public Scheduler {
	public:
		SequentialScheduler(String outMeasurementFile = String()) : mOutMeasurementFile(outMeasurementFile) { }
		void createSchedule(const CPS::Task::List& tasks, const Edges& inEdges, const Edges& outEdges);
		void step(Real time, Int timeStepCount);
		void stop();

	private:
		CPS::Task::List mSchedule;

		std::unordered_map<size_t, std::vector<std::chrono::nanoseconds>> mMeasurements;
		std::vector<std::chrono::nanoseconds> mStepMeasurements;
		CPS::String mOutMeasurementFile;
	};
}
