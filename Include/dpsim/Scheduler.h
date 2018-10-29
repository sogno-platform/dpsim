/** Task scheduler base class
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

#include <cps/Task.h>

#include <dpsim/Definitions.h>

#include <deque>

namespace DPsim {
	class Scheduler {
	public:
		typedef std::unordered_map<CPS::Task::Ptr, std::deque<CPS::Task::Ptr>> Edges;

		virtual void createSchedule(const CPS::Task::List& tasks, const Edges& inEdges, const Edges& outEdges) = 0;
		virtual void step(Real time, Int timeStepCount) = 0;
	};

	// TODO extend / subclass
	class SchedulingException {};
}
