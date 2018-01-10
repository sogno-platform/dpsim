/** Simulation
 *
 * @author Markus Mirz <mmirz@eonerc.rwth-aachen.de>
 * @copyright 2017, Institute for Automation of Complex Power Systems, EONERC
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

#include <signal.h>

#include "Config.h"
#include "Simulation.h"

namespace DPsim {

	class TimerExpiredException {
	};

	class RealTimeSimulation : public Simulation {

	protected:
		uint64_t mTimerCount = 0;

	public:
		/* Possible methods to achieve execution in real time. */
		enum Method {
			Exceptions, // use a normal timer and throw an exception in the signal handler if the timestep wasn't completed yet
			TimerFD,    // read on a timerfd after every step
		};

		/* Perform the main simulation loop in real time.
		 *
		 * @param rtMethod The method with which the realtime execution is achieved.
		 * @param startSynch If true, the simulation waits for the first external value before starting the timing.
		 */
		void run(Method method, bool startSynch);
		static void alarmHandler(int, siginfo_t*, void*);
	};

}
