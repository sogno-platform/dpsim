/** Simulation
 *
 * @author Steffen Vogel <stvogel@eonerc.rwth-aachen.de>
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

#include <signal.h>

#include <chrono>

#include <dpsim/Config.h>
#include <dpsim/Simulation.h>
#include <dpsim/Timer.h>

namespace DPsim {

	class RealTimeSimulation : public Simulation {

	protected:
		Real mTimeStep;
		Timer mTimer;

	public:
		/// Creates system matrix according to a given System topology
		RealTimeSimulation(String name, CPS::SystemTopology system,
			Real timeStep, Real finalTime,
			CPS::Domain domain = CPS::Domain::DP,
			Solver::Type solverType = Solver::Type::MNA,
			CPS::Logger::Level logLevel = CPS::Logger::Level::INFO,
			Bool steadyStateInit = false);

		/** Perform the main simulation loop in real time.
		 *
		 * @param startSynch If true, the simulation waits for the first external value before starting the timing.
		 */
		void run(const Timer::StartClock::duration &startIn = std::chrono::seconds(1));

		void run(const Timer::StartTimePoint &startAt);
	};
}

