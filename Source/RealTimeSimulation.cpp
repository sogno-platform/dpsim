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

#include <dpsim/RealTimeSimulation.h>

using namespace CPS;
using namespace DPsim;

RealTimeSimulation::RealTimeSimulation(String name, SystemTopology system, Real timeStep, Real finalTime,
		Domain domain, Solver::Type type, Logger::Level logLevel, Bool steadyStateInit)
	: Simulation(name, system, timeStep, finalTime, domain, type, logLevel, steadyStateInit),
	mTimeStep(timeStep),
	mTimer()
{
	addAttribute<Real>("time_step", &mTimeStep, Flags::read);
	addAttribute<Int >("overruns", nullptr, [=](){ return mTimer.overruns(); }, Flags::read);
}

void RealTimeSimulation::run(const Timer::StartClock::duration &startIn)
{
	run(Timer::StartClock::now() + startIn);
}

void RealTimeSimulation::run(const Timer::StartClock::time_point &startAt)
{
	auto startAtDur = startAt.time_since_epoch();
	auto startAtNSecs = std::chrono::duration_cast<std::chrono::nanoseconds>(startAtDur);

	sync();

	mLog.info() << "Starting simulation at " << startAt << " (delta_T = " << startAt - Timer::StartClock::now() << " seconds)" << std::endl;

	mTimer.setStartTime(startAt);
	mTimer.setInterval(mTimeStep);
	mTimer.start();

	// main loop
	do {
		step();
		mTimer.sleep();

		if (mTimer.ticks() == 1)
			mLog.info() << "Simulation started." << std::endl;
	} while (mTime < mFinalTime);

	mLog.info() << "Simulation finished." << std::endl;

	mTimer.stop();
}
