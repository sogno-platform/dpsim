/** Tests for real-time timer
 *
 * @author Steffen Vogel <stvogel@eonerc.rwth-aachen.de>
 * @copyright 2018, Institute for Automation of Complex Power Systems, EONERC
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

#include <dpsim/Timer.h>
#include <dpsim/Utils.h>

using namespace DPsim;

int main(int argc, char *argv[]) {
	Timer t;

	CommandLineArgs args(argc, argv);

	t.setStartTime(args.startTime);
	t.setInterval(args.timeStep);

	if (args.startTime == Timer::StartTimePoint())
		args.startTime = Timer::StartClock::now();

	std::cout << "Start clock at: " << args.startTime << "(in " << args.startTime - Timer::StartClock::now() << ")" << std::endl;
	std::cout << "Timer interval: " << t.interval() << std::endl;

	t.start();

	while (t.ticks() * args.timeStep < args.duration) {
		t.sleep();

		std::cout << ".";
		std::cout.flush();
	}

	t.stop();
}
