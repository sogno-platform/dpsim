/** Example of shared memory interface
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

#include "DPsim.h"

using namespace DPsim;
using namespace DPsim::Components::DP;

int main(int argc, char *argv[])
{
	Component::List comps;

	struct shmem_conf conf;
	conf.samplelen = 4;
	conf.queuelen = 1024;
	conf.polling = true;

	if (argc < 2) {
		std::cerr << "Not enough arguments (either 0 or 1 for the test number)" << std::endl;
		std::exit(1);
	}

	String in, out;

	if (String(argv[1]) == "0") {
		in  = "/villas0-in";
		out = "/villas0-out";
	}
	else if (String(argv[1]) == "1") {
		in  = "/villas1-in";
		out = "/villas1-out";
	}

	ShmemInterface shmem(in, out, &conf);

	if (String(argv[1]) == "0") {
		auto evs = VoltageSource::make("v_t", 2, GND, Complex(0, 0));

		comps = {
			VoltageSourceNorton::make("v_s", 0, GND, Complex(10000, 0), 1),
			Inductor::make("l_1", 0, 1, 0.1),
			Resistor::make("r_1", 1, 2, 1),
			evs
		};

		shmem.registerControllableSource(evs, GND, 0);
		shmem.registerExportedCurrent(evs, GND, 0);
	}
	else if (String(argv[1]) == "1") {
		auto ecs = CurrentSource::make("v_s", 0, GND, Complex(0, 0));

		comps = {
			Resistor::make("r_2", 0, GND, 10),
			ecs
		};

		shmem.registerControllableSource(ecs, GND, 0);
		// TODO: check if this was refactored correctly
		shmem.registerExportedVoltage(ecs, 0, 1);
	}
	else {
		std::cerr << "invalid test number" << std::endl;
		std::exit(1);
	}

	String simName = "ShmemDistributed";
	Real timeStep = 0.001000;

	RealTimeSimulation sim(simName + argv[1], comps, 2.0*M_PI*50.0, timeStep, 20, Logger::Level::INFO);
	sim.addExternalInterface(&shmem);

	if (String(argv[1]) == "1") {
		auto comps2 = comps;

		comps2.pop_back();
		comps2.push_back(Resistor::make("r_2", 0, GND, 8));

		sim.addSystemTopology(comps2);
		sim.setSwitchTime(10, 1);
	}

	sim.run(true);

	return 0;
}
