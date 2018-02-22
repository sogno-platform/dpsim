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

#include "RealTimeSimulation.h"
#include "ShmemInterface.h"
#include "Components.h"

using namespace DPsim;
using namespace DPsim::Components::DP;

int main(int argc, char *argv[])
{
	// Testing the interface with a simple circuit,
	// but the load is simulated in a different instance.
	// Values are exchanged using the ideal transformator model: an ideal
	// current source on the supply side and an ideal voltage source on the
	// supply side, whose values are received from the respective other circuit.
	// Here, the two instances directly communicate with each other without using
	// VILLASnode in between.

	Component::List comps;

	struct shmem_conf conf;
	conf.samplelen = 4;
	conf.queuelen = 1024;
	conf.polling = false;

	if (argc < 2) {
		std::cerr << "not enough arguments (either 0 or 1 for the test number)" << std::endl;
		std::exit(1);
	}

	String in, out;

	if (String(argv[1]) == "0") {
		in  = "/dpsim10";
		out = "/dpsim01";
	}
	else if (String(argv[1]) == "1") {
		in  = "/dpsim01";
		out = "/dpsim10";
	}

	ShmemInterface shmem(in, out, &conf);

	if (String(argv[1]) == "0") {
		auto evs = VoltageSource::make("v_t", 1, GND, Complex(0, 0));

		comps = {
			VoltageSourceNorton::make("v_s", 0, GND, Complex(10000, 0), 1),
			Inductor::make("l_1", 0, 1, 1e-3),
			evs
		};

		shmem.registerControllableSource(evs, GND, 0);
		shmem.registerExportedCurrent(evs, GND, 0);
	}
	else if (String(argv[1]) == "1") {
		auto ecs = CurrentSource::make("v_s", 0, GND, Complex(0, 0));

		comps = {
			Resistor::make("r_2", 0, GND, 1),
			ecs
		};

		shmem.registerControllableSource(ecs, GND, 0);
		shmem.registerExportedVoltage(0, GND, 0, 1);
	}
	else {
		std::cerr << "invalid test number" << std::endl;
		std::exit(1);
	}

	Real timeStep = 0.000150;
	RealTimeSimulation sim("ShmemDistributedDirect", comps, 2.0*M_PI*50.0, timeStep, 1);
	sim.addExternalInterface(&shmem);
	sim.run(false);

	return 0;
}
