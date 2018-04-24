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
using namespace CPS::Components::DP;

int main(int argc, char *argv[]) {
	// Testing the interface with a simple circuit,
	// but the load is simulated in a different instance.
	// Values are exchanged using the ideal transformator model: an ideal
	// current source on the supply side and an ideal voltage source on the
	// supply side, whose values are received from the respective other circuit.
	// Here, the two instances directly communicate with each other without using
	// VILLASnode in between.

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

	Real timeStep = 0.000150;

	if (String(argv[1]) == "0") {
		auto evs = VoltageSource::make("v_intf", GND, 1, Complex(0, 0));

		Component<Complex>::List comps = {
			VoltageSourceNorton::make("v_s", GND, 0, Complex(1000, 0), 1),
			Resistor::make("r_0_1", 0, 1, 1),
			Resistor::make("r_1_gnd", 1, GND, 1),
			evs
		};

		shmem.registerControllableAttribute(evs->findAttribute<Complex>("VoltageRef"), 0, 1);
		shmem.registerExportedAttribute(evs->findAttribute<Complex>("CompCurrent"), 0, 1);

		SystemTopology system(50, comps);		
		Simulation sim("ShmemDistributedDirect_1", system, timeStep, 1);
		sim.addInterface(&shmem);

		sim.run(false);
	}
	else if (String(argv[1]) == "1") {
		auto ecs = CurrentSource::make("i_intf", GND, 0, Complex(0, 0));
		//auto ecs_switch = CurrentSource::make("i_switch", GND, 1, Complex(0, 0));

		Component<Complex>::List comps = {
			Resistor::make("r_gnd_0", GND, 0, 1),
			Resistor::make("r_0_1", 0, 1, 1),
			ecs
			//ecs_switch
		};

		shmem.registerControllableAttribute(ecs->findAttribute<Complex>("CurrentRef"), 0, 1);
		shmem.registerExportedAttribute(ecs->findAttribute<Complex>("CompVoltage"), 0, 1);
		//shmem.registerControllableAttribute(ecs_switch->findAttribute('CurrentRef'), 2, 3);

		SystemTopology system(50, comps);
		Simulation sim("ShmemDistributedDirect_2", system, timeStep, 1);
		sim.addInterface(&shmem);

		sim.run(false);
	}
	else {
		std::cerr << "invalid test number" << std::endl;
		std::exit(1);
	}

	return 0;
}
