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

int main(int argc, char *argv[])
{
	ComponentBase::List comps, comps2;
	Node::List nodes;

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
		// Nodes
		auto n1 = Node::make("n1");
		auto n2 = Node::make("n2");
		auto n3 = Node::make("n3");

		// Components
		auto evs = VoltageSource::make("v_t", Node::List{GND, n3}, Complex(0, 0));
		auto vs = VoltageSourceNorton::make("v_s", Node::List{GND, n1}, Complex(10000, 0), 1);
		auto l1 = Inductor::make("l_1", Node::List{n1, n2}, 0.1);
		auto r1 = Resistor::make("r_1", Node::List{n2, n3}, 1);

		comps = ComponentBase::List{evs, vs, l1, r1};
		nodes = Node::List{GND, n1, n2, n3};

		shmem.registerControlledAttribute(evs->findAttribute<Complex>("voltage_ref"), 1.0, 0, 1);
		shmem.registerExportedAttribute(evs->findAttribute<Complex>("comp_current"), 1.0, 0, 1);
	}
	else if (String(argv[1]) == "1") {
		// Nodes
		auto n4 = Node::make("n4");

		// Components
		auto ecs = CurrentSource::make("v_s", Node::List{GND, n4}, Complex(0, 0));
		auto r2A = Resistor::make("r_2", Node::List{GND, n4}, 10);
		auto r2B = Resistor::make("r_2", Node::List{GND, n4}, 8);

		comps = ComponentBase::List{ecs, r2A};
		comps2 = ComponentBase::List{ecs, r2B};
		nodes = Node::List{GND, n4};

		shmem.registerControlledAttribute(ecs->findAttribute<Complex>("current_ref"), 1.0, 0, 1);
		shmem.registerExportedAttribute(ecs->findAttribute<Complex>("comp_voltage"), 1.0, 0, 1);
	}
	else {
		std::cerr << "invalid test number" << std::endl;
		std::exit(1);
	}

	String simName = "ShmemDistributed";
	Real timeStep = 0.001;

	auto sys1 = SystemTopology(50, nodes, comps);

	auto sim = RealTimeSimulation(simName + argv[1], sys1, timeStep, 20);
	sim.addInterface(&shmem);

	if (String(argv[1]) == "1") {
		auto sys2 = SystemTopology(50, comps2);

		sim.addSystemTopology(sys2);
		sim.setSwitchTime(10, 1);
	}

	sim.run(true);

	return 0;
}
