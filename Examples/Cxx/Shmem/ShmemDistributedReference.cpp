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

int main(int argc, char* argv[])
{
	// Nodes
	auto n1 = Node::make("n1");
	auto n2 = Node::make("n2");
	auto n3 = Node::make("n3");

	// Components
	auto vs = VoltageSourceNorton::make("v_s", Node::List{GLOBALGND, n1}, Complex(10000, 0), 1);
	auto l1 = Inductor::make("l_1", Node::List{n1, n2}, 0.1);
	auto r1 = Resistor::make("r_1", Node::List{n2, n3}, 1);

	auto r2A = Resistor::make("r_2", Node::List{n3, GLOBALGND}, 10);
	auto r2B = Resistor::make("r_2", Node::List{n3, GLOBALGND}, 8);

	auto nodes = Node::List{GLOBALGND, n1, n2, n3};

	auto sys1 = SystemTopology(50, nodes, ComponentBase::List{vs, l1, r1, r2A});
	auto sys2 = SystemTopology(50, nodes, ComponentBase::List{vs, l1, r1, r2B});

	auto sim = Simulation("ShmemDistributedRef", sys1, 0.001, 20);
	sim.addSystemTopology(sys2);
	sim.setSwitchTime(10, 1);
	sim.run();

	return 0;
}
