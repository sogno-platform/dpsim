/** Example of shared memory interface
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

#include <DPsim.h>

using namespace DPsim;
using namespace CPS::DP;
using namespace CPS::DP::Ph1;

int main(int argc, char* argv[])
{
	// Nodes
	auto n1 = Node::make("n1");
	auto n2 = Node::make("n2");
	auto n3 = Node::make("n3");
	auto n4 = Node::make("n4");

	// Components
	auto vs = VoltageSourceNorton::make("v_s");
	auto l1 = Inductor::make("l_1");
	auto r1 = Resistor::make("r_1");
	auto r2A = Resistor::make("r_2");
	auto r2B = Resistor::make("r_2");
	auto sw = Ph1::Switch::make("sw");

	// Parameters
	vs->setParameters(Complex(10000, 0), 1);
	l1->setParameters(0.1);
	r1->setParameters(1);
	r2A->setParameters(10);
	r2B->setParameters(8);
	sw->setParameters(1e9, 0.1, false);

	// Topology
	vs->connect({ Node::GND, n1 });
	l1->connect({ n1, n2 });
	r1->connect({ n2, n3 });
	r2A->connect({ n3, Node::GND });
	sw->connect({ n3, n4 });
	r2B->connect({ n4, Node::GND });

	auto nodes = SystemNodeList{Node::GND, n1, n2, n3, n4};

	auto sys = SystemTopology(50, nodes, SystemComponentList{vs, l1, r1, sw, r2A, r2B});

	auto sim = Simulation("ShmemDistributedRef", sys, 0.001, 20);

	auto evt = SwitchEvent::make(10, sw, true);

	sim.addEvent(evt);

	sim.run();

	return 0;
}
