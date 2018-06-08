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
	// Same circuit as above, but now with realtime support.
	ComponentBase::List comps;

	Interface::Config conf;
	conf.samplelen = 4;
	conf.queuelen = 1024;
	conf.polling = false;

	// Nodes
	auto n1 = Node::make("n1");
	auto n2 = Node::make("n2");
	auto n3 = Node::make("n3");
	auto n4 = Node::make("n4");

	// Components
	auto evs = VoltageSource::make("v_s", Node::List{GND, n1}, Complex(0, 0));
	auto rs =  Resistor::make("r_s", Node::List{n1, n2}, 1);
	auto rl =  Resistor::make("r_line", Node::List{n2, n3}, 1);
	auto ll =  Inductor::make("l_line", Node::List{n3, n4}, 1);
	auto rL =  Resistor::make("r_load", Node::List{n4, GND}, 1000);

	Interface intf("/villas1-in", "/villas1-out", &conf);
	intf.addImport(evs->findAttribute<Complex>("voltage_ref"), 1.0, 0, 1);
	intf.addExport(evs->findAttribute<Complex>("comp_current"), 1.0,  0, 1);

	Real timeStep = 0.001;
	auto sys = SystemTopology(50, Node::List{GND, n1, n2, n3, n4}, ComponentBase::List{evs, rs, rl, ll, rL});
	auto sim = RealTimeSimulation("ShmemRealTime", sys, timeStep, 5.0);

	sim.addInterface(&intf);
	sim.run(false);

	return 0;
}
