/** Reference Circuits
 *
 * @author Markus Mirz <mmirz@eonerc.rwth-aachen.de>
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
using namespace CPS::DP::Ph1;

int main(int argc, char* argv[]) {
	// Nodes
	auto n1 = Node::make("n1");
	auto n2 = Node::make("n2");
	auto n3 = Node::make("n3");

	// Components
	auto vs = VoltageSourceNorton::make("v_s");
	auto rl = Resistor::make("r_line");
	auto ll = Inductor::make("l_line");

	// Topology
	vs->connect({n1, GND});
	rl->connect({n1, n2});
	ll->connect({n2, n3});

	// Parameters
	vs->setParameters(Complex(10000, 0), 1);
	rl->setParameters(1)
	ll->setParameters(1)

	// Define system topology
	SystemTopology system0(50, SystemTopologyNode{n1, n2, n3, GND}, SystemTopologyComponent{vs, rl, ll});

	SystemTopology system1 = system0;
	SystemTopology system2 = system0;
	system1.addComponent(Resistor::make("r_load", 3, 0, 1000));
	system2.addComponent(Resistor::make("r_load", 3, 0, 800));

	// Define simulation scenario
	Real timeStep = 0.001;
	Real finalTime = 0.3;
	String simName = "DP_ResVC_RxLine1_" + std::to_string(timeStep);

	Simulation sim(simName, system1, timeStep, finalTime);
	sim.addSystemTopology(system2);
	sim.setSwitchTime(0.1, 1);

	sim.run();

	return 0;
}
