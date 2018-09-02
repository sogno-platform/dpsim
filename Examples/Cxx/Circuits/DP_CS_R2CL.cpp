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
using namespace CPS::DP;
using namespace CPS::DP::Ph1;

int main(int argc, char* argv[]) {
	// Nodes
	auto n1 = Node::make("n1");
	auto n2 = Node::make("n2");

	// Components
	auto cs = CurrentSource::make("cs");
	auto r1 = Resistor::make("r_1");
	auto c1 = Capacitor::make("c_1");
	auto l1 = Inductor::make("l_1");
	auto r2 = Resistor::make("r_2");

	// Topology
	cs->connect({ Node::GND, n1 });
	r1->connect({ n1, Node::GND });
	c1->connect({ n1, n2 });
	l1->connect({ n2, Node::GND });
	r2->connect({ n2, Node::GND });

	cs->setParameters(10);
	r1->setParameters(1);
	c1->setParameters(0.001);
	l1->setParameters(0.001);
	r2->setParameters(1);

	// Define system topology
	auto sys = SystemTopology(50, SystemNodeList{n1, n2}, SystemComponentList{cs, r1, c1, l1, r2});

	// Define simulation scenario
	Real timeStep = 0.001;
	Real finalTime = 0.1;
	String simName = "DP_CS_R2CL";

	Simulation sim(simName, sys, timeStep, finalTime);
	sim.run();

	return 0;
}
