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
using namespace CPS::EMT;
using namespace CPS::EMT::Ph1;

int main(int argc, char* argv[]) {
	// Nodes
	auto n1 = Node::make("n1");
	auto n2 = Node::make("n2");
	auto n3 = Node::make("n3");

	// Components
	auto vin = VoltageSource::make("v_in");
	auto r1 = Resistor::make("r_1");
	auto r2 = Resistor::make("r_2");
	auto r3 = Resistor::make("r_3");

	// Topology
	vin->connect({ n1, n2 });
	r1->connect({ n1, Node::GND });
	r2->connect({ n2, Node::GND });
	r3->connect({ n2, Node::GND });

	// Parameters
	vin->setParameters(10);
	r1->setParameters(5);
	r2->setParameters(10);
	r3->setParameters(2);

	// Define system topology
	SystemTopology system(50, SystemNodeList{n1, n2, n3, Node::GND}, SystemComponentList{vin, r1, r2, r3});

	// Define simulation scenario
	Real timeStep = 0.00005;
	Real finalTime = 0.2;
	String simName = "EMT_IdealVS_R1";

	Simulation sim(simName, system, timeStep, finalTime, Domain::EMT);
	sim.run();

	return 0;
}
