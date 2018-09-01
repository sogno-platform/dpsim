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
	auto vs = VoltageSource::make("vs");
	vs->setParameters(10);
	vs->connect({ Node::GND, n1 });
	auto r1 = Resistor::make("r_1");
	r1->setParameters(1);
	r1->connect({ n1, n2 });
	auto r2 = Resistor::make("r_2");
	r2->setParameters(1);
	r2->connect({ n2, Node::GND });
	auto r3 = Resistor::make("r_3");
	r3->setParameters(10);
	r3->connect({ n2, n3 });
	auto r4 = Resistor::make("r_4");
	r4->setParameters(5);
	r4->connect({ n3, Node::GND });
	auto cs = CurrentSource::make("cs");
	cs->setParameters(1);
	cs->connect({ Node::GND, n3 });

	// Define system topology
	auto sys = SystemTopology(50, SystemNodeList{n1, n2, n3}, SystemComponentList{vs, r1, r2, r3, r4, cs});
		
	// Define simulation scenario
	Real timeStep = 0.001;
	Real finalTime = 0.1;
	String simName = "EMT_IdealVS_CS_R4";

	Simulation sim(simName, sys, timeStep, finalTime, Domain::EMT);
	sim.run();

	return 0;
}
