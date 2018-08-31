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
	auto n4 = Node::make("n4");

	// Components
	auto vs = VoltageSource::make("vs");
	vs->setParameters(10 * sin(2 * PI * 50));  //V_in(t) = 10*sin(w*t)
	vs->setNodes(Node::List{ Node::GND, n1 });
	auto r1 = Resistor::make("r_1");
	r1->setParameters(1);
	r1->setNodes(Node::List{ n1, n2 });
	auto l1 = Inductor::make("l_1");
	l1->setParameters(0.02);
	l1->setNodes(Node::List{ n2, n3 });
	auto l2 = Inductor::make("l_2");
	l2->setParameters(0.1);
	l2->setNodes(Node::List{ n3, Node::GND });
	auto l3 = Inductor::make("l_3");
	l3->setParameters(0.05);
	l3->setNodes(Node::List{ n3, n4 });
	auto r2 = Resistor::make("r_2");
	r2->setParameters(2);
	r2->setNodes(Node::List{ n4, Node::GND });

	// Define system topology
	auto sys = SystemTopology(50, SystemNodeList{n1, n2, n3, n4}, SystemComponentList{vs, r1, l1, l2, l3, r2});
		
	// Define simulation scenario
	Real timeStep = 0.001;
	Real finalTime = 0.1;
	String simName = "EMT_IdealVS_R2L3";

	Simulation sim(simName, sys, timeStep, finalTime, Domain::EMT);
	sim.run();

	return 0;
}
