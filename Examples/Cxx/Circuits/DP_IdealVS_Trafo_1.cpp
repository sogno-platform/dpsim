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
	auto n3 = Node::make("n3");

	// Components
	auto v1 = VoltageSource::make("v_1");
//	auto l1 = Inductor::make("l_1");
//	auto r2 = Resistor::make("r_2");
	auto t1 = Transformer::make("trafo_1");
	auto r1 = Resistor::make("r_1");

	// Topology
	v1->connect({ Node::GND, n1 });
//	l1->connect({ n1, n2 });
//	r2->connect({ n2, Node::GND });
	t1->connect({ n1, n2 });
	r1->connect({ n2, Node::GND });

	// Parameters
	v1->setParameters(CPS::Math::polarDeg(100., 0 * -90.));
//	l1->setParameters(0.1);
//	r2->setParameters(1);
	t1->setParameters(10, 0, 0, 0.1);
	r1->setParameters(1);

	// Define system topology
	SystemTopology system(50, SystemNodeList{n1, n2, n3, Node::GND}, SystemComponentList{v1, t1, r1});

	// Define simulation scenario
	Real timeStep = 0.00005;
	Real finalTime = 0.2;
	String simName = "DP_IdealVS_Trafo_" + std::to_string(timeStep);

	Simulation sim(simName, system, timeStep, finalTime);
	sim.run();

	return 0;
}
