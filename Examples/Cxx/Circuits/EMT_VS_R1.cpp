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
	// Define simulation scenario
	Real timeStep = 0.0001;
	Real finalTime = 0.1;
	String simName = "EMT_VS_R1";

	// Nodes
	auto n1 = Node::make("n1");
	auto n2 = Node::make("n2");
	auto n3 = Node::make("n3");

	// Components
	auto vin = VoltageSource::make("v_in");
	vin->setParameters(10);
	auto r1 = Resistor::make("r_1");
	r1->setParameters(5);
	auto r2 = Resistor::make("r_2");
	r2->setParameters(10);
	auto r3 = Resistor::make("r_3");
	r3->setParameters(2);

	// Topology
	vin->connect({ n1, n2 });
	r1->connect({ n1, Node::GND });
	r2->connect({ n2, Node::GND });
	r3->connect({ n2, Node::GND });

	// Define system topology
	SystemTopology system(50, SystemNodeList{n1, n2, n3, Node::GND}, SystemComponentList{vin, r1, r2, r3});

	// Logging
	auto logger = DataLogger::make(simName);
	logger->addAttribute("v1", n1->attribute("voltage"));
	logger->addAttribute("v2", n2->attribute("voltage"));
	logger->addAttribute("v3", n3->attribute("voltage"));

	Simulation sim(simName, system, timeStep, finalTime, Domain::EMT);
	sim.addLogger(logger);

	sim.run();

	return 0;
}
