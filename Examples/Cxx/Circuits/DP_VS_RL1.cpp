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
	Real timeStep = 0.0001;
	Real finalTime = 0.1;
	String simName = "DP_VS_RL1";
	Logger::setLogDir("logs/"+simName);

	// Nodes
	auto n1 = Node::make("n1");
	auto n2 = Node::make("n2");

	// Components
	auto vs = VoltageSource::make("vs");
	vs->setParameters(Complex(10, 0));
	auto r1 = Resistor::make("r_1");
	r1->setParameters(5);
	auto l1 = Inductor::make("l_1");
	l1->setParameters(0.02);

	// Connections
	vs->connect(Node::List{ Node::GND, n1 });
	r1->connect(Node::List{ n1, n2 });
	l1->connect(Node::List{ n2, Node::GND });

	// Define system topology
	auto sys = SystemTopology(50,
		SystemNodeList{n1, n2},
		SystemComponentList{vs, r1, l1});

	// Logger
	auto logger = DataLogger::make(simName);
	logger->addAttribute("v1", n1->attribute("v"));
	logger->addAttribute("v2", n2->attribute("v"));
	logger->addAttribute("i12", r1->attribute("i_intf"));

	Simulation sim(simName, sys, timeStep, finalTime);
	sim.addLogger(logger);

	sim.run();

	return 0;
}
