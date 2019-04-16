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
	// Define simulation scenario
	Real timeStep = 0.0001;
	Real finalTime = 0.1;
	String simName = "DP_VS_RL2";
	Logger::setLogDir("logs/"+simName);

	// Nodes
	auto n1 = Node::make("n1");
	auto n2 = Node::make("n2");
	auto n3 = Node::make("n3");

	// Components
	auto vs = VoltageSource::make("vs");
	vs->setParameters(Complex(1000, 0));
	vs->connect(Node::List{ Node::GND, n1 });

	auto l_line = Inductor::make("l_line", Logger::Level::DEBUG);
	l_line->setParameters(0.1);
	l_line->connect(Node::List{ n1, n2 });

	auto r_line = Resistor::make("r_line");
	r_line->setParameters(1);
	r_line->connect(Node::List{ n2, n3 });

	auto r_load = Resistor::make("r_load");
	r_load->setParameters(100);
	r_load->connect(Node::List{ n3, Node::GND });

	// Define system topology
	auto sys = SystemTopology(50, SystemNodeList{n1, n2, n3}, SystemComponentList{vs, l_line, r_line, r_load});

	// Logger
	auto logger = DataLogger::make(simName);
	logger->addAttribute("v1", n1->attribute("v"));
	logger->addAttribute("v2", n2->attribute("v"));
	logger->addAttribute("i12", r_load->attribute("i_intf"));

	Simulation sim(simName, sys, timeStep, finalTime, Domain::DP, Solver::Type::MNA, Logger::Level::DEBUG);
	sim.addLogger(logger);

	sim.run();

	return 0;
}
