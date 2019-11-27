/** Reference Circuits
 *
 * @author Jan Dinkelbach <jdinkelbach@eonerc.rwth-aachen.de>
 * @copyright 2019, Institute for Automation of Complex Power Systems, EONERC
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
	Real timeStep = 1e-4;
	Real finalTime = 1e-3;
	String simName = "EMT_CS_RL1";
	Logger::setLogDir("logs/"+simName);

	// Nodes
	auto n1 = Node::make("n1", PhaseType::Single, std::vector<Complex>{ 2 });

	// Components
	auto cs = CurrentSource::make("cs", Logger::Level::info);
	cs->setParameters(Complex(10, 0), 0);
	auto r1 = Resistor::make("r_1");
	r1->setParameters(0.2);
	auto l1 = Inductor::make("l_1", Logger::Level::info);
	l1->setParameters(0.001);

	// Topology
	cs->connect(Node::List{ Node::GND, n1 });
	r1->connect(Node::List{ n1, Node::GND });
	l1->connect(Node::List{ n1, Node::GND });

	// Define system topology
	auto sys = SystemTopology(50, SystemNodeList{n1}, SystemComponentList{cs, r1, l1});

	// Logger
	auto logger = DataLogger::make(simName);
	logger->addAttribute("v1", n1->attribute("v"));
	logger->addAttribute("iR1", r1->attribute("i_intf"));
	logger->addAttribute("iL1", l1->attribute("i_intf"));

	Simulation sim(simName, sys, timeStep, finalTime, Domain::EMT);
	sim.addLogger(logger);

	sim.run();

	return 0;
}
