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
	Real finalTime = 0.2;
	String simName = "DP_VS_RL_f60";
	Logger::setLogDir("logs/"+simName);

	// Nodes
	auto n1 = Node::make("n1");
	auto n2 = Node::make("n2");
	auto n3 = Node::make("n3");

	// Components
	auto vs = VoltageSource::make("vs");
	vs->setParameters(Complex(230, 0), 10);	
	auto rline = Resistor::make("r_line");
	rline->setParameters(1);	
	auto lline = Inductor::make("l_line");
	lline->setParameters(0.02);
	auto rload = Resistor::make("r_load");
	rload->setParameters(10);

	// Connections
	vs->connect({ Node::GND, n1 });
	rline->connect({ n1, n2 });
	lline->connect({ n2, n3 });
	rload->connect({ n3, Node::GND });

	// Define system topology
	auto sys = SystemTopology(50, 
		SystemNodeList{n1, n2, n3}, 
		SystemComponentList{vs, rline, lline, rload});

	// Logger
	auto logger = DataLogger::make(simName);
	logger->addAttribute("v1", n1->attributeMatrixComp("v"));
	logger->addAttribute("v2", n2->attributeMatrixComp("v"));
	logger->addAttribute("v3", n3->attributeMatrixComp("v"));
	logger->addAttribute("i_line", rline->attributeMatrixComp("i_intf"));

	Simulation sim(simName, Logger::Level::INFO);
	sim.setSystem(sys);
	sim.setTimeStep(timeStep);
	sim.setFinalTime(finalTime);
	sim.initialize();

	sim.addLogger(logger);

	sim.run();

	return 0;
}
