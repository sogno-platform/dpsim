/** Example of shared memory interface
 *
 * @author Steffen Vogel <stvogel@eonerc.rwth-aachen.de>
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

#include <fstream>

#include <DPsim.h>

using namespace DPsim;
using namespace CPS::DP;
using namespace CPS::DP::Ph1;

int main(int argc, char* argv[]) {
	// Very simple test circuit. Just a few resistors and an inductance.
	// Voltage is read from VILLASnode and current through everything is written back.
	String simName = "Shmem_Example";
	Logger::setLogDir("logs/"+simName);
	Real timeStep = 0.001;

	// Nodes
	auto n1 = SimNode::make("n1");
	auto n2 = SimNode::make("n2");
	auto n3 = SimNode::make("n3");
	auto n4 = SimNode::make("n4");

	// Components
	auto evs = VoltageSource::make("v_s");
	evs->setParameters(Complex(0, 0));
	auto rs =  Resistor::make("r_s");
	rs->setParameters(1);
	auto rl =  Resistor::make("r_line");
	rl->setParameters(1);
	auto ll =  Inductor::make("l_line");
	ll->setParameters(1);
	auto rL =  Resistor::make("r_load");
	rL->setParameters(1000);

	// Topology
	evs->connect({ SimNode::GND, n1 });
	rs->connect({ n1, n2 });
	rl->connect({ n2, n3 });
	ll->connect({ n3, n4 });
	rL->connect({ n4, SimNode::GND });

	auto sys = SystemTopology(50,
		SystemNodeList{SimNode::GND, n1, n2, n3, n4},
		SystemComponentList{evs, rs, rl, ll, rL});

#ifdef REALTIME
	RealTimeSimulation sim(simName, sys, timeStep, 1.0);
	Interface intf("/villas1-in", "/villas1-out", nullptr, false);
#else
	Simulation sim(simName, sys, timeStep, 1.0);
	Interface intf("/villas1-in", "/villas1-out");
#endif

	// Interface
	evs->setAttributeRef("V_ref", intf.importComplex(0));
	intf.exportComplex(evs->attributeMatrixComp("i_intf")->coeff(0, 0), 0);
	sim.addInterface(&intf);

	// Logger
	auto logger = DataLogger::make(simName);
	logger->addAttribute("v1", n1->attribute("v"));
	logger->addAttribute("v2", n2->attribute("v"));
	logger->addAttribute("v3", n3->attribute("v"));
	logger->addAttribute("v4", n4->attribute("v"));
	logger->addAttribute("V_ref", evs->attribute("V_ref"));
	logger->addAttribute("i_evs", evs->attributeMatrixComp("i_intf"), 1, 1);
	sim.addLogger(logger);

	sim.run();

	//std::ofstream of("task_dependencies.svg");
	//sim.dependencyGraph().render(of);
}
