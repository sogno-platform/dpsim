/**
 * @copyright 2017 Institute for Automation of Complex Power Systems, EONERC
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

void simElements() {
	Real timeStep = 0.00005;
	Real finalTime = 0.1;
	String simName = "DP_PiLine_Elements";
	Logger::setLogDir("logs/"+simName);

	// Nodes
	auto n1 = Node::make("n1");
	auto n2 = Node::make("n2");
	auto vn1 = Node::make("vn1");

	// Components
	auto vs = Ph1::VoltageSource::make("v_1");
	vs->setParameters(CPS::Math::polar(100000, -PI/2.));

	Real resistance = 5;
	Real inductance = 0.16;
	Real capacitance = 1.0e-6 / 2.;
	auto res = Ph1::Resistor::make("R_line");
	res->setParameters(resistance);
	auto ind = Ph1::Inductor::make("L_line");
	ind->setParameters(inductance);
	auto cap1 = Ph1::Capacitor::make("Cp_1");
	cap1->setParameters(capacitance);
	auto cap2 = Ph1::Capacitor::make("Cp_2");
	cap2->setParameters(capacitance);

	auto load = Ph1::Resistor::make("R_load");
	load->setParameters(10000);

	// Topology
	vs->connect({ Node::GND, n1 });
	res->connect({n1, vn1});
	ind->connect({vn1, n2});
	cap1->connect({n1, Node::GND});
	cap2->connect({n2, Node::GND});
	load->connect({ n2, Node::GND });

	auto sys = SystemTopology(50,
		SystemNodeList{n1, n2, vn1},
		SystemComponentList{vs, res, ind, cap1, cap2, load});
		//SystemComponentList{vs, res, ind, load});

	// Logging
	auto logger = DataLogger::make(simName);
	logger->addAttribute("v1", n1->attribute("v"));
	logger->addAttribute("v2", n2->attribute("v"));

	Simulation sim(simName);
	sim.setSystem(sys);
	sim.setTimeStep(timeStep);
	sim.setFinalTime(finalTime);
	sim.addLogger(logger);

	sim.run();
}

void simPiLine() {
	Real timeStep = 0.00005;
	Real finalTime = 0.1;
	String simName = "DP_PiLine_Component";
	Logger::setLogDir("logs/"+simName);

	// Nodes
	auto n1 = Node::make("n1");
	auto n2 = Node::make("n2");

	// Components
	auto vs = Ph1::VoltageSource::make("v_1");
	vs->setParameters(CPS::Math::polar(100000, -PI/2.));

	// R=5, X=50 (L=0.16), B=0.003 (C=1e-6)
	auto line = Ph1::PiLine::make("Line");
	line->setParameters(5, 0.16, 1e-6);

	auto load = Ph1::Resistor::make("R_load");
	load->setParameters(10000);

	// Topology
	vs->connect({ Node::GND, n1 });
	line->connect({ n1, n2 });
	load->connect({ n2, Node::GND });

	auto sys = SystemTopology(50,
		SystemNodeList{n1, n2},
		SystemComponentList{vs, line, load});

	// Logging
	auto logger = DataLogger::make(simName);
	logger->addAttribute("v1", n1->attribute("v"));
	logger->addAttribute("v2", n2->attribute("v"));

	Simulation sim(simName);
	sim.setSystem(sys);
	sim.setTimeStep(timeStep);
	sim.setFinalTime(finalTime);
	sim.addLogger(logger);

	sim.run();
}

void simPiLineDiakoptics() {
	Real timeStep = 0.00005;
	Real finalTime = 0.1;
	String simName = "DP_PiLine_Diakoptics";
	Logger::setLogDir("logs/"+simName);

	// Nodes
	auto n1 = Node::make("n1");
	auto n2 = Node::make("n2");

	// Components
	auto vs = Ph1::VoltageSource::make("v_1");
	vs->setParameters(CPS::Math::polar(100000, -PI/2.));

	// R=5, X=50 (L=0.16), B=0.003 (C=1e-6)
	auto line = Ph1::PiLine::make("Line");
	line->setParameters(5, 0.16, 1e-6);

	auto load = Ph1::Resistor::make("R_load");
	load->setParameters(10000);

	// Topology
	vs->connect({ Node::GND, n1 });
	line->connect({ n1, n2 });
	load->connect({ n2, Node::GND });

	auto sys = SystemTopology(50,
		SystemNodeList{n1, n2},
		SystemComponentList{vs, load});
	sys.addTearComponent(line);

	// Logging
	auto logger = DataLogger::make(simName);
	logger->addAttribute("v1", n1->attribute("v"));
	logger->addAttribute("v2", n2->attribute("v"));

	Simulation sim(simName);
	sim.setSystem(sys);
	sim.setTearingComponents(sys.mTearComponents);
	sim.setTimeStep(timeStep);
	sim.setFinalTime(finalTime);
	sim.addLogger(logger);

	sim.run();
}

int main(int argc, char* argv[]) {
	simElements();
	simPiLine();
	simPiLineDiakoptics();
}
