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
	String simName = "DP_Decoupling_Elements";
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

void simDecoupling() {
	Real timeStep = 0.00005;
	Real finalTime = 0.1;
	String simName = "DP_Decoupling_Wave";
	Logger::setLogDir("logs/"+simName);

	// Nodes
	auto n1 = Node::make("n1");
	auto n2 = Node::make("n2");

	// Components
	auto vs = Ph1::VoltageSource::make("Vsrc");
	vs->setParameters(CPS::Math::polar(100000, 0));

	Real resistance = 5;
	Real inductance = 0.16;
	Real capacitance = 1.0e-6;
	auto dline = CPS::Signal::DecouplingLine::make("DecLine", Logger::Level::debug);
	dline->setParameters(n1, n2, resistance, inductance, capacitance);

	auto load = Ph1::Resistor::make("R_load");
	load->setParameters(10000);

	// Topology
	vs->connect({ Node::GND, n1 });
	load->connect({ n2, Node::GND });

	auto sys = SystemTopology(50,
		SystemNodeList{n1, n2},
		SystemComponentList{vs, dline, load});
	sys.addComponents(dline->getLineComponents());

	// Logging
	auto logger = DataLogger::make(simName);
	logger->addAttribute("v1", n1->attribute("v"));
	logger->addAttribute("v2", n2->attribute("v"));
	logger->addAttribute("i1", vs->attribute("i_intf"));
	logger->addAttribute("i2", load->attribute("i_intf"));
	logger->addAttribute("i_src1", dline->attribute("i_src1"));
	logger->addAttribute("i_src2", dline->attribute("i_src2"));

	Simulation sim(simName);
	sim.setSystem(sys);
	sim.setTimeStep(timeStep);
	sim.setFinalTime(finalTime);
	sim.addLogger(logger);

	sim.run();
}

void simDecouplingEMT() {
	Real timeStep = 0.00005;
	Real finalTime = 0.1;
	String simName = "EMT_Decoupling_Wave";
	Logger::setLogDir("logs/"+simName);

	// Nodes
	auto n1 = CPS::EMT::Node::make("n1");
	auto n2 = CPS::EMT::Node::make("n2");

	// Components
	auto vs = CPS::EMT::Ph1::VoltageSource::make("Vsrc_emt");
	vs->setParameters(CPS::Math::polar(100000, 0), 50);

	Real resistance = 5;
	Real inductance = 0.16;
	Real capacitance = 1.0e-6;
	auto dline = CPS::Signal::DecouplingLineEMT::make("DecLine_emt", Logger::Level::debug);
	dline->setParameters(n1, n2, resistance, inductance, capacitance);

	auto load = CPS::EMT::Ph1::Resistor::make("R_load");
	load->setParameters(10000);

	// Topology
	vs->connect({ CPS::EMT::Node::GND, n1 });
	load->connect({ n2, CPS::EMT::Node::GND });

	auto sys = SystemTopology(50,
		SystemNodeList{n1, n2},
		SystemComponentList{vs, dline, load});
	sys.addComponents(dline->getLineComponents());

	// Logging
	auto logger = DataLogger::make(simName);
	logger->addAttribute("v1", n1->attribute("v"));
	logger->addAttribute("v2", n2->attribute("v"));
	logger->addAttribute("i1", vs->attribute("i_intf"));
	logger->addAttribute("i2", load->attribute("i_intf"));
	logger->addAttribute("i_src1", dline->attribute("i_src1"));
	logger->addAttribute("i_src2", dline->attribute("i_src2"));

	Simulation sim(simName);
	sim.setSystem(sys);
	sim.setTimeStep(timeStep);
	sim.setFinalTime(finalTime);
	sim.setDomain(CPS::Domain::EMT);
	sim.addLogger(logger);

	sim.run();
}

int main(int argc, char* argv[]) {
	simElements();
	simDecoupling();
	simDecouplingEMT();
}
