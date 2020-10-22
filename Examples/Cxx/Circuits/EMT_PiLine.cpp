/* Copyright 2017-2020 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 * DPsim
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *********************************************************************************/

#include <DPsim.h>

using namespace DPsim;
using namespace CPS::CIM;
using namespace CPS::EMT;
using namespace CPS::EMT::Ph3;

void simElements() {
	Real timeStep = 0.00005;
	Real finalTime = 1;
	String simName = "EMT_PiLine_Elements";
	Logger::setLogDir("logs/"+simName);

	// Nodes
	auto n1 = SimNode::make("n1", PhaseType::ABC);
	auto n2 = SimNode::make("n2", PhaseType::ABC);
	auto vn1 = SimNode::make("vn1", PhaseType::ABC);

	// Components
	auto vs = VoltageSource::make("v_1", Logger::Level::debug);
	vs->setParameters(CPS::Math::polar(100000, -PI/2.), 50);

	// Parametrization of components
	Real resistance = 5;
	Real inductance = 0.16;
	Real capacitance = 1.0e-6;
	Real conductance = 1e-6;

	auto res = Resistor::make("R_line", Logger::Level::debug);
	res->setParameters(Reader::singlePhaseParameterToThreePhase(resistance));
	auto ind = Inductor::make("L_line", Logger::Level::debug);
	ind->setParameters(Reader::singlePhaseParameterToThreePhase(inductance));
	auto cap1 = Capacitor::make("Cp_1", Logger::Level::debug);
	cap1->setParameters(Reader::singlePhaseParameterToThreePhase(capacitance/2.));
	auto cap2 = Capacitor::make("Cp_2", Logger::Level::debug);
	cap2->setParameters(Reader::singlePhaseParameterToThreePhase(capacitance/2.));
	auto con1 = Resistor::make("Gp_1", Logger::Level::debug);
	con1->setParameters(Reader::singlePhaseParameterToThreePhase(2./conductance));
	auto con2 = Resistor::make("Gp_2", Logger::Level::debug);
	con2->setParameters(Reader::singlePhaseParameterToThreePhase(2./conductance));

	auto load = Resistor::make("R_load", Logger::Level::debug);
	load->setParameters(Reader::singlePhaseParameterToThreePhase(10000));

	// Topology
	vs->connect({ SimNode::GND, n1 });
	res->connect({n1, vn1});
	ind->connect({vn1, n2});
	cap1->connect({n1, SimNode::GND});
	cap2->connect({n2, SimNode::GND});
	con1->connect({n1, SimNode::GND});
	con2->connect({n2, SimNode::GND});
	load->connect({ n2, SimNode::GND });

	auto sys = SystemTopology(50,
		SystemNodeList{n1, n2, vn1},
		SystemComponentList{vs, res, ind, cap1, cap2, con1, con2, load});
		//SystemComponentList{vs, res, ind, cap1, cap2, load});
		//SystemComponentList{vs, res, ind, load});

	// Logging
	auto logger = DataLogger::make(simName);
	logger->addAttribute("v1", n1->attribute("v"));
	logger->addAttribute("v2", n2->attribute("v"));

	Simulation sim(simName, Logger::Level::debug);
	sim.setSystem(sys);
	sim.setDomain(Domain::EMT);
	sim.setTimeStep(timeStep);
	sim.setFinalTime(finalTime);
	sim.addLogger(logger);

	sim.run();
}

void simPiLine() {
	Real timeStep = 0.00005;
	Real finalTime = 1;
	String simName = "EMT_PiLine_Component";
	Logger::setLogDir("logs/"+simName);

	// Nodes
	auto n1 = SimNode::make("n1", PhaseType::ABC);
	auto n2 = SimNode::make("n2", PhaseType::ABC);
	auto vn1 = SimNode::make("vn1", PhaseType::ABC);

	// Components
	auto vs = VoltageSource::make("v_1", Logger::Level::debug);
	vs->setParameters(CPS::Math::polar(100000, -PI/2.), 50);

	// Parametrization of components
	Real resistance = 5;
	Real inductance = 0.16;
	Real capacitance = 1.0e-6;
	Real conductance = 1e-6;

	auto line = PiLine::make("Line", Logger::Level::debug);
	line->setParameters(Reader::singlePhaseParameterToThreePhase(resistance), Reader::singlePhaseParameterToThreePhase(inductance), Reader::singlePhaseParameterToThreePhase(capacitance), Reader::singlePhaseParameterToThreePhase(conductance));

	auto load = Resistor::make("R_load", Logger::Level::debug);
	load->setParameters(Reader::singlePhaseParameterToThreePhase(10000));

	// Topology
	vs->connect({ SimNode::GND, n1 });
	line->connect({ n1, n2 });
	load->connect({ n2, SimNode::GND });

	auto sys = SystemTopology(50,
		SystemNodeList{n1, n2},
		SystemComponentList{vs, line, load});

	// Logging
	auto logger = DataLogger::make(simName);
	logger->addAttribute("v1", n1->attribute("v"));
	logger->addAttribute("v2", n2->attribute("v"));

	Simulation sim(simName, Logger::Level::debug);
	sim.setSystem(sys);
	sim.setDomain(Domain::EMT);
	sim.setTimeStep(timeStep);
	sim.setFinalTime(finalTime);
	sim.addLogger(logger);

	sim.run();
}

int main(int argc, char* argv[]) {
	simElements();
	simPiLine();
}
