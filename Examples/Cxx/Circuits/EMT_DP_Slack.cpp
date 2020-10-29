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
using namespace CPS::DP;

void simElements() {
	Real timeStep = 0.00005;
	Real finalTime = 1;
	String simName = "DP_Slack_Elements";
	Logger::setLogDir("logs/"+simName);

	// Nodes
	auto n1 = SimNode::make("n1");

	// Components
	auto vs = Ph1::VoltageSource::make("v_1");
	vs->setParameters(CPS::Math::polar(100000, 0));

	auto load = Ph1::Resistor::make("R_load");
	load->setParameters(10000);

	// Topology
	vs->connect({ SimNode::GND, n1 });
	load->connect({ n1, SimNode::GND });

	auto sys = SystemTopology(50,
		SystemNodeList{n1},
		SystemComponentList{vs, load});

	// Logging
	auto logger = DataLogger::make(simName);
	logger->addAttribute("v1", n1->attribute("v"));
	logger->addAttribute("i1", vs->attribute("i_intf"));

	Simulation sim(simName);
	sim.setSystem(sys);
	sim.setTimeStep(timeStep);
	sim.setFinalTime(finalTime);
	sim.addLogger(logger);

	sim.run();
}

void simComponent() {
	Real timeStep = 0.00005;
	Real finalTime = 1;
	String simName = "DP_Slack_Component";
	Logger::setLogDir("logs/"+simName);

	// Nodes
	auto n1 = SimNode::make("n1");

	// Components
	auto sl = Ph1::NetworkInjection::make("v_1");
	sl->setParameters(CPS::Math::polar(100000, 0));

	auto load = Ph1::Resistor::make("R_load");
	load->setParameters(10000);

	// Topology
	sl->connect({ n1 });
	load->connect({ n1, SimNode::GND });

	auto sys = SystemTopology(50,
		SystemNodeList{n1},
		SystemComponentList{sl, load});

	// Logging
	auto logger = DataLogger::make(simName);
	logger->addAttribute("v1", n1->attribute("v"));
	logger->addAttribute("i1", sl->attribute("i_intf"));

	Simulation sim(simName);
	sim.setSystem(sys);
	sim.setTimeStep(timeStep);
	sim.setFinalTime(finalTime);
	sim.addLogger(logger);

	sim.run();
}

int main(int argc, char* argv[]) {
	simElements();
    simComponent();
}