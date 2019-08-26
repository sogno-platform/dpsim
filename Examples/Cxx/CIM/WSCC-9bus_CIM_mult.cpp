/**
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

#include <iostream>
#include <list>
#include <fstream>

#include <DPsim.h>

using namespace DPsim;
using namespace CPS;

void multiply_connected(SystemTopology& sys, int copies, Real resistance, Real inductance, Real capacitance) {
	sys.multiply(copies);
	int counter = 0;
	std::vector<String> nodes = {"BUS5", "BUS8", "BUS6"};

	for (auto orig_node : nodes) {
		std::vector<String> nodeNames{orig_node};
		for (int i = 2; i <= copies+1; i++) {
			nodeNames.push_back(orig_node + "_" + std::to_string(i));
		}
		nodeNames.push_back(orig_node);
		// if only a single copy is added, it doesn't really make sense to
		// "close the ring" by adding another line
		int nlines = copies == 1 ? 1 : copies+1;

		for (int i = 0; i < nlines; i++) {
			auto line = Signal::DecouplingLine::make("dline_" + orig_node + "_" + std::to_string(i),
				sys.node<Node<Complex>>(nodeNames[i]),
				sys.node<Node<Complex>>(nodeNames[i+1]),
				resistance, inductance, capacitance, Logger::Level::debug);
			sys.addComponent(line);
			sys.addComponents(line->getLineComponents());
			counter++;
		}
	}
}

int main(int argc, char *argv[]) {

	// Find CIM files
	std::list<fs::path> filenames;
	if (argc <= 1) {
		filenames = Utils::findFiles({
			"WSCC-09_RX_DI.xml",
			"WSCC-09_RX_EQ.xml",
			"WSCC-09_RX_SV.xml",
			"WSCC-09_RX_TP.xml"
		}, "Examples/CIM/WSCC-09_RX", "CIMPATH");
	}
	else {
		filenames = std::list<fs::path>(argv + 1, argv + argc);
	}

	String simName = "WSCC-9bus";

	CIM::Reader reader(simName, Logger::Level::off, Logger::Level::off);
	SystemTopology sys = reader.loadCIM(60, filenames);

	multiply_connected(sys, 15, 12.5, 0.16, 1e-6);

	Simulation sim(simName, sys, 0.0001, 0.5,
		Domain::DP, Solver::Type::MNA, Logger::Level::off, false);
	sim.setScheduler(std::make_shared<OpenMPLevelScheduler>(4));

	//std::ofstream of1("topology_graph.svg");
	//sys.topologyGraph().render(of1));

	//std::ofstream of2("dependency_graph.svg");
	//sim.dependencyGraph().render(of2);

	sim.run();

	return 0;
}
