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
		for (int i = 2; i < copies+2; i++) {
			nodeNames.push_back(orig_node + "_" + std::to_string(i));
		}
		nodeNames.push_back(orig_node);
		// if only a single copy is added, it doesn't really make sense to
		// "close the ring" by adding another line
		int nlines = copies == 1 ? 1 : copies+1;
		for (int i = 0; i < nlines; i++) {
			auto line = Signal::DecouplingLine::make("dline" + orig_node + std::to_string(i), sys.node<Node<Complex>>(nodeNames[i]), sys.node<Node<Complex>>(nodeNames[i+1]), resistance, inductance, capacitance, Logger::Level::DEBUG);
			sys.addComponent(line);
			sys.addComponents(line->getLineComponents());
			counter++;
		}
	}
}

int main(int argc, char *argv[]) {
#ifdef _WIN32
	String path("..\\..\\..\\..\\dpsim\\Examples\\CIM\\WSCC-09_RX\\");
#elif defined(__linux__) || defined(__APPLE__)
	String path("Examples/CIM/WSCC-09_RX/");
#endif

	std::list<String> filenames = {
		path + "WSCC-09_RX_DI.xml",
		path + "WSCC-09_RX_EQ.xml",
		path + "WSCC-09_RX_SV.xml",
		path + "WSCC-09_RX_TP.xml"
	};

	String simName = "WSCC-9bus";

	CIM::Reader reader(simName, Logger::Level::NONE, Logger::Level::NONE);
	SystemTopology sys = reader.loadCIM(60, filenames);

	multiply_connected(sys, 15, 12.5, 0.16, 1e-6);
	//sys.printGraph(sys.topologyGraph());
	Simulation sim(simName, sys, 0.0001, 0.5,
		Domain::DP, Solver::Type::MNA, Logger::Level::NONE, false);
	sim.setScheduler(std::make_shared<OpenMPLevelScheduler>(4));
	//std::ofstream of("graph.svg");
	//sim.renderDependencyGraph(of);
	sim.run();

	return 0;
}
