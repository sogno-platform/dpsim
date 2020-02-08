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

#include <DPsim.h>

using namespace DPsim;
using namespace CPS::DP;

int main(int argc, char *argv[]) {

	// Find CIM files
	std::list<fs::path> filenames;
	if (argc <= 1) {
		filenames = Utils::findFiles({
			"WSCC-09_RX_DI.xml",
			"WSCC-09_RX_EQ.xml",
			"WSCC-09_RX_SV.xml",
			"WSCC-09_RX_TP.xml"
		}, "Examples/CIM/grid-data/WSCC-09/WSCC-09_RX", "CIMPATH");
	}
	else {
		filenames = std::list<fs::path>(argv + 1, argv + argc);
	}

	String simName = "WSCC-9bus";
	Logger::setLogDir("logs/"+simName);

	CPS::CIM::Reader reader(simName, Logger::Level::debug, Logger::Level::off);
	SystemTopology sys = reader.loadCIM(60, filenames);

	// Logging
	auto logger = DataLogger::make(simName);
	logger->addAttribute("v1", sys.node<Node>("BUS1")->attribute("v"));
	logger->addAttribute("v2", sys.node<Node>("BUS2")->attribute("v"));
	logger->addAttribute("v3", sys.node<Node>("BUS3")->attribute("v"));
	logger->addAttribute("v4", sys.node<Node>("BUS4")->attribute("v"));
	logger->addAttribute("v5", sys.node<Node>("BUS5")->attribute("v"));
	logger->addAttribute("v6", sys.node<Node>("BUS6")->attribute("v"));
	logger->addAttribute("v7", sys.node<Node>("BUS7")->attribute("v"));
	logger->addAttribute("v8", sys.node<Node>("BUS8")->attribute("v"));
	logger->addAttribute("v9", sys.node<Node>("BUS9")->attribute("v"));

	Simulation sim(simName, sys, 0.0001, 0.1,
		Domain::DP, Solver::Type::MNA, Logger::Level::info, true);

	sim.addLogger(logger);
	sim.run();

	return 0;
}
