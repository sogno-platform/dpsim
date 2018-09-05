/*********************************************************************************
* @file
* @author Markus Mirz <mmirz@eonerc.rwth-aachen.de>
* @copyright 2017-2018, Institute for Automation of Complex Power Systems, EONERC
*
* CPowerSystems
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
using namespace CPS;

int main(int argc, char *argv[]) {
#ifdef _WIN32
	String path("..\\..\\..\\..\\dpsim\\Examples\\CIM\\WSCC-09_RX_Dyn\\");
#elif defined(__linux__) || defined(__APPLE__)
	String path("Examples/CIM/WSCC-09_RX_Dyn/");
#endif

	std::list<String> filenames = {
		path + "WSCC-09_RX_DI.xml",
		path + "WSCC-09_RX_EQ.xml",
		path + "WSCC-09_RX_SV.xml",
		path + "WSCC-09_RX_TP.xml"
	};

	String simName = "WSCC-9bus_dyn";

	CIM::Reader reader(simName, Logger::Level::DEBUG, Logger::Level::DEBUG);
	SystemTopology sys = reader.loadCIM(60, filenames);

	Simulation sim(simName, sys, 0.0001, 0.1,
		Domain::DP, Solver::Type::MNA, Logger::Level::DEBUG, true);
	sim.run();

	return 0;
}
