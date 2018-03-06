/** CIM Test
 *
 * @author Markus Mirz <mmirz@eonerc.rwth-aachen.de>
 * @copyright 2017, Institute for Automation of Complex Power Systems, EONERC
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

#include "CIM/Reader.h"
#include "Simulation.h"

using namespace DPsim;

static int testCIMReader(std::list<String> filenames) {
	Simulation sim("CIM", filenames, 50, 0.0001, 0.1, Logger::Level::DEBUG, SimulationType::DP);
	sim.run();
	return 0;
}

static int readFixedCIMFiles_IEEE9bus() {
#ifdef _WIN32
	String path("..\\..\\..\\..\\dpsim\\Examples\\CIM\\WSCC-09_Neplan_RX\\");
#elif defined(__linux__)
	String path("../../../dpsim/Examples/CIM/IEEE-09_Neplan_RX/");
#else
  #error "Unkown platform"
#endif
	
	std::list<String> filenames = {
		path + "WSCC-09_Neplan_RX_DI.xml",
		path + "WSCC-09_Neplan_RX_EQ.xml",
		path + "WSCC-09_Neplan_RX_SV.xml",
		path + "WSCC-09_Neplan_RX_TP.xml"
	};
	testCIMReader(filenames);
	return 0;
}

static int readCIMFilesFromInput(int argc, char *argv[]) {
	std::list<String> filenames;

	for (int i = 1; i < argc; i++) {
		std::cout << "Adding file: " << argv[i] << std::endl;
		filenames.push_back(String(argv[i]));
	}

	return testCIMReader(filenames);
}

int main(int argc, char *argv[]) {
	return argc < 2
		? readFixedCIMFiles_IEEE9bus()
		: readCIMFilesFromInput(argc, argv);
}
