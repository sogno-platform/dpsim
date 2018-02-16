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
	CIM::Reader reader(50, Logger::Level::INFO);

	for (String & filename : filenames) {
		if (!reader.addFile(filename))
			std::cout << "Failed to read file " << filename << std::endl;
	}

	try {
		reader.parseFiles();
	}
	catch (...) {
		std::cerr << "Failed to parse CIM files" << std::endl;
		return -1;
	}

	Component::List comps = reader.getComponents();

	Simulation sim("CIM", comps, 2 * PI * 50, 0.001, 0.3);

	sim.run();

	return 0;
}

static int readFixedCIMFiles_LineLoad() {
#ifdef _WIN32
	String path("..\\..\\..\\..\\dpsim\\Examples\\CIM\\Line_Load\\");
#elif defined(__linux__)
	String path("../../../dpsim/Examples/CIM/Line_Load/");
#else
  #error "Unkown platform"
#endif

	std::list<String> filenames = {
		path + "Line_Load.xml"
	};

	return testCIMReader(filenames);
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

	return testCIMReader(filenames);
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
