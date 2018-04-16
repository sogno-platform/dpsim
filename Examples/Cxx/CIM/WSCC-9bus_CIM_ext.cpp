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

#include "DPsim_MNA.h"
#include "cps/CIM/Reader.h"
#include "cps/Interfaces/ShmemInterface.h"

using namespace DPsim;
using namespace CPS::Components::DP;

int main(int argc, char *argv[]) {
	// Specify CIM files
#ifdef _WIN32
	String path("..\\..\\..\\..\\dpsim\\Examples\\CIM\\WSCC-09_Neplan_RX\\");
#elif defined(__linux__) || defined(__APPLE__)
	String path("../../../dpsim/Examples/CIM/IEEE-09_Neplan_RX/");
#endif
	std::list<String> filenames = {
		path + "WSCC-09_Neplan_RX_DI.xml",
		path + "WSCC-09_Neplan_RX_EQ.xml",
		path + "WSCC-09_Neplan_RX_SV.xml",
		path + "WSCC-09_Neplan_RX_TP.xml"
	};

	// Read CIM data
	CIM::Reader reader(50, Logger::Level::INFO, Logger::Level::INFO);
	reader.addFiles(filenames);
	reader.parseFiles();
	SystemTopology system = reader.getSystemTopology();

	// Extend system with current source
	auto ecs = CurrentSource::make("v_s", 0, GND, Complex(0, 0));
	system.mComponents.push_back(ecs);

	// Add shared memory interface
	struct shmem_conf conf;
	conf.samplelen = 4;
	conf.queuelen = 1024;
	conf.polling = false;
	in = "/dpsim10";
	out = "/dpsim01";
	ShmemInterface shmem(in, out, &conf);
	shmem.registerControllableSource(ecs, GND, 0);
	shmem.registerExportedVoltage(ecs, 0, 1);

	MnaSimulation sim("CIM", system, 0.0001, 0.1, SimulationType::DP, Logger::Level::DEBUG);
	sim.run();

	return 0;
}
